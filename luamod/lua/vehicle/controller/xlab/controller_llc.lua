-- Low-level physical actuation; one vehicle Lua VM owns this state.
-- Read prepareControlStep -> resolveSetpoints -> applySetpoints.
-- Manager inserts optional feedback between resolve/apply. Manual input bypasses
-- autonomous preview/feedback/slew; both paths may use the downstream roll filter.
-- Packet validation: execution_plan.lua. Cold-path calibration: llc_config.lua.
-- Tick helpers below use local bindings; no module loading or config parsing per tick.
--
-- Torque path priority (deterministic, mutually exclusive per step):
--   1. throttle_override  — direct [0,1], bypasses torque map + PI
--   2. torque map + inverse — needs ffi lib; skipped if no map
--
-- Torque maps use occupancy_rail: T(we, u, boost, ww_rad_s), inverted at live state.
--
-- Torque / wheel-speed composition (when not in throttle override):
--   torque_des     — feedforward torque [N·m] → inverse (nil = 0 FF)
--   omega_des      — rear wheel speed [m/s]; PI adds piTerm [N·m]
--   tCmd           = torque_des + clip(piTerm, t_min, t_max)  (unclamped if wr < 2 m/s)
--   piTerm         = pi_enable * (kp*e + ki*∫e), e = omega_des - omega
--   Omitting a scalar field clears it (replace semantics on each cmd).
--
-- Steering: independent PI on roadwheel [rad]; gated by control_mode authority.
-- Brake: external control asserts brake_des (zero if omitted). Manual fallback
--   preserves the processed pedal; parking brake remains under stock ownership.
local M = {}

local logTag = 'controller_llc'
local ffi = require('ffi')
local SteeringMap = require('vehicle.controller.xlab.steering_map')
local ExecutionPlan = require('vehicle.controller.xlab.execution_plan')
local Calibration = require('vehicle.controller.xlab.llc_config')
local steeringMap

local max = math.max
local min = math.min
local abs = math.abs
local floor = math.floor
local sqrt = math.sqrt
local huge = math.huge

local REF_DT = 0.01
local manualInput = nil
local inputSource = 0 -- GTState: 0=inactive/bypass, 1=manual, 2=external

-- With the bridge installed, actuator writes never overwrite stock driver events.
local function emitInput(name, value)
  if manualInput then
    manualInput.apply(name, value)
  else
    input.event(name, value, FILTER_AI)
  end
end

local CONTROL_AUTHORITY = {
  full = 'full',
  steering_only = 'steering_only',
  torque_only = 'torque_only',
}

local CMD_MODE = {
  none = 'none',
  scalar = 'scalar',
  trajectory = 'trajectory',
}

-- How trajectory x/y maps to wr/steer/torque setpoints:
--   next    — pick segment end waypoint (idx2)
--   lerp    — interpolate between idx1 and idx2 along projection t
--   closest — pick segment start waypoint (idx1)
--   preview — interpolate at projected phase + per-channel time/distance preview
local TRAJECTORY_PICK = {
  next = 'next',
  lerp = 'lerp',
  closest = 'closest',
  preview = 'preview',
}
local trajectoryPick = TRAJECTORY_PICK.next
-- Preview advances from measured spatial phase, not from packet receipt time.
local trajectoryPreview = {kind = 'time_s', steering = 0, torque = 0, wheel_speed = 0}
local trajectorySequence = 0
local selection = {}
local lastResolvedSteer = nil
local lastResolvedSequence = nil
local SELECTION_FIELDS = {
  'llc_trajectory_id', 'llc_segment', 'llc_fraction', 'llc_phase_time',
  'llc_phase_distance', 'llc_plan_age', 'llc_active', 'llc_preview_kind',
  'llc_query_steering', 'llc_query_torque', 'llc_query_wheel_speed',
  'llc_clamp_steering', 'llc_clamp_torque', 'llc_clamp_wheel_speed',
  'llc_steer_before_replace', 'llc_steer_after_replace',
}

-- ---------------------------------------------------------------------------
-- Torque map (ffi) — loaded once from OpenController payload (common.torqueMapPath)
local CDEF_OCCUPANCY = [[
  float drivetrain_forward_torque(float engine_speed_rads, float throttle_cmd,
                                  float boost_pressure, float rear_wheelspeed_rads);
  float drivetrain_inverse_throttle(float engine_speed_rads, float boost_pressure,
                                    float rear_wheelspeed_rads, float torque_cmd);
  float drivetrain_inverse_throttle_knots(float engine_speed_rads, float boost_pressure,
                                          float rear_wheelspeed_rads, float torque_cmd);
  int drivetrain_heads(float engine_speed_rads, float boost_pressure, float rear_wheelspeed_rads,
                       float *g_out, float *F_out);
  extern const float drivetrain_r_kin;
  extern const float drivetrain_r;
  extern const float drivetrain_brake_coeff_rear;
]]

-- Optional autonomous bound: brake * drivetrain torque <= overlap_torque_nm.
local brakeCoordination = nil
local torqueMapLib = nil
local restoreTorqueMapAfterReset = false
local torqueMapStem = nil
local initialized = false
local torqueMapCdefDone = false
local torqueMapRKin
local invertSanityLogged = false

local function loadTorqueMap(libPath)
  if torqueMapLib then return true end
  libPath = libPath or ''
  if libPath == '' then
    log('W', logTag, 'No torqueMapPath; torque / wheel-speed control disabled')
    return false
  end
  if not torqueMapCdefDone then
    ffi.cdef(CDEF_OCCUPANCY)
    torqueMapCdefDone = true
  end
  local ok, lib = pcall(ffi.load, libPath)
  if not ok or not lib then
    error('Torque map ffi.load failed: ' .. tostring(lib))
  end
  local r = tonumber(lib.drivetrain_r_kin)
  assert(r and r == r and r > 0.05 and r < huge, 'Invalid drivetrain_r_kin in torque map')
  torqueMapRKin = r
  torqueMapLib = lib
  log(
    'I',
    logTag,
    string.format(
      'Loaded torque map from %s r_kin=%.5f',
      tostring(libPath),
      torqueMapRKin
    )
  )
  if not invertSanityLogged then
    invertSanityLogged = true
    local boost = 0.0
    local t0a = tonumber(lib.drivetrain_forward_torque(444.0, 0.0, boost, 18.0))
    local t0b = tonumber(lib.drivetrain_forward_torque(671.0, 0.0, boost, 68.0))
    local ua = tonumber(lib.drivetrain_inverse_throttle(444.0, boost, 18.0, -300.0))
    local ub = tonumber(lib.drivetrain_inverse_throttle(671.0, boost, 68.0, -300.0))
    log(
      'I',
      logTag,
      string.format(
        'rwd_struct invert sanity T(444,0,18)=%.1f u(-300)=%.3f  T(671,0,68)=%.1f u(-300)=%.3f',
        t0a or 0, ua or 0, t0b or 0, ub or 0
      )
    )
  end
  return true
end

local function hasTorqueMap()
  return torqueMapLib ~= nil
end

local function wheelGain()
  return torqueMapLib and torqueMapRKin or 1
end

-- ---------------------------------------------------------------------------
-- Gains / state
-- ---------------------------------------------------------------------------

local planConfig = {frame_id=1, epoch_id=1, max_source_age_s=1.0}
local wirePlan, lastPlanPacket, lastPlanSequence, lastPlanSource

-- Autonomous physical-command history; new plans must not reset this anchor.
local slewAngle, slewTorque, slewTime
local autonomousRateLimits
local function resetAutonomousSlew()
  slewAngle, slewTorque, slewTime = nil,nil,nil
end

local common = nil
local gtStateController = nil

local gains = {
  pi_enable = 1.0,
  kp = 0.0,
  ki = 0.0,
  i_min = -100.0,
  i_max = 100.0,
  t_min = -500.0,  -- clamp on speed-loop piTerm [N·m], not total tCmd
  t_max = 500.0,
  u_smooth = 0.0,
  du_max = 1.0,
  steer_pi_enable = 1.0,
  steer_kp = 0.0,
  steer_ki = 0.0,
  steer_i_min = -1.0,
  steer_i_max = 1.0,
  command_timeout = 0.5,
}

local controlAuthority = CONTROL_AUTHORITY.full
local cmdMode = CMD_MODE.none

local EXECUTION_FIELDS = {'plan_steer_rad','plan_torque','plan_steer_gap','plan_torque_gap',
  'nominal_total_steer','nominal_total_torque','filter_delta_steer','filter_delta_torque',
  'brake_requested','brake_deadband_suppressed','brake_speed_enabled','brake_speed_suppressed','brake_target','brake_rear_torque','brake_torque_cap','brake_limited',
  'brake_drive_excess','effective_kp','slew_steer_rad','slew_torque','slew_dt','slew_steer_limited','slew_torque_limited'}

local ctrlState = {
  integral_omega = 0,
  u_prev = 0,
  u_ff_smooth = 0,
  integral_steer = 0,
  last_omega_des = nil,
  last_pi_term = 0,
  last_t_cmd = 0,
  last_plant_brake = 0,
  last_torque_min = 0,
  last_torque_max = 0,
}

local setpoints = {
  torque_des = nil,
  omega_des = nil,
  steer_des_rad = nil,
  steer_input_direct = nil,
  brake_des = nil,
  throttle_override = nil,
}

local trajectory = {
  x = {},
  y = {},
  wr = {},
  steer = {},
  torque = {},
  targetCount = 0,
}

local updateAccum = 0
local nowSim = 0
local lastCommandTime = 0
local active = false

local VERBOSE_PERIOD = 5.0
local verboseLastSim = 0
local verboseEnabled = false

local function llcLog(fmt, ...)
  if not verboseEnabled then return end
  log('I', logTag, string.format(fmt, ...))
end

local watch = {
  active = nil,
  cmdMode = nil,
  setpointSig = nil,
}

local SETPOINT_LABELS = {
  'torque_des', 'omega_des', 'steer_des_rad', 'steer_input_direct',
  'brake_des', 'throttle_override',
}
local SETPOINT_NAMES = {
  torque_des = 'torque',
  omega_des = 'wheel_speed',
  steer_des_rad = 'steering',
  steer_input_direct = 'steering_input',
  brake_des = 'brake',
  throttle_override = 'throttle',
}

local function scalarSetpointSignature()
  local keys = {}
  for i = 1, #SETPOINT_LABELS do
    local field = SETPOINT_LABELS[i]
    if setpoints[field] ~= nil then
      keys[#keys + 1] = SETPOINT_NAMES[field]
    end
  end
  return #keys > 0 and table.concat(keys, '+') or '—'
end

-- ---------------------------------------------------------------------------
-- Math helpers
-- ---------------------------------------------------------------------------

local function clip(x, lo, hi)
  return max(lo, min(hi, x))
end

local function scaledTorqueGains(dt)
  ctrlState.effective_kp = gains.kp
  local scale = dt / REF_DT
  return gains.kp * scale, gains.ki * scale
end

local function scaledSteerGains(dt)
  local scale = dt / REF_DT
  return gains.steer_kp * scale, gains.steer_ki * scale
end

local function readPlant(cs)
  return {
    engine_speed_rads = cs.we,
    boost_pressure = cs.pb,
    rear_wheelspeed_ms = 0.5 * (cs.w_rl + cs.w_rr),
    rear_wheel_torque_est = cs.rear_wheel_torque_est,
    forward_speed_m_s = cs.vx,
    throttle = cs.throttle,
    brake = cs.brake or 0,
    front_roadwheel_rad = 0.5 * (cs.delta_l + cs.delta_r),
    pos_x = cs.x,
    pos_y = cs.y,
    torque_min = cs.torque_min or 0,
    torque_max = cs.torque_max or 0,
  }
end

-- ---------------------------------------------------------------------------
-- Torque / throttle control law
-- ---------------------------------------------------------------------------

local function resetSpeedLoopState()
  ctrlState.integral_omega = 0
  ctrlState.u_ff_smooth = 0
end

local function syncSpeedLoopStateForSetpoints()
  -- Reset PI state when wheel-speed target appears/disappears or changes value.
  local omegaDes = setpoints.omega_des
  if ctrlState.last_omega_des ~= omegaDes then
    resetSpeedLoopState()
    ctrlState.last_omega_des = omegaDes
  end
end

-- Occupancy: rwd_struct bisect invert at live (we, boost, ww). Rails T(0)=g, T(1)=g+ΣF.
local MAP_WHEEL_SPEED_MIN_MS = 1.0
-- Below this rear speed, i_min/i_max and t_min/t_max are not applied (launch).
local PI_UNBOUND_BELOW_MS = 2.0

local function occupancyWheelRads(rearWheelspeedMs)
  local r = torqueMapRKin
  local v = rearWheelspeedMs
  if v < MAP_WHEEL_SPEED_MIN_MS then v = MAP_WHEEL_SPEED_MIN_MS end
  return v / r
end

local function inverseOccupancyRails(plant, torqueCmd)
  local tMin = plant.torque_min or 0
  local tMax = plant.torque_max or 0
  ctrlState.last_torque_min = tMin
  ctrlState.last_torque_max = tMax
  local ww = occupancyWheelRads(plant.rear_wheelspeed_ms)
  local u = tonumber(torqueMapLib.drivetrain_inverse_throttle(
    plant.engine_speed_rads, plant.boost_pressure or 0, ww, torqueCmd
  ))
  return clip(u or 0, 0, 1)
end

-- Preserve brake; a tiny positive pedal does not switch all propulsion off.
local function brakeTorqueCap(plant, sp)
  ctrlState.brake_target,ctrlState.brake_rear_torque=sp.brake_des or 0,0
  ctrlState.brake_torque_cap,ctrlState.brake_limited,ctrlState.brake_drive_excess=0,0,0
  if not brakeCoordination or sp.manual_steering~=nil then return nil end
  assert(hasTorqueMap(), 'Brake coordination requires occupancy_rail torque map')
  local b=clip(sp.brake_des or 0,0,1)
  local lever=tonumber(torqueMapLib.drivetrain_r)
  local coefficient=tonumber(torqueMapLib.drivetrain_brake_coeff_rear)
  assert(lever>0 and coefficient>0, 'Invalid exported rear brake calibration')
  ctrlState.brake_rear_torque=-b*lever*coefficient
  if b==0 then return nil end
  local cap=brakeCoordination.overlap_torque_nm/b
  ctrlState.brake_torque_cap=cap
  return cap
end

-- Enforce the physical cap after throttle conditioning, using the same map.
-- Coasting torque may exceed the cap: emit zero throttle and report the residual.
local function capBrakeThrottle(plant, u, cap, b)
  if not cap then return u end
  local limit=inverseOccupancyRails(plant,cap)
  local limited=min(u,limit)
  if limited<u then ctrlState.brake_limited=1 end
  local torque=tonumber(torqueMapLib.drivetrain_forward_torque(plant.engine_speed_rads,
    limited,plant.boost_pressure or 0,occupancyWheelRads(plant.rear_wheelspeed_ms)))
  ctrlState.brake_drive_excess=max(0,b*torque-brakeCoordination.overlap_torque_nm)
  return limited
end

-- Shape total physical targets after PI, before the safety filter. Measurements
-- seed engagement only; subsequent bounds follow the final issued command.
local function autonomousSlew(plant, sp, angle, torque, fallbackDt)
  if sp.manual_steering ~= nil or sp.steer_input_direct ~= nil
    or controlAuthority ~= CONTROL_AUTHORITY.full or angle == nil then
    resetAutonomousSlew()
    return angle,torque
  end
  local steeringRate,torqueRate
  if autonomousRateLimits then
    steeringRate,torqueRate=autonomousRateLimits.steering_rate_max,autonomousRateLimits.torque_rate_max
  elseif common.rollFilter then
    steeringRate,torqueRate=common.rollFilter.getRateLimits()
  else
    resetAutonomousSlew()
    return angle,torque
  end
  local dt=slewTime and nowSim-slewTime or fallbackDt
  assert(dt>0 and dt<huge, 'Invalid autonomous slew interval')
  local anchorAngle=slewAngle or plant.front_roadwheel_rad
  local anchorTorque=slewTorque or plant.rear_wheel_torque_est
  assert(type(anchorAngle)=='number' and anchorAngle==anchorAngle and abs(anchorAngle)<huge,
    'Autonomous slew requires measured steering on engagement')
  local limitedAngle=clip(angle,anchorAngle-steeringRate*dt,anchorAngle+steeringRate*dt)
  local limitedTorque=torque
  if torqueRate then
    assert(type(anchorTorque)=='number' and anchorTorque==anchorTorque and abs(anchorTorque)<huge,
      'Autonomous slew requires estimated torque on engagement')
    limitedTorque=clip(torque,anchorTorque-torqueRate*dt,anchorTorque+torqueRate*dt)
  end
  ctrlState.slew_steer_rad,ctrlState.slew_torque,ctrlState.slew_dt=limitedAngle,limitedTorque,dt
  ctrlState.slew_steer_limited=limitedAngle~=angle and 1 or 0
  ctrlState.slew_torque_limited=limitedTorque~=torque and 1 or 0
  return limitedAngle,limitedTorque
end

-- Downstream safety phase, shared by manual and autonomous physical targets.
-- The filter borrows context for this call; it must not retain or modify it.
local filterContext = {}
local function filterPhysicalTargets(sp, nominalSteerInput, filteredSteerInput, tCmd, dt, brakeCap)
  local decision
  if common.rollFilter then
    local supported = steeringMap ~= nil
      and controlAuthority == CONTROL_AUTHORITY.full
      and nominalSteerInput ~= nil and sp.steer_input_direct == nil
      and (sp.manual_parkingbrake == nil or sp.manual_parkingbrake == 0)
      and (common.controlStateOut.pbrake or 0) == 0
    if supported then
      local nominalAngle = steeringMap.angle(filteredSteerInput)
      filterContext.source = sp.manual_steering ~= nil and 'manual' or 'external'
      filterContext.brake = clip(sp.brake_des or 0, 0, 1)
      filterContext.torque_command_max = brakeCap
      filterContext.steering_command_min_rad = steeringMap.minimum
      filterContext.steering_command_max_rad = steeringMap.maximum
      local nextAngle, nextTorque, selected = common.rollFilter.step(common.controlStateOut,
        nominalAngle, tCmd, dt, nowSim, sp.steering_rate_des, sp.torque_rate_des,
        filterContext)
      assert(type(selected) == 'table', 'Roll filter must return its decision table')
      decision = selected
      local converted = type(nextAngle)=='number' and nextAngle==nextAngle and abs(nextAngle)<huge and steeringMap.input(nextAngle)
      local representable = type(converted)=='number' and converted==converted
        and nextAngle>=steeringMap.minimum and nextAngle<=steeringMap.maximum and type(nextTorque)=='number'
        and nextTorque==nextTorque and abs(nextTorque)<huge
      if sp.manual_steering ~= nil and not representable then
        -- Reject the complete correction pair before either axis is emitted.
        -- A late steering clamp would change the pair tested by the joint row.
        common.rollFilter.unavailable('actuator_command_unrepresentable')
        decision={steering_override=false,torque_override=false,
          reason='actuator_command_unrepresentable'}
      else
        filteredSteerInput = converted
        tCmd = nextTorque
      end
    else
      common.rollFilter.unavailable()
    end
  end

  return filteredSteerInput, tCmd, decision
end

-- Compose PI -> autonomous slew -> brake overlap -> safety -> inverse map.
local function stepTorqueLoop(plant, sp, dt, nominalSteerInput)
  local brakeCap=brakeTorqueCap(plant,sp)
  if sp.torque_des == nil and sp.omega_des == nil and sp.throttle_override == nil then
    return nil
  end
  if sp.throttle_override ~= nil then
    local u = capBrakeThrottle(plant,clip(sp.throttle_override, 0, 1),brakeCap,sp.brake_des or 0)
    ctrlState.u_prev = u
    return u
  end

  -- Torque / wheel-speed loops need the drivetrain inverse map.
  if not hasTorqueMap() then
    return nil
  end

  local torqueDes = sp.torque_des or 0
  local omegaDes = sp.omega_des
  local omega = plant.rear_wheelspeed_ms

  local integralBefore = ctrlState.integral_omega
  local integral = integralBefore
  local piTerm = 0
  local e = 0

  if omegaDes ~= nil and gains.pi_enable ~= 0 then
    local launchUnbound = omega < PI_UNBOUND_BELOW_MS
    e = omegaDes - omega
    integral = integral + e * dt
    if not launchUnbound then
      integral = clip(integral, gains.i_min, gains.i_max)
    end
    local kpEff, kiEff = scaledTorqueGains(dt)
    local piTermRaw = gains.pi_enable * (kpEff * e + kiEff * integral)
    if launchUnbound then
      piTerm = piTermRaw
    else
      piTerm = clip(piTermRaw, gains.t_min, gains.t_max)
      -- Anti-windup when piTerm saturates (back off integral for this step).
      if piTerm ~= piTermRaw then
        if (piTerm >= gains.t_max and e > 0) or (piTerm <= gains.t_min and e < 0) then
          integral = integral - e * dt
        end
      end
    end
    ctrlState.integral_omega = integral
  else
    ctrlState.integral_omega = 0
  end

  -- Physical torque composition: this includes PI, not only trajectory feedforward.
  -- The optional roll-filter hook below acts on this total before final conversion.
  -- See FEEDBACK_CONTRACT.md for command timing, units and failure behavior.
  local tCmd = torqueDes + piTerm
  ctrlState.nominal_total_torque = tCmd

  -- Final physical-command hook: steering PI is already composed, torque PI is
  -- included above. The module sees normalized measured state, not path coordinates.
  -- It returns an angle target and ABSOLUTE torque; neither gets integrated twice.
  local nominalAngle=nominalSteerInput and steeringMap.angle(nominalSteerInput)
  local limitedAngle,limitedTorque=autonomousSlew(plant,sp,nominalAngle,tCmd,dt)
  -- Do not accumulate speed-loop error while it pushes into the slew bound.
  local slewBlockedIntegral=(tCmd-limitedTorque)*e>0
  if slewBlockedIntegral then ctrlState.integral_omega=integralBefore end
  tCmd=limitedTorque
  if brakeCap and tCmd>brakeCap then
    tCmd=brakeCap
    ctrlState.brake_limited=1
    if e>0 then ctrlState.integral_omega=integralBefore end
  end
  local filteredSteerInput=limitedAngle and steeringMap.input(limitedAngle)
  local decision
  filteredSteerInput, tCmd, decision = filterPhysicalTargets(
    sp, nominalSteerInput, filteredSteerInput, tCmd, dt, brakeCap)

  if brakeCap then
    assert(tCmd<=brakeCap+1e-6, 'Roll filter exceeded brake torque cap')
  end
  if ctrlState.slew_dt then
    -- Commit the filter's final physical pair, never the discarded raw target.
    slewAngle,slewTorque,slewTime=steeringMap.angle(filteredSteerInput),tCmd,nowSim
  end

  local uFfRaw
  if sp.manual_throttle ~= nil and tCmd == torqueDes then
    -- Exact driver passthrough in shadow/unavailable/no-torque-intervention cases.
    -- Avoid a lossy forward/inverse round trip and duplicate input smoothing.
    uFfRaw = sp.manual_throttle
  else
    uFfRaw = inverseOccupancyRails(plant, tCmd)
  end

  -- Existing actuator-side conditioning changes the inverse-map throttle request.
  -- This retained smoothing/slew stage means the filtered torque target need not
  -- equal instantaneous executed torque; it is part of actuator response.
  if sp.manual_throttle ~= nil then
    -- Stock input processing already conditioned the pedal. A corrected physical
    -- torque goes through the same inverse map, without a second throttle filter.
    ctrlState.u_ff_smooth, ctrlState.u_prev = uFfRaw, uFfRaw
    ctrlState.last_pi_term, ctrlState.last_t_cmd = 0, tCmd
    return uFfRaw, filteredSteerInput, decision
  end
  local alpha = clip(gains.u_smooth, 0, 0.99)
  local uFf
  if alpha > 0 then
    uFf = alpha * ctrlState.u_ff_smooth + (1 - alpha) * uFfRaw
  else
    uFf = uFfRaw
  end

  local duMax = gains.du_max * (dt / REF_DT)
  local du = uFf - ctrlState.u_prev
  du = clip(du, -duMax, duMax)
  local uCmd = clip(ctrlState.u_prev + du, 0, 1)

  local conditioned=uCmd
  uCmd=capBrakeThrottle(plant,uCmd,brakeCap,sp.brake_des or 0)
  if uCmd<conditioned then
    uFf=min(uFf,uCmd)
    if e>0 then ctrlState.integral_omega=integralBefore; slewBlockedIntegral=true end
  end

  -- Throttle saturation anti-windup (cannot push harder into the plant).
  if omegaDes ~= nil and gains.pi_enable ~= 0 then
    if (uCmd >= 1 and e > 0) or (uCmd <= 0 and e < 0) then
      local integralUw = slewBlockedIntegral and ctrlState.integral_omega or ctrlState.integral_omega - e * dt
      if omega < PI_UNBOUND_BELOW_MS then
        ctrlState.integral_omega = integralUw
      else
        ctrlState.integral_omega = clip(integralUw, gains.i_min, gains.i_max)
      end
    end
  end

  ctrlState.u_ff_smooth = uFf
  ctrlState.u_prev = uCmd
  ctrlState.last_pi_term = piTerm
  ctrlState.last_t_cmd = tCmd
  return uCmd, filteredSteerInput, decision
end

-- ---------------------------------------------------------------------------
-- Steering PI on roadwheel angle [rad] → steering_input [-1, 1]
-- ---------------------------------------------------------------------------

local function stepSteeringLoop(plant, sp, dt)
  if sp.steer_des_rad == nil then return nil end

  local e = sp.steer_des_rad - plant.front_roadwheel_rad
  local integral = ctrlState.integral_steer + e * dt
  integral = clip(integral, gains.steer_i_min, gains.steer_i_max)

  local kpEff, kiEff = scaledSteerGains(dt)
  local steerCmdRad = sp.steer_des_rad + gains.steer_pi_enable * (kpEff * e + kiEff * integral)
  local steerInput = steeringMap.input(steerCmdRad)

  ctrlState.integral_steer = integral
  return steerInput
end

-- ---------------------------------------------------------------------------
-- Trajectory projection — closest segment + pick mode (next / lerp / closest)
-- ---------------------------------------------------------------------------

local function findClosestSegment(px, py)
  local N = trajectory.targetCount
  if N < 2 then return 1, 1, 0 end

  local bestI, bestJ, bestT = 1, 2, 0
  local bestD2 = 1e10
  for i = 1, N - 1 do
    local j = i + 1
    local x1, y1 = trajectory.x[i], trajectory.y[i]
    local x2, y2 = trajectory.x[j], trajectory.y[j]
    local vx, vy = x2 - x1, y2 - y1
    local wx, wy = px - x1, py - y1
    local segLen2 = vx * vx + vy * vy
    local t, qx, qy
    if segLen2 <= 1e-12 then
      t = 0
      qx, qy = x1, y1
    else
      t = (wx * vx + wy * vy) / segLen2
      t = clip(t, 0, 1)
      qx = x1 + t * vx
      qy = y1 + t * vy
    end
    local dx, dy = px - qx, py - qy
    local d2 = dx * dx + dy * dy
    if d2 < bestD2 then
      bestD2 = d2
      bestI, bestJ, bestT = i, j, t
    end
  end
  return bestI, bestJ, bestT
end

local function lerp(a, b, t)
  return a + t * (b - a)
end

local function pickAlongTrajectory(arr, i1, i2, t)
  if trajectoryPick == TRAJECTORY_PICK.lerp then
    return lerp(arr[i1], arr[i2], t)
  end
  if trajectoryPick == TRAJECTORY_PICK.closest then
    return arr[i1]
  end
  -- next: segment end waypoint (idx2)
  return arr[i2] or arr[i1]
end

local function interpolateSeries(grid, arr, query)
  local n = #grid
  if query <= grid[1] then return arr[1], query < grid[1] end
  if query >= grid[n] then return arr[n], query > grid[n] end
  local lo, hi = 1, n
  while hi - lo > 1 do
    local mid = floor((lo + hi) / 2)
    if grid[mid] <= query then lo = mid else hi = mid end
  end
  return lerp(arr[lo], arr[hi], (query - grid[lo]) / (grid[hi] - grid[lo])), false
end

local function trajectorySetpoints(plant)
  local i1, i2, f = findClosestSegment(plant.pos_x, plant.pos_y)
  local phaseTime = trajectory.t and lerp(trajectory.t[i1], trajectory.t[i2], f) or -1
  local phaseDistance = lerp(trajectory.distance[i1], trajectory.distance[i2], f)
  selection = {
    llc_trajectory_id = trajectorySequence, llc_segment = i1, llc_fraction = f,
    llc_phase_time = phaseTime, llc_phase_distance = phaseDistance,
    llc_plan_age = trajectory.t and (nowSim - trajectory.t[1]) or -1,
    llc_active = 1,
    llc_preview_kind = trajectoryPick == TRAJECTORY_PICK.preview
      and (trajectoryPreview.kind == 'time_s' and 1 or 2) or 0,
  }
  local function target(arr, channel)
    if trajectoryPick ~= TRAJECTORY_PICK.preview then
      selection['llc_query_' .. channel] = -1
      selection['llc_clamp_' .. channel] = 0
      return pickAlongTrajectory(arr, i1, i2, f)
    end
    local grid = trajectoryPreview.kind == 'time_s' and trajectory.t or trajectory.distance
    local phase = trajectoryPreview.kind == 'time_s' and phaseTime or phaseDistance
    local query = phase + trajectoryPreview[channel]
    local value, clamped = interpolateSeries(grid, arr, query)
    selection['llc_query_' .. channel] = query
    selection['llc_clamp_' .. channel] = clamped and 1 or 0
    return value
  end
  local sp = {
    omega_des = target(trajectory.wr, 'wheel_speed'),
    torque_des = target(trajectory.torque, 'torque'),
    steer_des_rad = target(trajectory.steer, 'steering'),
    brake_des = trajectory.brake and target(trajectory.brake, 'torque') or setpoints.brake_des,
    throttle_override = setpoints.throttle_override,
  }
  sp.plan_steer_rad,sp.plan_torque = sp.steer_des_rad,sp.torque_des
  if lastResolvedSequence ~= trajectorySequence then
    trajectory.steerBeforeReplace = lastResolvedSteer or sp.steer_des_rad
    trajectory.steerAfterReplace = sp.steer_des_rad
  end
  selection.llc_steer_before_replace = trajectory.steerBeforeReplace
  selection.llc_steer_after_replace = trajectory.steerAfterReplace
  lastResolvedSteer, lastResolvedSequence = sp.steer_des_rad, trajectorySequence
  return sp
end

local function scalarSetpoints()
  return {
    torque_des = setpoints.torque_des,
    omega_des = setpoints.omega_des,
    steer_des_rad = setpoints.steer_des_rad,
    steering_rate_des = setpoints.steering_rate_des,
    torque_rate_des = setpoints.torque_rate_des,
    steer_input_direct = setpoints.steer_input_direct,
    brake_des = setpoints.brake_des,
    throttle_override = setpoints.throttle_override,
  }
end

-- ---------------------------------------------------------------------------
-- Actuation + authority gating
-- ---------------------------------------------------------------------------

local function applyActuation(plant, sp, dt)
  ctrlState.nominal_total_torque,ctrlState.nominal_total_steer=nil,nil
  ctrlState.effective_kp=0
  ctrlState.slew_steer_rad,ctrlState.slew_torque,ctrlState.slew_dt=nil,nil,nil
  ctrlState.slew_steer_limited,ctrlState.slew_torque_limited=0,0
  local manual = sp.manual_steering ~= nil and manualInput ~= nil
  ctrlState.brake_requested=sp.brake_des or 0
  ctrlState.brake_deadband_suppressed=0
  if not manual and brakeCoordination and brakeCoordination.pedal_deadband
    and (sp.brake_des or 0)<=brakeCoordination.pedal_deadband then
    ctrlState.brake_deadband_suppressed=(sp.brake_des or 0)>0 and 1 or 0
    sp.brake_des=0
  end
  ctrlState.brake_speed_suppressed=0
  if not manual and brakeCoordination and brakeCoordination.speed_disable_m_s then
    local speed=plant.forward_speed_m_s
    assert(type(speed)=='number' and speed==speed and abs(speed)<huge, 'Brake speed guard requires signed forward speed')
    if speed<=brakeCoordination.speed_disable_m_s then ctrlState.brake_speed_enabled=0
    elseif speed>=brakeCoordination.speed_enable_m_s then ctrlState.brake_speed_enabled=1 end
    if ctrlState.brake_speed_enabled~=1 then
      ctrlState.brake_speed_suppressed=(sp.brake_des or 0)>0 and 1 or 0
      sp.brake_des=0
    end
  elseif manual then ctrlState.brake_speed_enabled=0 end
  local decision
  local allowThrottle = controlAuthority == CONTROL_AUTHORITY.full
      or controlAuthority == CONTROL_AUTHORITY.torque_only
  local allowSteer = controlAuthority == CONTROL_AUTHORITY.full
      or controlAuthority == CONTROL_AUTHORITY.steering_only
  local allowBrake = controlAuthority == CONTROL_AUTHORITY.full

  if sp.throttle_override ~= nil or not allowThrottle or not hasTorqueMap()
    or (sp.torque_des == nil and sp.omega_des == nil) then resetAutonomousSlew() end
  local steer
  local steerIntegralBefore=ctrlState.integral_steer
  if allowSteer then
    if sp.manual_steering ~= nil then
      -- Already processed by BeamNG: do not add steering PI to human intent.
      steer = sp.manual_steering
    elseif sp.steer_input_direct ~= nil then
      steer = clip(sp.steer_input_direct, -1, 1)
    else
      steer = stepSteeringLoop(plant, sp, dt)
    end
  end

  if common.rollFilter and (sp.throttle_override ~= nil or not allowThrottle or not hasTorqueMap()
    or (sp.torque_des == nil and sp.omega_des == nil)) then
    common.rollFilter.unavailable()
  end
  ctrlState.nominal_total_steer = steer and steeringMap.angle(steer) or sp.steer_des_rad
  if allowThrottle then
    local throttle, filteredSteer, selected = stepTorqueLoop(plant, sp, dt, steer)
    decision = selected
    if filteredSteer ~= nil then steer = filteredSteer end
    if ctrlState.slew_steer_limited==1 and sp.steer_des_rad ~= nil
      and (ctrlState.nominal_total_steer-ctrlState.slew_steer_rad)*(sp.steer_des_rad-plant.front_roadwheel_rad)>0 then
      ctrlState.integral_steer=steerIntegralBefore
    end
    if manual and not (decision and decision.torque_override) then
      manualInput.release('throttle')
    elseif throttle ~= nil then
      emitInput('throttle', throttle)
      electrics.values.throttle = throttle
      electrics.values.throttle_input = throttle
    end
  end
  if manual and (not allowThrottle) then manualInput.release('throttle') end
  if manual and (not allowSteer or not (decision and decision.steering_override)) then
    manualInput.release('steering')
    ctrlState.last_steer_cmd = steeringMap.angle(sp.manual_steering)
  elseif allowSteer and steer ~= nil then
    ctrlState.last_steer_cmd = steeringMap.angle(steer)
    emitInput('steering', steer)
    electrics.values.steering_input = steer
  end

  -- Assert brake every tick while active (0 clears a latched manual brake press).
  local b = 0
  if allowBrake and sp.brake_des ~= nil then
    b = clip(sp.brake_des, 0, 1)
  end
  if manual then
    manualInput.release('brake')
  else
    emitInput('brake', b)
    electrics.values.brake = b
    electrics.values.brake_input = b
  end

  if (ctrlState.last_plant_brake or 0) > 0.05 and b <= 0 then
    resetSpeedLoopState()
  end
  ctrlState.last_plant_brake = plant.brake
end

-- ---------------------------------------------------------------------------
-- Split control-step API (for controller_manager + optional feedback plugins)
-- ---------------------------------------------------------------------------

-- dt: elapsed physics seconds; c: manager-owned shared state.
-- Returns one due control step, or nil when inactive/not due/waiting for state.
function M.prepareControlStep(dt, c)
  common = c or common
  if not common or not common.isRunning then return nil end

  updateAccum = updateAccum + dt
  if updateAccum < common.controllerRate then return nil end
  updateAccum = updateAccum - common.controllerRate

  nowSim = common.getSimTime()

  if restoreTorqueMapAfterReset then
    -- GTState.reset reinitializes its sensor and clears the map attachment. All
    -- controller reset callbacks have finished by this tick, independent of their
    -- order. Reattach once, then wait one cadence for fresh estimates/live rails.
    gtStateController.setTorqueMapLib(torqueMapLib)
    restoreTorqueMapAfterReset = false
    return nil
  end

  if active and wirePlan and nowSim-wirePlan.source > planConfig.max_source_age_s then
    M.clearCommand()
    log('E',logTag,'Execution plan source expired; external control released')
  end
  if active and (nowSim - lastCommandTime) > gains.command_timeout then
    M.clearCommand()
    log('I', logTag, string.format('Command timeout — manual control (t=%.3f)', nowSim))
  end

  -- Fresh external input wins; on timeout use the independently preserved human
  -- request. Vehicles without a roll filter retain their existing inactive path.
  local previousSource = inputSource
  local manual = not active and manualInput ~= nil
  local manualOff = manual and (not common.rollFilter or not common.rollFilter.isEnabled())
  inputSource = (common.isBypassed or manualOff) and 0 or (active and 2 or (manual and 1 or 0))
  if gtStateController and gtStateController.setCustomField then
    gtStateController.setCustomField('llc_active', inputSource ~= 0 and 1 or 0)
    gtStateController.setCustomField('llc_input_source', inputSource)
  end
  if inputSource ~= 2 then resetAutonomousSlew() end
  if inputSource == 0 then
    if manualInput then manualInput.release() end
    if common.rollFilter then common.rollFilter.idle() end
    return nil
  end

  local cs = common.controlStateOut
  if not cs or cs.t < 0 then
    if manual then
      manualInput.release()
      if common.rollFilter then common.rollFilter.idle() end
    end
    return nil
  end
  if manual and previousSource ~= 1 then
    manualInput.beginManual(steeringMap.input(.5 * (cs.delta_l + cs.delta_r)),
      cs.throttle or 0)
  end

  local doVerbose = verboseEnabled and (nowSim - verboseLastSim) >= VERBOSE_PERIOD
  return {
    plant = readPlant(cs),
    manual = manual,
    dt_control = common.controllerRate,
    doVerbose = doVerbose,
    tLoop0 = doVerbose and os.clock() or nil,
    nowSim = nowSim,
  }
end

-- Return physical setpoints for one step, or false when plan coverage is exhausted.
-- plant is this step's measured snapshot; dt is the control interval [s].
function M.resolveSetpoints(plant, dt)
  if inputSource == 1 then
    -- Passthrough executes stock processed inputs. Evaluate that SAME pedal,
    -- even if reference-ramp settings are present in the capture helper.
    -- Such a software ramp cannot constrain a mechanically connected driver.
    local request = manualInput.getDriverTargets()
    local throttle = request.throttle
    local torque = tonumber(torqueMapLib.drivetrain_forward_torque(plant.engine_speed_rads,
      throttle, plant.boost_pressure or 0, occupancyWheelRads(plant.rear_wheelspeed_ms)))
    assert(torque and torque == torque and abs(torque) < huge, 'Invalid manual torque-map result')
    return {torque_des=torque, steer_des_rad=steeringMap.angle(request.steering),
      brake_des=request.brake, manual_throttle=throttle, manual_steering=request.steering,
      manual_parkingbrake=request.parkingbrake}
  end
  if cmdMode == CMD_MODE.trajectory then
    local sp=trajectorySetpoints(plant)
    if sp==false then return false end
    return sp
  end
  return scalarSetpoints()
end

-- Keep human input separate from autonomous output, with or without a filter.
function M.enableManualInput(settings)
  if common.rollFilter then
    assert(hasTorqueMap(), 'Manual roll filtering requires a calibrated torque map')
    assert(controlAuthority == CONTROL_AUTHORITY.full, 'Manual roll filtering requires full authority')
  end
  assert(not manualInput, 'Manual bridge is already enabled')
  manualInput = require('vehicle.controller.xlab.manual_input')
  manualInput.init(settings, steeringMap)
end

function M.releaseInput()
  if manualInput then manualInput.release() end
end

-- Asymmetric measured limits; both return values are physical roadwheel rad.
function M.getSteeringCapability()
  assert(steeringMap, 'Measured steering calibration is not initialized')
  return steeringMap.minimum, steeringMap.maximum
end

-- Release external authority without resetting measurement/observer history.
function M.clearCommand()
  if manualInput then manualInput.release() end
  ctrlState.brake_speed_enabled=0
  resetAutonomousSlew()
  wirePlan=nil
  active = false
  cmdMode = CMD_MODE.none
  resetSpeedLoopState()
  ctrlState.integral_steer = 0
  ctrlState.last_omega_des = nil
  watch.active = false
  watch.cmdMode = CMD_MODE.none
  watch.setpointSig = nil
end

-- Borrowed command view, refreshed on access. Feedback consumes it synchronously;
-- neither retain the wrapper nor modify the nested command arrays/selection.
local rawTargets = {scalar={}, trajectory={}}
function M.getRawTargets()
  rawTargets.mode, rawTargets.selection = cmdMode, selection
  local scalar = rawTargets.scalar
  scalar.torque_des, scalar.omega_des = setpoints.torque_des, setpoints.omega_des
  scalar.steer_des_rad, scalar.steer_input_direct = setpoints.steer_des_rad, setpoints.steer_input_direct
  scalar.steering_rate_des, scalar.torque_rate_des = setpoints.steering_rate_des, setpoints.torque_rate_des
  scalar.brake_des, scalar.throttle_override = setpoints.brake_des, setpoints.throttle_override
  local path = rawTargets.trajectory
  path.x, path.y, path.z, path.t = trajectory.x, trajectory.y, trajectory.z, trajectory.t
  path.wr, path.steer, path.torque = trajectory.wr, trajectory.steer, trajectory.torque
  path.brake, path.distance, path.targetCount = trajectory.brake, trajectory.distance, trajectory.targetCount
  return rawTargets
end

-- Execute once: PI, autonomous slew, brake cap, optional filter, actuator conversion.
-- Updates existing GT diagnostics; step carries optional verbose timing metadata.

function M.applySetpoints(plant, sp, dt, step)
  -- Feedback corrects actuator targets before the common PI/filter path.
  applyActuation(plant, sp, dt)
  ctrlState.plan_steer_rad,ctrlState.plan_torque = sp.plan_steer_rad,sp.plan_torque
  ctrlState.plan_steer_gap = sp.plan_steer_rad and ctrlState.last_steer_cmd-sp.plan_steer_rad or 0
  ctrlState.plan_torque_gap = sp.plan_torque and ctrlState.last_t_cmd-sp.plan_torque or 0
  ctrlState.filter_delta_steer = ctrlState.last_steer_cmd and ctrlState.nominal_total_steer
    and ctrlState.last_steer_cmd-(ctrlState.slew_steer_rad or ctrlState.nominal_total_steer) or 0
  ctrlState.filter_delta_torque = ctrlState.last_t_cmd- (ctrlState.slew_torque or ctrlState.nominal_total_torque or ctrlState.last_t_cmd)

  if gtStateController and gtStateController.setCustomField then
    local tCmd = ctrlState.last_t_cmd
    if sp.torque_des == nil and sp.omega_des == nil then
      tCmd = 0
    end
    gtStateController.setCustomField('target_torque', tCmd)
    gtStateController.setCustomField('target_wr', sp.omega_des or 0)
    gtStateController.setCustomField('target_steer', ctrlState.last_steer_cmd or sp.steer_des_rad or 0)
    gtStateController.setCustomField('target_pi_torque', ctrlState.last_pi_term or 0)
    for _,name in ipairs(EXECUTION_FIELDS) do gtStateController.setCustomField('llc_'..name,ctrlState[name] or 0) end
    if cmdMode == CMD_MODE.trajectory then
      for _, name in ipairs(SELECTION_FIELDS) do
        gtStateController.setCustomField(name, selection[name] or 0)
      end
    end
  end

  if step and step.doVerbose and step.tLoop0 then
    local loopMs = (os.clock() - step.tLoop0) * 1000.0
    local wg = wheelGain()
    local pm = common.performanceMetrics
    local latMs = (pm.avgLatency or 0) * 1000
    log(
      'I',
      logTag,
      string.format(
        'SimTime=%.3f mode=%s wr_t=%.2f wr=%.2f tor_des=%.2f tCmd=%.2f piT=%.2f Tmin=%.2f Tmax=%.2f steer_t=%.3f delta=%.3f thr=%.3f brk=%.3f mapc=%.4f lat=%.1fms loop=%.3fms',
        step.nowSim or nowSim,
        cmdMode,
        sp.omega_des or 0,
        plant.rear_wheelspeed_ms,
        (sp.torque_des or 0),
        ctrlState.last_t_cmd,
        ctrlState.last_pi_term,
        ctrlState.last_torque_min or 0,
        ctrlState.last_torque_max or 0,
        sp.steer_des_rad or 0,
        plant.front_roadwheel_rad,
        ctrlState.u_prev,
        plant.brake,
        wg,
        latMs,
        loopMs
      )
    )
    verboseLastSim = step.nowSim or nowSim
  end
end

function M.finishControlStep(_c, _step)
end

-- ---------------------------------------------------------------------------
-- Command / tune
-- ---------------------------------------------------------------------------

local function isTrajectoryPayload(data)
  local torqueArr = data.desired_torque or data.torque
  return type(data.x) == 'table' and type(data.y) == 'table'
      and type(data.wr) == 'table' and type(data.steer) == 'table'
      and type(torqueArr) == 'table'
end

local function finiteNumber(v)
  return type(v) == 'number' and v == v and abs(v) < huge
end

local function loadTrajectory(data)
  local n = #data.x
  if data.execution_mode~=nil or data.steer_rate~=nil or data.torque_rate~=nil or data.brake_rate~=nil then
    log('E',logTag,'Trajectory execution accepts actuator targets only');return false
  end
  if n < 1 then return false end
  if #data.y ~= n or #data.wr ~= n or #data.steer ~= n then
    log('E', logTag, 'Trajectory array length mismatch')
    return false
  end
  local torqueArr = data.desired_torque or data.torque
  if type(torqueArr) ~= 'table' or #torqueArr ~= n then
    log('E', logTag, 'Trajectory torque array required and must match path length')
    return false
  end
  if data.t ~= nil and (type(data.t) ~= 'table' or #data.t ~= n) then
    log('E', logTag, 'Trajectory t length mismatch')
    return false
  end
  if data.z ~= nil and (type(data.z) ~= 'table' or #data.z ~= n) then
    log('E', logTag, 'Trajectory z length mismatch')
    return false
  end
  if trajectoryPick == TRAJECTORY_PICK.preview then
    if trajectoryPreview.kind == 'time_s' and data.t == nil then
      log('E', logTag, 'Time preview requires trajectory t')
      return false
    end
    if trajectoryPreview.kind == 'distance_m' and data.z == nil then
      log('E', logTag, '3D distance preview requires trajectory z')
      return false
    end
  end
  for _,key in ipairs({'brake'}) do
    local values = data[key]
    if values ~= nil then
      if type(values) ~= 'table' or #values ~= n then return false end
      for i=1,n do if not finiteNumber(values[i]) then return false end end
    end
  end
  local distance = {0}
  for i = 1, n do
    for _, arr in ipairs({data.x, data.y, data.wr, data.steer, torqueArr}) do
      if not finiteNumber(arr[i]) then
        log('E', logTag, 'Trajectory contains non-finite values')
        return false
      end
    end
    if data.z and not finiteNumber(data.z[i]) then return false end
    if data.t and (not finiteNumber(data.t[i]) or (i > 1 and data.t[i] <= data.t[i - 1])) then
      log('E', logTag, 'Trajectory t must be finite and strictly increasing')
      return false
    end
    if i > 1 then
      local dx, dy = data.x[i] - data.x[i - 1], data.y[i] - data.y[i - 1]
      local dz = data.z and (data.z[i] - data.z[i - 1]) or 0
      distance[i] = distance[i - 1] + sqrt(dx * dx + dy * dy + dz * dz)
    end
  end
  trajectorySequence = trajectorySequence + 1
  trajectory.t, trajectory.z, trajectory.distance = data.t, data.z, distance
  trajectory.x = data.x
  trajectory.y = data.y
  trajectory.wr = data.wr
  trajectory.steer = data.steer
  trajectory.torque = torqueArr
  trajectory.brake = data.brake
  trajectory.targetCount = n
  -- Trajectory replaces scalar setpoints; do not inherit throttle_override / brake.
  setpoints.torque_des = nil
  setpoints.omega_des = nil
  setpoints.steer_des_rad = nil
  setpoints.steering_rate_des, setpoints.torque_rate_des = nil,nil
  setpoints.steer_input_direct = nil
  setpoints.brake_des = nil
  setpoints.throttle_override = nil
  -- Keep PI integral and throttle smoothing across ordinary horizon replacement.
  if not active or cmdMode ~= CMD_MODE.trajectory then
    resetSpeedLoopState()
    ctrlState.last_omega_des = nil
  end
  cmdMode = CMD_MODE.trajectory
  return true
end

local function loadScalar(data)
  local hasTarget = (
    data.torque ~= nil or data.wheel_speed ~= nil or data.steering ~= nil
    or data.steering_input ~= nil or data.brake ~= nil or data.throttle ~= nil
  )
  if not hasTarget then return false end
  for _,key in ipairs({'steering_rate','torque_rate','steering','torque','wheel_speed',
    'steering_input','brake','throttle'}) do
    if data[key] ~= nil and not finiteNumber(data[key]) then return false end
  end

  -- Replace semantics: omitted fields are cleared (no stale throttle_override etc.)
  setpoints.torque_des = nil
  setpoints.omega_des = nil
  setpoints.steer_des_rad = nil
  setpoints.steering_rate_des, setpoints.torque_rate_des = nil,nil
  setpoints.steer_input_direct = nil
  setpoints.brake_des = nil
  setpoints.throttle_override = nil

  if data.torque ~= nil then setpoints.torque_des = tonumber(data.torque) end
  if data.wheel_speed ~= nil then setpoints.omega_des = tonumber(data.wheel_speed) end
  if data.steering ~= nil then setpoints.steer_des_rad = tonumber(data.steering) end
  setpoints.steering_rate_des, setpoints.torque_rate_des = data.steering_rate,data.torque_rate
  if data.brake ~= nil then setpoints.brake_des = tonumber(data.brake) end
  if data.throttle ~= nil then setpoints.throttle_override = tonumber(data.throttle) end
  -- Direct BeamNG input — bypasses the measured steering map (calibration only).
  if data.steering_input ~= nil then
    setpoints.steer_input_direct = tonumber(data.steering_input)
    setpoints.steer_des_rad = nil
    setpoints.steering_rate_des, setpoints.torque_rate_des = nil, nil
  end
  syncSpeedLoopStateForSetpoints()
  cmdMode = CMD_MODE.scalar
  return true
end

-- Reception validates first; only onPlanPacket may replace command authority.
function M.decodePlan(bytes, now)
  return ExecutionPlan.decode(bytes, now, planConfig, lastPlanSource, lastPlanSequence, active)
end

function M.onPlanPacket(bytes, feedback)
  if bytes==lastPlanPacket then return true end -- duplicates never renew source time
  local ok,received=pcall(M.decodePlan,bytes,common.getSimTime())
  if not ok then log('E',logTag,'Plan rejected: '..tostring(received));return false end
  if received.gains and not (feedback and feedback.setPlan) then
    log('E',logTag,'Gain-bearing plan requires feedback=tvlqr');return false
  end
  if received.enabled then
    assert(M.onCommand(received.native)~=false,'LLC rejected validated execution plan')
  else M.clearCommand() end
  wirePlan=received.enabled and received or nil
  if feedback and feedback.setPlan then feedback.setPlan(received,M) end
  lastPlanSequence,lastPlanPacket,lastPlanSource=received.sequence,bytes,received.source
  return true
end

function M.onCommand(data)
  if type(data) ~= 'table' then return false end

  local prevMode = cmdMode
  local prevActive = active

  if isTrajectoryPayload(data) then
    if not loadTrajectory(data) then return false end
  elseif not loadScalar(data) then
    return false
  end

  if not prevActive or prevMode ~= cmdMode then
    resetSpeedLoopState()
    resetAutonomousSlew()
  end
  nowSim = common.getSimTime()
  lastCommandTime = nowSim
  active = true

  if not prevActive then
    llcLog('LLC active: mode=%s (t=%.3f)', cmdMode, nowSim)
  elseif prevMode ~= cmdMode then
    if cmdMode == CMD_MODE.trajectory then
      llcLog('cmd mode: %s → trajectory (N=%d, t=%.3f)', prevMode, trajectory.targetCount, nowSim)
    else
      llcLog('cmd mode: %s → scalar (t=%.3f)', prevMode, nowSim)
    end
  end

  if cmdMode == CMD_MODE.scalar then
    local sig = scalarSetpointSignature()
    if watch.setpointSig ~= sig then
      llcLog('scalar setpoints: %s (t=%.3f)', sig, nowSim)
      watch.setpointSig = sig
    end
  else
    watch.setpointSig = nil
  end
  watch.cmdMode = cmdMode
  watch.active = active

  local pm = common.performanceMetrics
  pm.lastCommandTimestamp = nowSim
  pm.commandsReceived = pm.commandsReceived + 1
  return true
end

-- Calibration is cold-path work: validate a candidate, then commit once.
function M.calibrate(params)
  local ok, next = pcall(Calibration.resolve, params, {
    gains = gains, brake = brakeCoordination, rates = autonomousRateLimits,
    steering = steeringMap, plan = planConfig, preview = trajectoryPreview,
    pick = trajectoryPick, authority = controlAuthority, verbose = verboseEnabled,
    torqueMap = torqueMapStem,
  }, {
    active = active, initialized = initialized, hasFilter = common and common.rollFilter ~= nil,
    trajectory = cmdMode == CMD_MODE.trajectory and trajectory or nil,
  })
  if not ok then
    log('E', logTag, 'Calibration rejected: ' .. tostring(next))
    return false
  end
  local previousAuthority = controlAuthority
  gains, brakeCoordination, autonomousRateLimits = next.gains, next.brake, next.rates
  steeringMap, planConfig, trajectoryPreview = next.steering, next.plan, next.preview
  trajectoryPick, controlAuthority = next.pick, next.authority
  verboseEnabled, torqueMapStem = next.verbose, next.torqueMap
  if params.execution_plan ~= nil then
    wirePlan, lastPlanPacket, lastPlanSequence, lastPlanSource = nil, nil, nil, nil
  end
  if previousAuthority ~= controlAuthority then resetSpeedLoopState() end
  log('I', logTag, 'Calibration applied; control_mode=' .. controlAuthority)
  return true
end

function M.onTune(data)
  if type(data) ~= 'table' then return false end
  if data.manual_input ~= nil then
    if not manualInput then
      log('E', logTag, 'manual_input tune requires the input bridge')
      return false
    end
    local ok, reason = pcall(manualInput.validate, data.manual_input)
    if not ok then log('E', logTag, 'Manual tune rejected: ' .. tostring(reason));return false end
  end
  local settings = data.calibration
  if settings == nil then
    -- Manager envelopes may contain only another module's namespace.
    settings = (data.manual_input or data.roll_filter or data.feedback_config) and {} or data
  end
  if not M.calibrate(settings) then return false end
  if data.manual_input ~= nil then manualInput.onTune(data.manual_input) end
  return true
end

-- ---------------------------------------------------------------------------
-- Plugin lifecycle
-- ---------------------------------------------------------------------------

function M.init(c)
  assert(steeringMap, 'Measured steering_map calibration required before LLC initialization')
  common = c

  if not common.gtStateManager or not common.gtStateSensorId then
    log('E', logTag, 'gtState manager or sensor id missing')
    return false
  end

  gtStateController = common.gtStateManager.getGtStateController(common.gtStateSensorId)
  if not gtStateController then
    log('E', logTag, 'gtState controller not found')
    return false
  end

  if gtStateController.registerCustomField then
    gtStateController.registerCustomField('target_torque', 0)
    gtStateController.registerCustomField('target_wr', 0)
    gtStateController.registerCustomField('target_steer', 0)
    gtStateController.registerCustomField('target_pi_torque', 0)
    gtStateController.registerCustomField('llc_input_source', 0)
    for _,name in ipairs(EXECUTION_FIELDS) do gtStateController.registerCustomField('llc_'..name,0) end
    for _, name in ipairs(SELECTION_FIELDS) do
      gtStateController.registerCustomField(name, 0)
    end
  end

  loadTorqueMap(common.torqueMapPath)

  if torqueMapLib and gtStateController.setTorqueMapLib then
    gtStateController.setTorqueMapLib(torqueMapLib)
  elseif not torqueMapLib then
    log('W', logTag, 'No torque map lib; gtState forward torque estimate unavailable')
  end

  local wg = wheelGain()
  common.drivetrainWheelGain = wg
  log('I', logTag, string.format(
    'LLC init torque_map=%s mapc=%.4f authority=%s torque_ctrl=%s verbose=%s',
    tostring(torqueMapStem), wg, controlAuthority, tostring(hasTorqueMap()),
    tostring(verboseEnabled)
  ))
  initialized = true
  return true
end

function M.update(dt, c)
  local step = M.prepareControlStep(dt, c)
  if not step then return end
  local sp = M.resolveSetpoints(step.plant,step.dt_control)
  if sp==false then M.finishControlStep(c,step);return end
  M.applySetpoints(step.plant, sp, step.dt_control, step)
  M.finishControlStep(c, step)
end

function M.stop()
  initialized = false
  steeringMap=nil
  resetAutonomousSlew()
  if manualInput then manualInput.stop(); manualInput = nil end
  inputSource = 0
  if gtStateController and gtStateController.setTorqueMapLib then
    gtStateController.setTorqueMapLib(nil)
  end
  torqueMapLib = nil
  restoreTorqueMapAfterReset = false
  log('I', logTag, 'LLC stopped')
end

function M.reset()
  resetAutonomousSlew()
  wirePlan,lastPlanPacket,lastPlanSequence,lastPlanSource=nil,nil,nil,nil
  restoreTorqueMapAfterReset = torqueMapLib ~= nil and gtStateController ~= nil
    and type(gtStateController.setTorqueMapLib) == 'function'
  if manualInput then manualInput.release() end
  inputSource = 0
  ctrlState = {
    integral_omega = 0,
    u_prev = 0,
    u_ff_smooth = 0,
    integral_steer = 0,
    last_omega_des = nil,
    last_pi_term = 0,
    last_t_cmd = 0,
    last_plant_brake = 0,
    last_torque_min = 0,
    last_torque_max = 0,
  }
  setpoints = {
    torque_des = nil, omega_des = nil, steer_des_rad = nil,
    steer_input_direct = nil,
    brake_des = nil, throttle_override = nil,
  }
  trajectory = { x = {}, y = {}, wr = {}, steer = {}, torque = {}, targetCount = 0 }
  selection = {}
  lastResolvedSteer, lastResolvedSequence = nil, nil
  updateAccum = 0
  lastCommandTime = 0
  active = false
  cmdMode = CMD_MODE.none
  verboseLastSim = 0
  watch.active = false
  watch.cmdMode = CMD_MODE.none
  watch.setpointSig = nil
  log('I', logTag, 'LLC reset')
end

function M.getStatus()
  return {
    isRunning = common and common.isRunning,
    active = active,
    manualInput = manualInput and manualInput.getStatus() or nil,
    inputSource = inputSource == 2 and 'external' or (inputSource == 1 and 'manual' or 'inactive'),
    cmdMode = cmdMode,
    controlAuthority = controlAuthority,
    trajectoryPick = trajectoryPick,
    trajectoryPreview = trajectoryPreview,
    autonomousRateLimits = autonomousRateLimits,
    brakeCoordination = brakeCoordination,
    selection = selection,
    torqueMapStem = torqueMapStem,
    hasTorqueMap = hasTorqueMap(),
    gains = gains,
    steering = steeringMap and {count=steeringMap.count,
      minimum_rad=steeringMap.minimum, maximum_rad=steeringMap.maximum} or nil,
    ctrlState = ctrlState,
    setpoints = setpoints,
  }
end

return M
