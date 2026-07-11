-- Low-level controller: scalar or trajectory setpoints.
--
-- Torque path priority (deterministic, mutually exclusive per step):
--   1. throttle_override  — direct [0,1], bypasses torque map + PI
--   2. torque map + inverse — needs ffi lib; skipped if no map
--
-- Torque / wheel-speed composition (when not in throttle override):
--   torque_des     — feedforward torque [N·m] → inverse (nil = 0 FF)
--   omega_des      — rear wheel speed [m/s]; PI adds piTerm [N·m]
--   tCmd           = torque_des + clip(piTerm, t_min, t_max)
--   piTerm         = pi_enable * (kp*e + ki*∫e), e = omega_des - omega
--   Omitting a scalar field clears it (replace semantics on each cmd).
--
-- Steering: independent PI on roadwheel [rad]; gated by control_mode authority.
-- Brake: while LLC is active, brake is asserted every tick (0 unless brake_des set)
--   so a manual brake press cannot latch after the pedal is released.
local M = {}

local logTag = 'controller_llc'
local ffi = require('ffi')

local max = math.max
local min = math.min

local REF_DT = 0.01

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
local TRAJECTORY_PICK = {
  next = 'next',
  lerp = 'lerp',
  closest = 'closest',
}
local trajectoryPick = TRAJECTORY_PICK.next

-- ---------------------------------------------------------------------------
-- Torque map (ffi) — loaded once from OpenController payload (common.torqueMapPath)
-- ---------------------------------------------------------------------------

ffi.cdef([[
  float drivetrain_inverse_throttle(float engine_speed_rads, float boost_pressure,
                                    float rear_wheelspeed_ms, float torque_cmd);
  float drivetrain_forward_torque(float engine_speed_rads, float boost_pressure,
                                  float rear_wheelspeed_ms, float throttle_cmd);
  extern const float drivetrain_wheel_gain;
]])

local torqueMapLib = nil
local torqueMapStem = nil

local function loadTorqueMap(libPath)
  if torqueMapLib then return true end
  libPath = libPath or ''
  if libPath == '' then
    log('W', logTag, 'No torqueMapPath; torque / wheel-speed control disabled')
    return false
  end
  local ok, lib = pcall(ffi.load, libPath)
  if not ok or not lib then
    log('E', logTag, 'ffi.load failed: ' .. tostring(lib))
    return false
  end
  torqueMapLib = lib
  log('I', logTag, 'Loaded torque map from ' .. tostring(libPath))
  return true
end

local function hasTorqueMap()
  return torqueMapLib ~= nil
end

local function inverseThrottle(engineSpeedRads, boostPressure, rearWheelspeedMs, torqueCmd)
  if not torqueMapLib then return 0 end
  return tonumber(torqueMapLib.drivetrain_inverse_throttle(
    engineSpeedRads, boostPressure, rearWheelspeedMs, torqueCmd
  ))
end

local function wheelGain()
  if not torqueMapLib then return 1 end
  return tonumber(torqueMapLib.drivetrain_wheel_gain)
end

-- ---------------------------------------------------------------------------
-- Gains / state
-- ---------------------------------------------------------------------------

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
  steering_to_input = 1.0,
  command_timeout = 0.5,
}

local controlAuthority = CONTROL_AUTHORITY.full
local cmdMode = CMD_MODE.none

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

local function inverseThrottleBracketed(plant, torqueCmd)
  local tMin = plant.torque_min or 0
  local tMax = plant.torque_max or 0
  ctrlState.last_torque_min = tMin
  ctrlState.last_torque_max = tMax
  if torqueCmd >= tMax then return 1.0 end
  if torqueCmd <= tMin then return 0.0 end
  local uMlp = inverseThrottle(
    plant.engine_speed_rads,
    plant.boost_pressure,
    plant.rear_wheelspeed_ms,
    torqueCmd
  )
  local rear_wr = plant.rear_wheelspeed_ms
  if rear_wr > 2.0 then
    return uMlp
  end
  local span = tMax - tMin
  if span <= 1e-9 then
    return clip(uMlp, 0, 1)
  end
  local uLin = (torqueCmd - tMin) / span
  return clip(max(uMlp, uLin), 0, 1)
  -- return clip(max(uMlp, uLin), 0, 1)
end

local function stepTorqueLoop(plant, sp, dt)
  if sp.torque_des == nil and sp.omega_des == nil and sp.throttle_override == nil then
    return nil
  end
  if sp.throttle_override ~= nil then
    local u = clip(sp.throttle_override, 0, 1)
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

  local integral = ctrlState.integral_omega
  local piTerm = 0
  local e = 0

  if omegaDes ~= nil and gains.pi_enable ~= 0 then
    e = omegaDes - omega
    integral = integral + e * dt
    integral = clip(integral, gains.i_min, gains.i_max)
    local kpEff, kiEff = scaledTorqueGains(dt)
    local piTermRaw = gains.pi_enable * (kpEff * e + kiEff * integral)
    piTerm = clip(piTermRaw, gains.t_min, gains.t_max)
    -- Anti-windup when piTerm saturates (back off integral for this step).
    if piTerm ~= piTermRaw then
      if (piTerm >= gains.t_max and e > 0) or (piTerm <= gains.t_min and e < 0) then
        integral = integral - e * dt
      end
    end
    ctrlState.integral_omega = integral
  else
    ctrlState.integral_omega = 0
  end

  local tCmd = torqueDes + piTerm

  local uFfRaw = inverseThrottleBracketed(plant, tCmd)

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

  -- Throttle saturation anti-windup (cannot push harder into the plant).
  if omegaDes ~= nil and gains.pi_enable ~= 0 then
    if (uCmd >= 1 and e > 0) or (uCmd <= 0 and e < 0) then
      ctrlState.integral_omega = clip(
        ctrlState.integral_omega - e * dt,
        gains.i_min,
        gains.i_max
      )
    end
  end

  ctrlState.u_ff_smooth = uFf
  ctrlState.u_prev = uCmd
  ctrlState.last_pi_term = piTerm
  ctrlState.last_t_cmd = tCmd
  return uCmd
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
  local denom = gains.steering_to_input
  if denom == 0 then denom = 1 end
  local steerInput = clip(steerCmdRad / denom, -1, 1)

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

local function trajectorySetpoints(plant)
  local i1, i2, t = findClosestSegment(plant.pos_x, plant.pos_y)
  return {
    omega_des = pickAlongTrajectory(trajectory.wr, i1, i2, t),
    torque_des = pickAlongTrajectory(trajectory.torque, i1, i2, t),
    steer_des_rad = pickAlongTrajectory(trajectory.steer, i1, i2, t),
    brake_des = setpoints.brake_des,
    throttle_override = setpoints.throttle_override,
  }
end

local function scalarSetpoints()
  return {
    torque_des = setpoints.torque_des,
    omega_des = setpoints.omega_des,
    steer_des_rad = setpoints.steer_des_rad,
    steer_input_direct = setpoints.steer_input_direct,
    brake_des = setpoints.brake_des,
    throttle_override = setpoints.throttle_override,
  }
end

-- ---------------------------------------------------------------------------
-- Actuation + authority gating
-- ---------------------------------------------------------------------------

local function applyActuation(plant, sp, dt)
  local allowThrottle = controlAuthority == CONTROL_AUTHORITY.full
      or controlAuthority == CONTROL_AUTHORITY.torque_only
  local allowSteer = controlAuthority == CONTROL_AUTHORITY.full
      or controlAuthority == CONTROL_AUTHORITY.steering_only
  local allowBrake = controlAuthority == CONTROL_AUTHORITY.full

  if allowThrottle then
    local throttle = stepTorqueLoop(plant, sp, dt)
    if throttle ~= nil then
      input.event('throttle', throttle, FILTER_AI)
      electrics.values.throttle = throttle
      electrics.values.throttle_input = throttle
    end
  end

  if allowSteer then
    if sp.steer_input_direct ~= nil then
      local steer = clip(sp.steer_input_direct, -1, 1)
      input.event('steering', steer, FILTER_AI)
      electrics.values.steering_input = steer
    else
      local steer = stepSteeringLoop(plant, sp, dt)
      if steer ~= nil then
        input.event('steering', steer, FILTER_AI)
        electrics.values.steering_input = steer
      end
    end
  end

  -- Assert brake every tick while active (0 clears a latched manual brake press).
  local b = 0
  if allowBrake and sp.brake_des ~= nil then
    b = clip(sp.brake_des, 0, 1)
  end
  input.event('brake', b, FILTER_AI)
  electrics.values.brake = b
  electrics.values.brake_input = b

  if (ctrlState.last_plant_brake or 0) > 0.05 and b <= 0 then
    resetSpeedLoopState()
  end
  ctrlState.last_plant_brake = plant.brake
end

-- ---------------------------------------------------------------------------
-- Split control-step API (for controller_manager + optional feedback plugins)
-- ---------------------------------------------------------------------------

function M.prepareControlStep(dt, c)
  common = c or common
  if not common or not common.isRunning then return nil end

  updateAccum = updateAccum + dt
  if updateAccum < common.controllerRate then return nil end
  updateAccum = updateAccum - common.controllerRate

  nowSim = common.getSimTime()

  if active and (nowSim - lastCommandTime) > gains.command_timeout then
    active = false
    cmdMode = CMD_MODE.none
    resetSpeedLoopState()
    ctrlState.integral_steer = 0
    ctrlState.last_omega_des = nil
    log('I', logTag, string.format('Command timeout — manual control (t=%.3f)', nowSim))
    watch.active = false
    watch.cmdMode = CMD_MODE.none
    watch.setpointSig = nil
  end

  if common.isBypassed or not active then
    return nil
  end

  local cs = common.controlStateOut
  if not cs or cs.t < 0 then return nil end

  local doVerbose = verboseEnabled and (nowSim - verboseLastSim) >= VERBOSE_PERIOD
  return {
    plant = readPlant(cs),
    dt_control = common.controllerRate,
    doVerbose = doVerbose,
    tLoop0 = doVerbose and os.clock() or nil,
    nowSim = nowSim,
  }
end

function M.resolveSetpoints(plant)
  if cmdMode == CMD_MODE.trajectory then
    return trajectorySetpoints(plant)
  end
  return scalarSetpoints()
end

function M.getRawTargets()
  local raw = {
    mode = cmdMode,
    scalar = {
      torque_des = setpoints.torque_des,
      omega_des = setpoints.omega_des,
      steer_des_rad = setpoints.steer_des_rad,
      steer_input_direct = setpoints.steer_input_direct,
      brake_des = setpoints.brake_des,
      throttle_override = setpoints.throttle_override,
    },
    trajectory = {
      x = trajectory.x,
      y = trajectory.y,
      wr = trajectory.wr,
      steer = trajectory.steer,
      torque = trajectory.torque,
      targetCount = trajectory.targetCount,
    },
  }
  return raw
end

function M.applySetpoints(plant, sp, dt, step)
  applyActuation(plant, sp, dt)

  if gtStateController and gtStateController.setCustomField then
    local tCmd = ctrlState.last_t_cmd
    if sp.torque_des == nil and sp.omega_des == nil then
      tCmd = 0
    end
    gtStateController.setCustomField('target_torque', tCmd)
    gtStateController.setCustomField('target_wr', sp.omega_des or 0)
    gtStateController.setCustomField('target_steer', sp.steer_des_rad or 0)
    gtStateController.setCustomField('target_pi_torque', ctrlState.last_pi_term or 0)
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
        'SimTime=%.3f mode=%s wr_t=%.2f wr=%.2f tor_des=%.2f tCmd=%.2f piT=%.2f Tmin=%.2f Tmax=%.2f steer_t=%.3f delta=%.3f thr=%.3f brk=%.3f wg=%.4f lat=%.1fms loop=%.3fms',
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

local function loadTrajectory(data)
  local n = #data.x
  if #data.y ~= n or #data.wr ~= n or #data.steer ~= n then
    log('E', logTag, 'Trajectory array length mismatch')
    return false
  end
  local torqueArr = data.desired_torque or data.torque
  if type(torqueArr) ~= 'table' or #torqueArr ~= n then
    log('E', logTag, 'Trajectory torque array required and must match path length')
    return false
  end
  trajectory.x = data.x
  trajectory.y = data.y
  trajectory.wr = data.wr
  trajectory.steer = data.steer
  trajectory.torque = torqueArr
  -- local wg = wheelGain()
  -- for i = 1, n do
  --   trajectory.torque[i] = torqueArr[i] / wg
  -- end
  trajectory.targetCount = n
  -- Trajectory replaces scalar setpoints; do not inherit throttle_override / brake.
  setpoints.torque_des = nil
  setpoints.omega_des = nil
  setpoints.steer_des_rad = nil
  setpoints.steer_input_direct = nil
  setpoints.brake_des = nil
  setpoints.throttle_override = nil
  resetSpeedLoopState()
  ctrlState.last_omega_des = nil
  cmdMode = CMD_MODE.trajectory
  return true
end

local function loadScalar(data)
  local hasTarget = (
    data.torque ~= nil or data.wheel_speed ~= nil or data.steering ~= nil
    or data.steering_input ~= nil or data.brake ~= nil or data.throttle ~= nil
  )
  if not hasTarget then return false end

  -- Replace semantics: omitted fields are cleared (no stale throttle_override etc.)
  setpoints.torque_des = nil
  setpoints.omega_des = nil
  setpoints.steer_des_rad = nil
  setpoints.steer_input_direct = nil
  setpoints.brake_des = nil
  setpoints.throttle_override = nil

  if data.torque ~= nil then setpoints.torque_des = tonumber(data.torque) end
  if data.wheel_speed ~= nil then setpoints.omega_des = tonumber(data.wheel_speed) end
  if data.steering ~= nil then setpoints.steer_des_rad = tonumber(data.steering) end
  if data.brake ~= nil then setpoints.brake_des = tonumber(data.brake) end
  if data.throttle ~= nil then setpoints.throttle_override = tonumber(data.throttle) end
  -- Direct BeamNG input — bypasses steering_to_input (calibration / unknown scale).
  if data.steering_input ~= nil then
    setpoints.steer_input_direct = tonumber(data.steering_input)
    setpoints.steer_des_rad = nil
  end
  syncSpeedLoopStateForSetpoints()
  cmdMode = CMD_MODE.scalar
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

local GAIN_KEYS = {
  'pi_enable', 'kp', 'ki', 'i_min', 'i_max', 't_min', 't_max',
  'u_smooth', 'du_max',
  'steer_pi_enable', 'steer_kp', 'steer_ki', 'steer_i_min', 'steer_i_max',
  'steering_to_input', 'command_timeout',
}

function M.calibrate(params)
  if type(params) ~= 'table' then return end

  local prevAuthority = controlAuthority
  local tuneKeys = {}

  if params.torque_map ~= nil then
    torqueMapStem = tostring(params.torque_map)
    tuneKeys[#tuneKeys + 1] = 'torque_map'
  end

  if params.control_mode ~= nil then
    local mode = params.control_mode
    if type(mode) == 'string' and CONTROL_AUTHORITY[mode] then
      controlAuthority = mode
      tuneKeys[#tuneKeys + 1] = 'control_mode'
    end
  end

  local pick = params.trajectory_pick
  if type(pick) == 'string' and TRAJECTORY_PICK[pick] then
    trajectoryPick = pick
    tuneKeys[#tuneKeys + 1] = 'trajectory_pick'
  end

  if params.verbose ~= nil then
    verboseEnabled = params.verbose == true
    tuneKeys[#tuneKeys + 1] = 'verbose'
  end

  local nested = params.gains
  if type(nested) == 'table' then
    for _, key in ipairs(GAIN_KEYS) do
      if nested[key] ~= nil then
        gains[key] = tonumber(nested[key]) or gains[key]
        tuneKeys[#tuneKeys + 1] = key
      end
    end
  end

  for _, key in ipairs(GAIN_KEYS) do
    if params[key] ~= nil then
      gains[key] = tonumber(params[key]) or gains[key]
      tuneKeys[#tuneKeys + 1] = key
    end
  end

  if #tuneKeys > 0 then
    log('I', logTag, string.format(
      'tune: control_mode=%s keys=%s',
      controlAuthority,
      table.concat(tuneKeys, ',')
    ))
  end
  if prevAuthority ~= controlAuthority then
    log('I', logTag, string.format('authority: %s → %s', prevAuthority, controlAuthority))
  end
end

function M.onTune(data)
  M.calibrate(data)
  return true
end

-- ---------------------------------------------------------------------------
-- Plugin lifecycle
-- ---------------------------------------------------------------------------

function M.init(c)
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
    'LLC init torque_map=%s wheel_gain=%.4f authority=%s torque_ctrl=%s verbose=%s',
    tostring(torqueMapStem), wg, controlAuthority, tostring(hasTorqueMap()),
    tostring(verboseEnabled)
  ))
  return true
end

function M.update(dt, c)
  local step = M.prepareControlStep(dt, c)
  if not step then return end
  local sp = M.resolveSetpoints(step.plant)
  M.applySetpoints(step.plant, sp, step.dt_control, step)
  M.finishControlStep(c, step)
end

function M.stop()
  if gtStateController and gtStateController.setTorqueMapLib then
    gtStateController.setTorqueMapLib(nil)
  end
  torqueMapLib = nil
  log('I', logTag, 'LLC stopped')
end

function M.reset()
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
    cmdMode = cmdMode,
    controlAuthority = controlAuthority,
    trajectoryPick = trajectoryPick,
    torqueMapStem = torqueMapStem,
    hasTorqueMap = hasTorqueMap(),
    gains = gains,
    ctrlState = ctrlState,
    setpoints = setpoints,
  }
end

return M
