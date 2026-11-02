-- Optional final physical-command layer. No socket, trajectory lookup or actuator I/O.
-- C owns model evaluation, observer and QP; this module owns command timing/policy.
-- See FEEDBACK_CONTRACT.md for phase definitions, units and failure semantics.
local ffi = require('ffi')
local abs = math.abs
local max = math.max
local min = math.min
local rad = math.rad
local huge = math.huge
local pi = math.pi
local sqrt = math.sqrt
local M = {}
local common, environment, lib, gt, cfg
local state, vehicleQuery, query, commandQuery, measurement, observer, settings, bias, bounds
local nominal, candidate, diagnostic, alignment
local previousAngle, previousTorque, previousNominalTorque, previousTime, observerTime
local previousSource, measuredSteeringRate
local infeasiblePenaltyScale
local report = {}
local lowSpeed = true
local preparedTime, preparedRevision, preparedBrake, prepareStatus, observerStatus
local POSITION_KEYS = {'x', 'y', 'z'}
local VELOCITY_KEYS = {'vx', 'vy', 'vz'}
local ANGULAR_RATE_KEYS = {'p', 'q', 'r'}
local REQUIRED_STATE_KEYS = {
  'x', 'y', 'z', 'vx', 'vy', 'vz', 'p', 'q', 'r',
  'delta_l', 'delta_r', 'w_rl', 'w_rr', 'brake', 'pbrake',
}

-- Fallback settings; vehicle-specific defaults belong to vehicle_catalog.yaml.
-- mode: off skips evaluation, shadow logs only, active executes QP/recovery.
-- apply_bias gates compensation, not estimation. tau/phi/alpha and packed SPD P
-- define the roll score/barrier; phi is a tuning scale, not a proven angle cap.
-- recovery_enabled selects alignment/reduce on infeasibility for either source.
-- Disable it only for the native maximum-margin/penalty comparison.
-- Speed thresholds use the full velocity norm and hysteresis. Objective scales
-- define a fixed command-distance metric, independent of changing actuator bounds.
-- torque_correction=false locks torque to the physically clipped nominal target;
-- the QP then modifies steering only, and no filter-side torque slew is applied.
-- Optional torque_rate_max [Nm/s] applies only when torque_correction=true.
-- Optional infeasible_steering_penalty=rho>0 (steering-only mode) retains nominal
-- command cost on an infeasible row. Use 0 for the existing max-margin policy.
-- Feasible hard-QP decisions are unchanged; status 1 still means infeasible.
-- Explicit steering_min/max_rad are startup-only; tune may not raise rate capability.
local DEFAULTS = {
  mode = 'shadow', apply_bias = true, torque_correction = true, recovery_enabled = true,
  infeasible_steering_penalty = 0, -- 0: maximum-margin; positive: finite scalar slack penalty
  tau_s = 0.3, phi_limit_rad = rad(35), alpha_per_s = 1 / 0.3,
  P = {37 / 16, 31 / 16, 8 / 16, 52 / 16, 13 / 16, 7 / 16},
  steering_rate_max = 0.9,
  speed_enable_m_s = 1.0, speed_disable_m_s = 0.5,
  -- Fixed metric: 0.1 rad target-angle / 0.01 s, and 500 Nm. Not live bound widths.
  steering_rate_scale = 10, torque_scale = 500,
  observer_gain_per_s = 10, observer_max_gap_s = 0.05, max_state_age_s = 0.1,
}
local KEYS = {steering_min_rad=true, steering_max_rad=true, torque_rate_max=true, infeasible_steering_penalty=true}
for key in pairs(DEFAULTS) do KEYS[key] = true end
local FIELDS = {
  'rf_status', 'rf_intervened', 'rf_score', 'rf_margin', 'rf_bias',
  'rf_nominal_rate', 'rf_nominal_torque', 'rf_filtered_rate', 'rf_filtered_torque',
  'rf_observer_status', 'rf_torque_slew_excess',
  'rf_steering_override', 'rf_torque_override', 'rf_measured_rate', 'rf_measured_rate_valid',
  'rf_nominal_margin', 'rf_maximum_margin', 'rf_manual', 'rf_branch_override', 'rf_angle_reentry',
  'rf_command_anchor_offset',
  'rf_handback_pending', 'rf_alignment_target', 'rf_alignment_cost',
  'rf_brake_measured', 'rf_brake', 'rf_brake_h_jump',
}
local function finite(x)
  return type(x) == 'number' and x == x and abs(x) < huge
end
local function nativeFinite(x)
  return finite(x) and finite(tonumber(ffi.new('vehicle_real', x)))
end
local function positive(x)
  return nativeFinite(x) and tonumber(ffi.new('vehicle_real', x)) > 0
end

-- Validate complete settings before installing any change. P uses packed symmetric
-- order 11,12,13,22,23,33; the Cholesky pivots below must be strictly positive.
local function validate(c)
  assert(c.mode == 'off' or c.mode == 'shadow' or c.mode == 'active', 'mode must be off, shadow or active')
  assert(type(c.recovery_enabled) == 'boolean', 'recovery_enabled must be boolean')
  assert(type(c.apply_bias) == 'boolean', 'apply_bias must be boolean')
  assert(type(c.torque_correction) == 'boolean', 'torque_correction must be boolean')
  for _,key in ipairs({'tau_s','phi_limit_rad','alpha_per_s','steering_rate_max',
    'steering_rate_scale','torque_scale','speed_enable_m_s','speed_disable_m_s','observer_gain_per_s','observer_max_gap_s','max_state_age_s'}) do
    assert(positive(c[key]), 'Positive roll_filter.'..key..' required')
  end
  assert(c.speed_disable_m_s < c.speed_enable_m_s, 'Speed disable threshold must be below enable')
  assert(c.phi_limit_rad < pi/2, 'phi_limit_rad must be below pi/2')
  assert(finite(c.steering_min_rad) and finite(c.steering_max_rad)
    and c.steering_min_rad < c.steering_max_rad, 'Explicit steering angle bounds required')
  assert(c.torque_rate_max == nil or positive(c.torque_rate_max), 'Invalid torque_rate_max')
  assert(nativeFinite(c.infeasible_steering_penalty) and c.infeasible_steering_penalty >= 0,
    'Invalid infeasible_steering_penalty')
  assert(c.infeasible_steering_penalty == 0 or not c.torque_correction,
    'infeasible_steering_penalty requires torque_correction=false')
  local packed = c.P
  assert(type(packed) == 'table' and #packed == 6, 'P must contain six packed entries')
  local p = {}
  for i,v in ipairs(packed) do
    assert(nativeFinite(v), 'P entry is not representable in native precision')
    p[i] = tonumber(ffi.new('vehicle_real', v))
  end
  assert(p[1] > 0, 'P must be SPD')
  local d2 = p[4] - p[2]*p[2]/p[1]
  local off = p[5] - p[2]*p[3]/p[1]
  assert(d2 > 0 and p[6]-p[3]*p[3]/p[1]-off*off/d2 > 0, 'P must be SPD')
  if c.infeasible_steering_penalty > 0 then
    local cofactor = c.P[4]*c.P[6]-c.P[5]*c.P[5]
    local determinant = c.P[1]*cofactor-c.P[2]*(c.P[2]*c.P[6]-c.P[3]*c.P[5])
      +c.P[3]*(c.P[2]*c.P[5]-c.P[3]*c.P[4])
    local level = c.phi_limit_rad*c.phi_limit_rad*determinant/cofactor
    local penalty = c.infeasible_steering_penalty*(c.steering_rate_scale/(c.alpha_per_s*level))^2
    assert(finite(penalty), 'Infeasible steering penalty overflow')
  end
end
local function merged(base, changes)
  local c = {}
  for k,v in pairs(base) do c[k] = v end
  for k,v in pairs(changes) do
    assert(KEYS[k], 'Unknown roll_filter setting: '..tostring(k))
    c[k] = v
  end
  validate(c)
  local p = {}; for i=1,6 do p[i] = c.P[i] end; c.P = p
  return c
end
local function install(c)
  cfg = c
  settings[0].tau_s, settings[0].phi_limit_rad = c.tau_s, c.phi_limit_rad
  settings[0].alpha_per_s = c.alpha_per_s
  for i=1,6 do settings[0].P[i-1] = c.P[i] end
  bounds[0].scale.steering_rate, bounds[0].scale.torque = c.steering_rate_scale, c.torque_scale
  infeasiblePenaltyScale = nil
  if c.infeasible_steering_penalty > 0 then
    -- Cache lambda=rho*(rate_scale/(alpha*C))^2; C is the deployed score level.
    local p = c.P
    local cofactor = p[4]*p[6]-p[5]*p[5]
    local determinant = p[1]*cofactor-p[2]*(p[2]*p[6]-p[3]*p[5])
      +p[3]*(p[2]*p[5]-p[3]*p[4])
    local level = c.phi_limit_rad*c.phi_limit_rad*determinant/cofactor
    infeasiblePenaltyScale = c.infeasible_steering_penalty
      *(c.steering_rate_scale/(c.alpha_per_s*level))^2
    assert(finite(infeasiblePenaltyScale), 'Infeasible steering penalty overflow')
  end
end
local function publish(status)
  report.status, report.mode = status, cfg.mode
  report.steering_override = report.steering_override == true
  report.torque_override = report.torque_override == true
  report.rf_steering_override = report.steering_override and 1 or 0
  report.rf_torque_override = report.torque_override and 1 or 0
  report.rf_measured_rate = measuredSteeringRate
  report.rf_measured_rate_valid = finite(measuredSteeringRate) and 1 or 0
  for _,name in ipairs(FIELDS) do
    local value = report[name]
    gt.setCustomField(name, finite(value) and value or 0)
  end
  gt.setCustomField('rf_status', status)
end

-- Startup requires an environment plus explicit vehicle steering-angle limits.
-- Everything else has visible defaults above. Omit the whole module to disable.
function M.init(c, config)
  assert(not lib, 'Roll filter already initialized')
  local validated = merged(DEFAULTS, config)
  common, environment = c, assert(c.environment, 'roll_filter requires environment')
  lib = environment.getNativeLibrary()
  assert(lib.rf_abi_version() == 4, 'Unsupported roll filter ABI')
  gt = assert(c.gtStateManager.getGtStateController(c.gtStateSensorId), 'GTState required')
  assert(gt.registerCustomField and gt.setCustomField, 'GTState custom fields required')
  state, vehicleQuery = ffi.new('vehicle_state[1]'), ffi.new('vehicle_query[1]')
  query, measurement = ffi.new('rf_query[1]'), ffi.new('rf_measurement[1]')
  commandQuery = ffi.new('rf_query[1]')
  observer, settings, bias = ffi.new('rf_observer[1]'), ffi.new('rf_settings[1]'), ffi.new('rf_bias[1]')
  bounds, nominal, candidate = ffi.new('rf_bounds[1]'), ffi.new('rf_command[1]'), ffi.new('rf_command[1]')
  diagnostic, alignment = ffi.new('rf_diagnostics[1]'), ffi.new('rf_alignment[1]')
  install(validated)
  for _,name in ipairs(FIELDS) do gt.registerCustomField(name, 0) end
  M.reset()
  return true
end

-- Atomic tune namespace: {roll_filter={...}}. Angle limits are startup-only;
-- steering rate can be reduced/tuned up to its startup capability. Rejected tunes
-- preserve all previous settings. Supported keys are DEFAULTS plus torque_rate_max;
-- steering_min/max_rad are forbidden here. reset_observer=true is a one-shot action.
-- P is packed 11,12,13,22,23,33 and must remain SPD. Numeric scales/gains/times and
-- speed thresholds must be positive; disable speed < enable speed. Off releases
-- actuation and skips evaluation; shadow retains diagnostics without ownership.
-- Gain changes reset before the next sample; patch updates alone do not reset.
local steeringRateCapability
function M.onTune(changes)
  assert(lib, 'Roll filter is not initialized')
  if type(changes) ~= 'table' then return false end
  if changes.steering_min_rad ~= nil or changes.steering_max_rad ~= nil then return false end
  if changes.reset_observer ~= nil and type(changes.reset_observer) ~= 'boolean' then return false end
  local updates = {}
  for key,value in pairs(changes) do if key ~= 'reset_observer' then updates[key] = value end end
  local ok,c = pcall(merged, cfg, updates)
  if not ok or c.steering_rate_max > steeringRateCapability then return false end
  if changes.reset_observer or c.observer_gain_per_s ~= cfg.observer_gain_per_s then
    lib.rf_observer_reset(observer)
    observerTime, preparedTime = nil,nil
  end
  install(c)
  return true
end

function M.reset()
  if not lib then return end
  steeringRateCapability = steeringRateCapability or cfg.steering_rate_max
  lib.rf_observer_reset(observer)
  previousAngle, previousTorque, previousNominalTorque, previousTime, observerTime = nil,nil,nil,nil,nil
  previousSource, measuredSteeringRate = nil,nil
  preparedTime, preparedRevision, prepareStatus, observerStatus = nil,nil,nil,1
  report = {}
  lowSpeed = true
  publish(-7) -- adapter idle: no active physical command processed yet
end
function M.stop()
  if not lib then return end
  M.reset()
  common, environment, lib, gt, cfg, steeringRateCapability = nil,nil,nil,nil,nil,nil
end
function M.getStatus() return report end

-- Shared command limits remain available in off/shadow/unavailable modes.
-- Returns steering [rad/s], optional torque [Nm/s]; no configuration copy.
function M.getRateLimits()
  assert(cfg, 'Roll filter is not initialized')
  return cfg.steering_rate_max, cfg.torque_rate_max
end

-- Small infeasible policy; native alignment reuses the prepared terrain/model.
-- Command bounds are already intersected by step(); never clip after selection.
local function recover(q, anchor, torque, dt, rlo, rhi, tlo, thi, parameters)
  local status = lib.rf_alignment_target(q, parameters,
    cfg.steering_min_rad, cfg.steering_max_rad, alignment)
  if status ~= 0 then return nil, nil, tonumber(status) end
  local target = tonumber(alignment[0].steering_angle_rad)
  report.alignment_target, report.alignment_cost = target, tonumber(alignment[0].cost)
  report.rf_alignment_target, report.rf_alignment_cost = report.alignment_target, report.alignment_cost
  report.alignment_status = tonumber(alignment[0].front_load_N) <= 1 and 1 or 0
  report.fallback_torque_mode, report.fallback_torque_target = 'reduce', min(torque,0)
  return min(rhi,max(rlo,(target-anchor)/dt)), min(thi,max(tlo,min(torque,0))), 0
end

function M.isEnabled() return cfg ~= nil and cfg.mode ~= 'off' end

local function observationFailure(status)
  observer[0].valid = 0
  prepareStatus = status
  report = {rf_bias=tonumber(observer[0].bias), rf_observer_status=5}
  observerStatus = 5
  measuredSteeringRate = nil
  publish(status)
  return status
end

-- Called independently of active commands by the manager. Cache preparation
-- only for the SAME state timestamp and patch revision; no repeated projection
-- or duplicate observer update when step() consumes this snapshot in the same tick.
function M.observe(cs, now)
  if not lib or cfg.mode == 'off' then return -7 end
  if not (cs and finite(cs.t) and finite(now) and now >= cs.t
    and now-cs.t <= cfg.max_state_age_s) then
    return observationFailure(-5)
  end
  measuredSteeringRate = cs.steering_rate_valid == true and cs.steering_rate_raw or nil
  -- Near standstill the exported dynamics are not qualified. Use the full body
  -- velocity norm (rotation invariant), with hysteresis; do not evaluate the model
  -- or update its observer below this gate. Re-entry starts with a fresh estimate.
  if not (finite(cs.vx) and finite(cs.vy) and finite(cs.vz)) then
    return observationFailure(-6)
  end
  local speed = sqrt(cs.vx*cs.vx + cs.vy*cs.vy + cs.vz*cs.vz)
  if lowSpeed then
    lowSpeed = speed < cfg.speed_enable_m_s
  else
    lowSpeed = speed <= cfg.speed_disable_m_s
    if lowSpeed then lib.rf_observer_reset(observer) end
  end
  if lowSpeed then
    observerTime, preparedTime, preparedRevision = nil,nil,nil
    prepareStatus, observerStatus = -8,1
    previousAngle, previousTorque, previousNominalTorque, previousTime = nil,nil,nil,nil
    report = {rf_bias=0, rf_observer_status=1}
    publish(-8) -- nominal passthrough: model is not qualified near zero speed
    return -8
  end
  if common.rollInputValid ~= true then return observationFailure(-6) end
  if not (finite(cs.brake) and cs.brake>=0 and cs.brake<=1 and finite(cs.pbrake)) then
    return observationFailure(-6)
  end
  if cs.pbrake ~= 0 then return observationFailure(-3) end
  local revision = environment.getRevision()
  if cs.t == preparedTime and revision == preparedRevision and cs.brake == preparedBrake
    and prepareStatus == 0 then return 0 end
  for _, key in ipairs(REQUIRED_STATE_KEYS) do
    if not finite(cs[key]) then return observationFailure(-6) end
  end
  if type(cs.quat) ~= 'table' then return observationFailure(-6) end
  for i=1,4 do
    if not finite(cs.quat[i]) then return observationFailure(-6) end
  end
  preparedTime, preparedRevision = cs.t, revision
  preparedBrake = cs.brake
  prepareStatus = -1
  -- Phase 3: pack the measured physical state, then project exactly once against
  -- the current received patch. nu is the geometric relative-angle derivative,
  -- generally not body p. C validates the state and handles all terrain derivatives.
  for i, key in ipairs(POSITION_KEYS) do
    state[0].position_world_m[i - 1] = cs[key]
  end
  for i = 1, 4 do
    state[0].quaternion_xyzw[i - 1] = cs.quat[i]
  end
  for i, key in ipairs(VELOCITY_KEYS) do
    state[0].velocity_body_m_s[i - 1] = cs[key]
  end
  for i, key in ipairs(ANGULAR_RATE_KEYS) do
    state[0].angular_velocity_body_rad_s[i - 1] = cs[key]
  end
  state[0].rear_peripheral_speed_m_s = .5*(cs.w_rl+cs.w_rr)
  local status = environment.prepare(state,vehicleQuery)
  if status ~= 0 then return observationFailure(-1) end
  -- GTState brake belongs to this measurement. It must not be replaced by a
  -- newly requested command while updating the acceleration observer.
  status = lib.rf_prepare(vehicleQuery,.5*(cs.delta_l+cs.delta_r),cs.brake,0,query)
  if status ~= 0 then return observationFailure(tonumber(status)) end
  local parameters = environment.getModelParameters()
  status = lib.rf_measure(query,parameters,measurement)
  if status ~= 0 then return observationFailure(tonumber(status)) end

  -- Phase 4: observer runs on NEW state samples, including in shadow or bias-off.
  -- Do not call it twice for one timestamp: native duplicate handling invalidates
  -- an estimate. Time discontinuities/gaps explicitly reset/reinitialize it.
  if observerTime == nil or cs.t ~= observerTime then
    local elapsed = observerTime and cs.t-observerTime or 0
    if elapsed < 0 then lib.rf_observer_reset(observer); elapsed = 0 end
    observerStatus = tonumber(lib.rf_observer_update(observer,elapsed,
      measurement[0].nu,measurement[0].model_acc,cfg.observer_gain_per_s,cfg.observer_max_gap_s))
    observerTime = cs.t
  end
  bias[0].value, bias[0].valid, bias[0].apply = observer[0].bias,observer[0].valid,cfg.apply_bias and 1 or 0
  report.rf_bias, report.rf_observer_status = tonumber(observer[0].bias), observerStatus
  gt.setCustomField('rf_bias', report.rf_bias)
  gt.setCustomField('rf_observer_status', observerStatus)

  prepareStatus = 0
  return 0
end

-- Clear only command history on timeout/bypass, not an otherwise valid observer.
function M.idle()
  if not lib then return end
  previousAngle, previousTorque, previousNominalTorque, previousTime = nil,nil,nil,nil
  previousSource = nil
  report = {rf_bias=tonumber(observer[0].bias), rf_observer_status=observerStatus}
  publish(prepareStatus ~= 0 and prepareStatus or -7)
end

-- Inputs: normalized current state cs; next roadwheel target [rad]; total nominal
-- rear torque [Nm] AFTER PI; control interval [s]; execution time [vehicle s].
-- Optional reference rates are retained separately; absolute targets remain the
-- authority here. Outputs: next angle [rad], absolute torque [Nm], decision table.
-- context is borrowed for this call only. Caller executes
-- them once. Throttle smoothing remains in the LLC and is an execution limitation.
function M.step(cs, angle, torque, dt, now, referenceRate, referenceTorqueRate, context)
  assert(lib, 'Roll filter is not initialized')
  if cfg.mode == 'off' then M.idle(); return angle,torque,report end
  local manual = context and context.source == 'manual' or false
  local source = manual and 'manual' or 'external'
  if previousSource and previousSource ~= source then
    previousAngle, previousTorque, previousNominalTorque, previousTime = nil,nil,nil,nil
  end
  previousSource = source
  report = {rf_nominal_torque=torque, reference_rate=referenceRate,
    reference_torque_rate=referenceTorqueRate, rf_manual=manual and 1 or 0, source=source}
  local function failure(status)
    if status == -2 then
      measuredSteeringRate = nil
    end
    report.rf_bias, report.rf_observer_status = tonumber(observer[0].bias), observerStatus
    report.steering_override, report.torque_override = false,false
    report.rf_manual, report.source = manual and 1 or 0,source
    report.reason = status == -9 and 'measured_rate_unavailable'
      or status == -11 and 'command_capability_unavailable' or 'filter_unavailable'
    -- Explicit invalid-input policy: retain nominal command, never execute NaNs.
    -- These are not certified safe commands; status identifies unavailable filtering.
    publish(status)
    if manual or status == -8 then
      previousAngle, previousTorque, previousNominalTorque, previousTime = nil,nil,nil,nil
    elseif finite(angle) and finite(torque) and finite(now) then
      previousAngle, previousTorque, previousNominalTorque, previousTime = angle,torque,torque,now
    end
    return angle, torque, report
  end
  assert(finite(angle) and finite(torque), 'Nonfinite physical command must not reach actuation')
  if not (positive(dt) and finite(now)) then return failure(-2) end
  if previousTime then dt = now - previousTime end
  if not positive(dt) or dt > cfg.observer_max_gap_s then
    previousAngle, previousTorque = nil,nil
    return failure(-2)
  end
  if not (cs and finite(cs.t) and now >= cs.t and now-cs.t <= cfg.max_state_age_s) then return failure(-5) end

  local observed = M.observe(cs, now)
  if observed ~= 0 then return failure(observed) end
  if manual and not finite(measuredSteeringRate) then return failure(-9) end
  -- BeamNG input is treated as held between updates. Evaluate a new brake
  -- value at the current state; do not disguise a pedal step as a finite ramp.
  -- The native ABI also accepts continuous brake rate for other adapters.
  local brake = context and context.brake
  if brake == nil then brake = cs.brake end
  if not (nativeFinite(brake) and brake>=0 and brake<=1) then return failure(-6) end
  commandQuery[0] = query[0]
  commandQuery[0].brake, commandQuery[0].brake_rate_per_s = brake,0
  report.rf_brake_measured,report.rf_brake = cs.brake,brake

  -- Phase 1: consistent nominal steering rate. The previously emitted angle is
  -- the command anchor, not the previous raw reference or the measured lagged angle.
  -- On activation only, seed it from measured steering. Absolute target matching
  -- already includes its reference progression: never add its reference rate again.
  local measuredAngle = .5*(cs.delta_l+cs.delta_r)
  local steeringOwned = manual and previousAngle ~= nil
  local anchor = previousAngle or measuredAngle
  local torqueAnchor = previousTorque or cs.rear_wheel_torque_est
  if not (finite(anchor) and finite(torqueAnchor)) then return failure(-6) end
  if manual then
    -- Measured wheel motion may exceed the angle represented by input +/-1
    -- (compliance/dynamic overshoot/calibration). Preserve that measurement in
    -- the model and rate estimate. Only seed the autonomous COMMAND reference
    -- at the nearest representable endpoint before constructing the QP bounds.
    local steeringMin,steeringMax = context.steering_command_min_rad,context.steering_command_max_rad
    if not (finite(steeringMin) and finite(steeringMax) and steeringMin<steeringMax
      and cfg.steering_min_rad>=steeringMin and cfg.steering_max_rad<=steeringMax) then
      return failure(-11)
    end
    local commandAnchor = min(steeringMax,max(steeringMin,anchor))
    report.rf_command_anchor_offset = commandAnchor-anchor
    report.command_anchor,report.measured_angle = commandAnchor,measuredAngle
    report.command_limit_rad = capability
    anchor = commandAnchor
  end
  -- Free human motion uses measured continuation. While we own steering, that
  -- motion includes OUR correction and cannot approve returning to a different
  -- retained driver target. During ownership, use the same target-matching
  -- convention as automatic commands; the bounded QP performs the handback.
  -- This is a command-space rate, not an estimate of human or actuator motion.
  local rate = manual and not steeringOwned and measuredSteeringRate or (angle-anchor)/dt
  report.nominal_rate_source = manual and not steeringOwned and 'measured_continuation'
    or steeringOwned and 'owned_target_matching' or 'external_target_matching'
  report.rf_handback_pending = steeringOwned and 1 or 0
  report.handback_angle_gap = steeringOwned and angle-anchor or nil
  report.rf_nominal_rate = rate
  report.nominal_torque_rate = previousNominalTorque and (torque-previousNominalTorque)/dt or 0

  -- Phase 2: intersect physical capabilities, command slew and angle reachability.
  -- Fixed objective scales are NOT recomputed from these changing intervals.
  local lo, hi = cs.torque_min, cs.torque_max
  if not (finite(lo) and finite(hi) and lo <= hi) then return failure(-4) end
  -- The autonomous brake allocator owns this cap before QP candidate selection.
  if not manual and context and context.torque_command_max~=nil then
    local cap=context.torque_command_max
    if not finite(cap) then return failure(-4) end
    hi=min(hi,cap)
    if lo>hi then return failure(-4) end
  end
  report.rf_torque_slew_excess = 0
  if not cfg.torque_correction then
    -- A degenerate torque interval retains the exact native barrier evaluation.
    -- Do not anchor it to previous filtered torque: that would preserve earlier
    -- interventions and stop this from being a steering-only comparison.
    local fixedTorque = manual and torque or min(max(torque, lo), hi)
    lo, hi = fixedTorque, fixedTorque
  elseif cfg.torque_rate_max then
    local change = cfg.torque_rate_max * dt
    local physicalLo, physicalHi = lo, hi
    lo = max(physicalLo, torqueAnchor - change)
    hi = min(physicalHi, torqueAnchor + change)
    if lo > hi then
      -- Physical rails have priority over command slew. The nearest physical
      -- torque uniquely minimizes |T - previous_target|, hence the unavoidable
      -- slew violation. Restrict the QP to that point; keep steering available.
      -- Do not post-clip a QP solution or retain an out-of-rail command as history.
      local nearest = min(max(torqueAnchor, physicalLo), physicalHi)
      report.rf_torque_slew_excess = max(0, abs(nearest - torqueAnchor) - change)
      lo, hi = nearest, nearest
    end
  end
  local rlo = max(-cfg.steering_rate_max,(cfg.steering_min_rad-anchor)/dt)
  local rhi = min(cfg.steering_rate_max,(cfg.steering_max_rad-anchor)/dt)
  if manual then
    -- A driver can already be outside the automatic angle range. The command
    -- anchor is within the interface range; return inward at bounded rate.
    rlo = min(cfg.steering_rate_max,rlo)
    rhi = max(-cfg.steering_rate_max,rhi)
    report.rf_angle_reentry = (anchor<cfg.steering_min_rad or anchor>cfg.steering_max_rad) and 1 or 0
  end
  if lo > hi or rlo > rhi then return failure(-4) end
  local handbackReachable = not steeringOwned or (rate>=rlo and rate<=rhi)
  report.handback_reachable = handbackReachable
  bounds[0].low.steering_rate, bounds[0].high.steering_rate = rlo,rhi
  bounds[0].low.torque, bounds[0].high.torque = lo,hi
  nominal[0].steering_rate, nominal[0].torque = rate,torque

  -- Phases 3/4 were prepared independently from command activity; reuse them.
  local status
  local parameters = environment.getModelParameters()
  report.rf_bias, report.rf_observer_status = tonumber(observer[0].bias), observerStatus

  -- Phase 5: one native affine-row QP in steering RATE and ABSOLUTE torque.
  -- RF_INFEASIBLE (1) contains a bounded maximum-margin candidate; it does not
  -- certify recovery. Other failures may contain NaNs and are never executed.
  -- Apply toggles are read each solve, including when the state snapshot is reused.
  bias[0].value, bias[0].valid, bias[0].apply = observer[0].bias,observer[0].valid,cfg.apply_bias and 1 or 0
  status = tonumber(lib.rf_filter_step(commandQuery,parameters,settings,bias,bounds,nominal,candidate,diagnostic))
  if status ~= 0 and status ~= 1 then return failure(status) end
  -- Exact same-state score jump from observed to scheduled brake. Diagnostic
  -- only: a post-step feasible row does not prove pre-step membership survived.
  local tau = cfg.tau_s
  local appliedBias = cfg.apply_bias and observer[0].valid==1 and tonumber(observer[0].bias) or 0
  local w3 = tau*tau*(tonumber(measurement[0].model_acc)+appliedBias)
  local dw3 = tau*tau*(tonumber(diagnostic[0].measurement.model_acc)-tonumber(measurement[0].model_acc))
  local s3 = cfg.P[3]*tonumber(measurement[0].phi)+cfg.P[5]*tau*tonumber(measurement[0].nu)+cfg.P[6]*w3
  report.rf_brake_h_jump = -2*s3*dw3-cfg.P[6]*dw3*dw3
  report.rf_nominal_margin = tonumber(diagnostic[0].qp.nominal_margin)
  report.rf_maximum_margin = tonumber(diagnostic[0].qp.maximum_margin)
  local filteredRate,filteredTorque = tonumber(candidate[0].steering_rate),tonumber(candidate[0].torque)
  if not finite(filteredRate) or not finite(filteredTorque) then return failure(5) end
  if status == 1 and not cfg.recovery_enabled and infeasiblePenaltyScale then
    -- Exact scalar soft solution ONLY when the fixed-torque hard row is infeasible:
    -- min ((r-r_nom)/scale)^2 + rho*(max(0,-margin)/(alpha*C))^2.
    -- Every point in this interval violates the row, so one quadratic branch
    -- suffices. Keep status=1: this is damage limitation, never a safety certificate.
    local row = diagnostic[0].row
    local intercept = tonumber(row.b)+tonumber(row.g_torque)*filteredTorque
    local gain = tonumber(row.g_rate)
    filteredRate = min(rhi,max(rlo,(rate-infeasiblePenaltyScale*gain*intercept)
      /(1+infeasiblePenaltyScale*gain*gain)))
    candidate[0].steering_rate = filteredRate
    diagnostic[0].qp.margin = intercept+gain*filteredRate
    diagnostic[0].qp.scaled_objective = ((filteredRate-rate)/cfg.steering_rate_scale)^2
    diagnostic[0].qp.intervened = filteredRate ~= rate and 1 or 0
  end
  report.rf_filtered_rate, report.rf_filtered_torque = filteredRate,filteredTorque
  report.rf_score, report.rf_margin = tonumber(diagnostic[0].score),tonumber(diagnostic[0].qp.margin)
  report.bias_applied = tonumber(diagnostic[0].bias_applied) ~= 0

  -- Evaluate the unboxed continuation first. An infeasible AUTOMATIC command
  -- box, or a box projection that differs from the human, is not itself an
  -- intervention request. This is an instantaneous model test, not a guarantee
  -- about the driver's next action. Once steering is owned, a favorable row
  -- alone must not bypass the active command's rate/angle bounds and jump to a
  -- remote retained target. Release only when that target is reachable this tick.
  if manual and report.rf_nominal_margin >= 0 and handbackReachable then
    report.reason = 'manual_continuation_acceptable'
    report.rf_intervened, report.rf_margin = 0,report.rf_nominal_margin
    previousAngle, previousTorque, previousNominalTorque, previousTime = nil,nil,torque,now
    publish(status)
    return angle,torque,report
  end

  if cfg.mode == 'active' and status == 1 and cfg.recovery_enabled then
    local selectedRate,selectedTorque,recoveryStatus = recover(commandQuery,anchor,torque,dt,rlo,rhi,lo,hi,parameters)
    if recoveryStatus ~= 0 then
      report.branch_error = recoveryStatus
      return failure(-10)
    end
    filteredRate,filteredTorque = selectedRate,selectedTorque
    report.rf_branch_override = 1
    report.rf_filtered_rate,report.rf_filtered_torque = filteredRate,filteredTorque
    local row = diagnostic[0].row
    report.rf_margin = tonumber(row.b)+tonumber(row.g_rate)*filteredRate+tonumber(row.g_torque)*filteredTorque
  end

  -- Phase 6: reconstruct once, from the SAME anchor/dt as phase 1. Shadow mode
  -- leaves the original LLC path unchanged. Active mode executes the selected
  -- bounded infeasible policy. There is no post-QP independent angle/torque clip.
  local sentAngle,sentTorque = angle,torque
  if cfg.mode == 'active' then
    if manual then
      report.steering_override = not handbackReachable
        or abs(filteredRate-rate)>1e-7 or report.rf_branch_override==1
      report.torque_override = cfg.torque_correction and abs(filteredTorque-torque)>1e-4
      if status == 0 then
        -- Ownership is derived from ONE joint solution. Dropping a numerically
        -- small correction must not turn that pair into a violating pair.
        -- Restore omitted positive row contributions if the proposed release
        -- has negative margin; keep torque disabled when configured as such.
        local row = diagnostic[0].row
        local gr,gt = tonumber(row.g_rate),tonumber(row.g_torque)
        local selectedRate = report.steering_override and filteredRate or rate
        local selectedTorque = report.torque_override and filteredTorque or torque
        report.release_margin = tonumber(row.b)+gr*selectedRate+gt*selectedTorque
        if report.release_margin < 0 then
          if not report.steering_override and gr*(filteredRate-rate)>0 then
            report.steering_override = true
            report.release_preserved_correction = true
          end
          if cfg.torque_correction and not report.torque_override and gt*(filteredTorque-torque)>0 then
            report.torque_override = true
            report.release_preserved_correction = true
          end
        end
      end
      if report.steering_override then sentAngle = anchor+dt*filteredRate end
      if report.torque_override then sentTorque = filteredTorque end
    else
      sentAngle,sentTorque = anchor+dt*filteredRate,filteredTorque
      report.steering_override = abs(sentAngle-angle)>1e-8
      report.torque_override = abs(sentTorque-torque)>1e-4
    end
    report.rf_intervened = (report.steering_override or report.torque_override) and 1 or 0
  end
  report.reason = cfg.mode == 'shadow' and 'shadow' or report.rf_branch_override==1 and 'alignment_reduce' or 'qp'
  report.emitted_torque_rate = (sentTorque-torqueAnchor)/dt
  report.emitted_rate = manual and not report.steering_override and rate or (sentAngle-anchor)/dt
  if manual then
    local row = diagnostic[0].row
    report.candidate_margin = report.rf_margin
    report.rf_margin = tonumber(row.b)+tonumber(row.g_rate)*report.emitted_rate
      +tonumber(row.g_torque)*sentTorque
  end
  previousAngle = (not manual or report.steering_override) and sentAngle or nil
  previousTorque = (not manual or report.torque_override) and sentTorque or nil
  previousNominalTorque = torque
  previousTime = now
  publish(status)
  return sentAngle,sentTorque,report
end

-- Unsupported direct-input/parking-brake/partial-authority paths keep existing actuation
-- and report explicitly; this avoids silently claiming filtering those commands.
function M.unavailable(reason)
  if lib then
    M.idle()
    report.reason = reason or 'unsupported_command_path'
    publish(reason=='actuator_command_unrepresentable' and -12 or -3)
  end
end
return M
