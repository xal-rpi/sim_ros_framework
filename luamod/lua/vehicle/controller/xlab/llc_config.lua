-- Cold-path LLC calibration. resolve(params, current, mode) returns a complete
-- candidate without mutating current settings or command history. Invalid input
-- raises; the LLC reports rejection and commits nothing. No physics-tick calls.
local SteeringMap = require('vehicle.controller.xlab.steering_map')
local abs, huge, floor = math.abs, math.huge, math.floor
local M = {}
local GAIN_KEYS = {
  'pi_enable', 'kp', 'ki', 'i_min', 'i_max', 't_min', 't_max', 'u_smooth', 'du_max',
  'steer_pi_enable', 'steer_kp', 'steer_ki', 'steer_i_min', 'steer_i_max', 'command_timeout',
}
local ALLOWED = {gains=true, steering_map=true, brake_coordination=true,
  autonomous_rate_limits=true, execution_plan=true, trajectory_preview=true,
  trajectory_pick=true, control_mode=true, verbose=true, torque_map=true}
local GAIN = {}
for _, key in ipairs(GAIN_KEYS) do ALLOWED[key], GAIN[key] = true, true end
local AUTHORITY = {full=true, steering_only=true, torque_only=true}
local PICK = {next=true, lerp=true, closest=true, preview=true}

local function finite(value)
  return type(value) == 'number' and value == value and abs(value) < huge
end
local function copy(values)
  local result = {}
  for key, value in pairs(values) do result[key] = value end
  return result
end
local function keys(values, allowed, label)
  assert(type(values) == 'table', label .. ' must be a table')
  for key in pairs(values) do assert(allowed[key], 'Unknown ' .. label .. '.' .. tostring(key)) end
end

-- params is a partial calibration; current supplies resolved gains/map/plan.
-- mode carries initialized, active, hasFilter and the optional preview trajectory.
-- Returns a complete candidate; all validation precedes the caller's commit.
function M.resolve(params, current, mode)
  assert(type(params) == 'table', 'calibration must be a table')
  assert(params.torque_map_api == nil, 'torque_map_api was removed; configure only torque_map')
  keys(params, ALLOWED, 'calibration')
  local next = copy(current)
  next.gains = copy(current.gains)
  if params.gains ~= nil then
    keys(params.gains, GAIN, 'gains')
    for key, value in pairs(params.gains) do
      assert(finite(value), 'Nonfinite gain: ' .. key)
      next.gains[key] = value
    end
  end
  -- Flat startup gains and nested partial gains have the same validation.
  for _, key in ipairs(GAIN_KEYS) do
    if params[key] ~= nil then
      assert(finite(params[key]), 'Nonfinite gain: ' .. key)
      next.gains[key] = params[key]
    end
  end
  for _, pair in ipairs({{'i_min','i_max'}, {'t_min','t_max'}, {'steer_i_min','steer_i_max'}}) do
    assert(next.gains[pair[1]] <= next.gains[pair[2]], 'Inverted gain bounds: ' .. pair[1])
  end
  assert(next.gains.command_timeout > 0, 'command_timeout must be positive')
  if params.control_mode ~= nil then
    assert(AUTHORITY[params.control_mode], 'Invalid control_mode')
    next.authority = params.control_mode
  end
  if params.verbose ~= nil then
    assert(type(params.verbose) == 'boolean', 'verbose must be boolean')
    next.verbose = params.verbose
  end
  if params.torque_map ~= nil then
    assert(type(params.torque_map) == 'string' and params.torque_map ~= '', 'torque_map must be a name')
    assert(not mode.initialized or params.torque_map == current.torqueMap,
      'Recreate controller to change torque_map after initialization')
    next.torqueMap = params.torque_map
  end
  if params.steering_map ~= nil then
    assert(not current.steering or not (mode.active or mode.hasFilter),
      'Clear control and recreate controller to change steering map with roll_filter')
    next.steering = SteeringMap.new(params.steering_map)
  end
  if params.autonomous_rate_limits ~= nil then
    local limits = params.autonomous_rate_limits
    keys(limits, {steering_rate_max=true, torque_rate_max=true}, 'autonomous_rate_limits')
    assert(finite(limits.steering_rate_max) and limits.steering_rate_max > 0,
      'autonomous_rate_limits requires positive steering_rate_max [rad/s]')
    for key, value in pairs(limits) do assert(finite(value) and value > 0, 'Invalid rate: ' .. key) end
    next.rates = copy(limits)
  end
  if params.brake_coordination ~= nil then
    local brake = params.brake_coordination
    if brake == false then next.brake = nil
    else
      keys(brake, {overlap_torque_nm=true, speed_disable_m_s=true,
        speed_enable_m_s=true, pedal_deadband=true}, 'brake_coordination')
      assert(finite(brake.overlap_torque_nm) and brake.overlap_torque_nm > 0,
        'brake_coordination requires positive overlap_torque_nm')
      if brake.pedal_deadband ~= nil then
        assert(finite(brake.pedal_deadband) and brake.pedal_deadband >= 0 and brake.pedal_deadband < 1,
          'pedal_deadband must be in [0,1)')
      end
      if brake.speed_disable_m_s ~= nil or brake.speed_enable_m_s ~= nil then
        assert(finite(brake.speed_disable_m_s) and brake.speed_disable_m_s >= 0
          and finite(brake.speed_enable_m_s) and brake.speed_disable_m_s < brake.speed_enable_m_s,
          'Brake guard requires 0 <= speed_disable_m_s < speed_enable_m_s')
      end
      next.brake = copy(brake)
    end
  end
  if params.execution_plan ~= nil then
    keys(params.execution_plan, {frame_id=true, epoch_id=true, max_source_age_s=true}, 'execution_plan')
    assert(not mode.active, 'Clear command before changing execution_plan')
    next.plan = copy(current.plan)
    for key, value in pairs(params.execution_plan) do
      assert(finite(value), 'Invalid execution_plan value')
      if key == 'max_source_age_s' then assert(value > 0, 'Plan source age must be positive')
      else assert(value >= 0 and value < 4294967296 and value == floor(value), 'Invalid plan frame/epoch') end
      next.plan[key] = value
    end
  end
  if params.trajectory_preview ~= nil then
    local preview = params.trajectory_preview
    assert(type(preview) == 'table' and ((preview.time_s ~= nil) ~= (preview.distance_m ~= nil)),
      'trajectory_preview requires exactly one of time_s or distance_m')
    local kind = preview.time_s ~= nil and 'time_s' or 'distance_m'
    keys(preview, {[kind]=true}, 'trajectory_preview')
    local result = {kind=kind, steering=0, torque=0, wheel_speed=0}
    keys(preview[kind], {steering=true, torque=true, wheel_speed=true}, 'preview channel')
    for key, value in pairs(preview[kind]) do
      assert(finite(value) and value >= 0, 'Preview values must be finite and nonnegative')
      result[key] = value
    end
    next.preview = result
  end
  if params.trajectory_pick ~= nil then
    assert(PICK[params.trajectory_pick], 'Invalid trajectory_pick')
    next.pick = params.trajectory_pick
  end
  if next.pick == 'preview' and mode.trajectory then
    assert(next.preview.kind ~= 'time_s' or mode.trajectory.t, 'Time preview requires trajectory.t')
    assert(next.preview.kind ~= 'distance_m' or mode.trajectory.z, 'Distance preview requires trajectory.z')
  end
  return next
end
return M
