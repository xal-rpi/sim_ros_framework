-- Preserve driver input across autonomous execution and optional roll filtering.
--
-- Stock input.event/state remain untouched: keyboard/gamepad smoothing and wheel
-- angle matching run in input.updateGFX exactly once. Capture their processed
-- result, then reapply the last LLC output before downstream actuator consumers.
-- LLC writes use apply(), never input.event(), so they cannot replace human intent.
-- All values use BeamNG normalized inputs: steering [-1,1], pedals [0,1].
local M = {}
local abs, min, max, huge = math.abs, math.min, math.max, math.huge
local channels = {'steering', 'throttle', 'brake'}
local driver, request, output = {}, {}, {}
local config = {}
local steeringCalibration
local originalUpdate, updateHook
local ownsOutput = {}

local function write(name, value)
  input[name] = value
  electrics.values[name .. '_input'] = value
  if name ~= 'steering' then electrics.values[name] = value end
end

-- Manual-reference rates are independent of downstream QP actuator bounds.
-- Omitted startup rates disable the corresponding additional ramp. Partial tunes
-- retain omitted settings. Configuration errors fail before changing any setting.
local RATE_KEYS = {steering_rate_rad_s=true, throttle_rise_per_s=true, throttle_fall_per_s=true}
function M.validate(changes)
  assert(type(changes) == 'table', 'manual_input must be a configuration table')
  for key, value in pairs(changes) do
    assert(RATE_KEYS[key], 'Unknown manual_input setting: '..tostring(key))
    assert(type(value) == 'number' and value > 0 and value < huge,
      'Positive finite manual_input.'..key..' required')
  end
  return true
end

function M.onTune(changes)
  M.validate(changes)
  for key, value in pairs(changes) do config[key] = value end
  return true
end

local function ramp(value, target, rise, fall, dt)
  local delta = target - value
  if delta > 0 and rise then delta = min(delta, rise * dt) end
  if delta < 0 and fall then delta = max(delta, -fall * dt) end
  return value + delta
end

-- Install once per LLC lifecycle. Calibration converts normalized steering into
-- signed roadwheel radians. No stock files or input-event metadata are changed.
function M.init(settings, calibration)
  assert(not originalUpdate, 'Manual input bridge is already installed')
  assert(type(input.updateGFX) == 'function', 'Stock input.updateGFX required')
  assert(type(calibration)=='table' and type(calibration.angle)=='function'
    and type(calibration.input)=='function', 'Measured steering conversion map required')
  steeringCalibration = calibration
  config = {}
  M.onTune(settings or {})
  for _, name in ipairs(channels) do driver[name] = input[name] or 0 end
  driver.parkingbrake = input.parkingbrake or 0
  for _, name in ipairs(channels) do request[name] = driver[name] end
  request.parkingbrake = driver.parkingbrake
  originalUpdate = input.updateGFX
  updateHook = function(dt)
    -- Preserve the stock processor's previous processed values, not LLC output.
    for _, name in ipairs(channels) do input[name] = driver[name] end
    originalUpdate(dt)
    for _, name in ipairs(channels) do driver[name] = input[name] or 0 end
    driver.parkingbrake = input.parkingbrake or 0
    for _, name in ipairs(channels) do
      if ownsOutput[name] and output[name] ~= nil then write(name, output[name]) end
    end
  end
  input.updateGFX = updateHook
end

-- Seed reference ramps on manual activation/handoff from measured actuation,
-- avoiding stale manual history during external control. Inputs are normalized.
function M.beginManual(steering, throttle)
  assert(type(steering) == 'number' and abs(steering) < huge, 'Invalid measured steering')
  assert(type(throttle) == 'number' and throttle >= 0 and throttle <= 1, 'Invalid measured throttle')
  request.steering = min(1, max(-1, steering))
  request.throttle = throttle
end

-- Advance once per manual control tick, BEFORE LLC target/map/PI composition.
-- dt is elapsed vehicle time [s]. No dt means read the latest borrowed snapshot.
-- Steering rate uses roadwheel rad/s; pedal rates use normalized units/s.
-- Brake/parking brake are never ramped. No driver or external input is mutated.
function M.getTargets(dt)
  if dt ~= nil then
    assert(type(dt) == 'number' and dt > 0 and dt < huge, 'Positive finite manual input dt required')
    if config.steering_rate_rad_s then
      local previous,target=steeringCalibration.angle(request.steering),steeringCalibration.angle(driver.steering)
      request.steering=steeringCalibration.input(ramp(previous,target,
        config.steering_rate_rad_s,config.steering_rate_rad_s,dt))
    else
      request.steering=driver.steering
    end
    request.throttle = ramp(request.throttle, driver.throttle,
      config.throttle_rise_per_s, config.throttle_fall_per_s, dt)
    request.brake, request.parkingbrake = driver.brake, driver.parkingbrake
  end
  return request
end

-- Current processed human intent; borrowed until the next graphics update.
-- The manual LLC path reads this directly without allocating a status wrapper.
function M.getDriverTargets() return driver end

-- Diagnostics only: borrowed snapshots/settings, never modify these tables.
function M.getStatus() return {driver=driver, request=request, limits=config, owned=ownsOutput} end

-- Publish this control tick and retain it across subsequent graphics updates.
-- Does not create an input event or mutate input.state / lastInputs.
function M.apply(name, value)
  assert(originalUpdate, 'Manual input bridge is not installed')
  assert(name == 'steering' or name == 'throttle' or name == 'brake', 'Unsupported owned input')
  assert(type(value) == 'number' and value == value and abs(value) <= 1,
    'Nonfinite or out-of-range normalized input')
  ownsOutput[name] = true
  output[name] = value
  write(name, value)
end

-- Bypass/stop returns immediate authority to the latest processed driver request.
function M.release(channel)
  if not originalUpdate then return end
  assert(channel == nil or channel == 'steering' or channel == 'throttle' or channel == 'brake',
    'Unsupported released input')
  for _, name in ipairs(channels) do
    if channel == nil or channel == name then
      ownsOutput[name], output[name] = nil, nil
      write(name, driver[name])
    end
  end
end

function M.stop()
  if not originalUpdate then return end
  M.release()
  assert(input.updateGFX == updateHook, 'Input hook changed while manual bridge was installed')
  input.updateGFX = originalUpdate
  originalUpdate, updateHook = nil, nil
  driver, request, output, ownsOutput = {}, {}, {}, {}
  config, steeringCalibration = {}, nil
end

return M
