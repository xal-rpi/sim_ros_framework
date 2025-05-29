-- ./vehicle/controller/xlab/controller_throttle_sweep.lua
local M = {}
local common
local logTag = 'controller_throttle_sweep'

-- local throwaway to avoid global definition warning
local _ = nil

-- Module-local state
local messageCounter = 0
local nowSim = 0.0
local nextUpdateTime = 0.0

-- for logging real update rate
local nowClock = 0
local realLastClock = os.clock()
local realUpdateCount = 0

-- controller internal state, now with two target slots
local controllerState = {
  lastAppliedThrottle = 0,
  lastAppliedBrake = 0,

  prevTarget = { throttle_target = 0, brake_target = 0, time = 0 },
  nextTarget = { throttle_target = 0, brake_target = 0, time = 0 },
}

local calibration = {
}

-- Parse incoming JSON control message
local function parseMessage(msg)
  if not msg or msg == '' then return false end
  if not jsonDecode then
    log('E', logTag, 'JSON decoder not initialized')
    return false
  end

  local ok, data = pcall(jsonDecode, msg)
  if not ok then
    log('E', logTag, 'Failed to parse JSON: ' .. tostring(data))
    return false
  end

  -- must have the three control channels
  if data.throttle_target == nil or data.brake_target == nil then
    log(
      'E',
      logTag,
      'Incomplete control message: throttle_target or brake_target missing'
    )
    return false
  end

  -- decide reachTime: prefer data.time, else data.timestamp, else now
  local reachTime
  if data.time ~= nil then
    reachTime = data.time
    -- if it's not the special "0" and it's already in the past, drop it
    if reachTime ~= 0 and reachTime < nowSim then
      log(
        'W',
        logTag,
        string.format('Expired target (time %.3f < now %.3f), ignoring', reachTime, nowSim)
      )
      return false
    end
  else
    reachTime = data.timestamp or nowSim
  end

  -- shift our two‐slot target buffer
  controllerState.prevTarget = controllerState.nextTarget
  controllerState.nextTarget = {
    throttle_target = data.throttle_target,
    brake_target = data.brake_target,
    time = reachTime,
  }

  -- special immediate‐apply case: zero out interpolation
  if reachTime == 0 then controllerState.prevTarget = controllerState.nextTarget end

  -- perf metrics
  local pm = common.performanceMetrics
  pm.lastCommandTimestamp = nowSim
  pm.commandsReceived = pm.commandsReceived + 1
  messageCounter = messageCounter + 1

  return true
end

-- Build a JSON state message
local function createStateMessage()
  if not jsonEncode then
    log('E', logTag, 'JSON encoder not initialized')
    return '{}'
  end
  local pm = common.performanceMetrics
  pm.lastResponseTimestamp = nowSim

  -- full gtState
  local vr = common.cachedGtReading
  local vs = common.vehicleState
  local cs = controllerState

  local state = {
    simtime = nowSim,
    realtime = os.clock(),
    avg_latency = pm.avgLatency,
    max_latency = pm.maxLatency,
    speed = obj:getGroundSpeed(), -- m/s
    -- Engine data
    engine = {
      rpm = vr.RPM,
      throttle = vr.throttle or 0, -- actual throttle
      engineTorque = vr.engineTorque or vr.flywheelTorque or 0,
      max_torque = vs.maxTorque,
      max_rpm = vs.maxRPM,
    },

    -- Vehicle control inputs
    controls = {
      steering = vr.steering,
      throttle = vr.throttle,
      brake = vr.brake,
      clutch = vr.clutch,
      parkingbrake = vr.pbrake,
    },

    -- Received targets for debugging
    targets = {
      throttle = cs.nextTarget.throttle_target,
      brake = cs.nextTarget.brake_target,
    },
  }
  local ok, js = pcall(jsonEncode, state)
  if not ok then
    log('E', logTag, 'JSON encode error: ' .. tostring(js))
    return '{}'
  end
  return js
end

-- apply computed controls by interpolating between prevTarget→nextTarget
local function applyTargets(dt)
  local cs = controllerState

  -- interpolation factor
  local p = cs.prevTarget
  local n = cs.nextTarget
  local t0, t1 = p.time or nowSim, n.time or nowSim
  local f = 1
  if t1 > t0 then
    f = (nowSim - t0) / (t1 - t0)
    if f < 0 then
      f = 0
    elseif f > 1 then
      f = 1
    end
  end

  -- lerp each channel
  local desiredThrottle = p.throttle_target + (n.throttle_target - p.throttle_target) * f
  local desiredBrake = p.brake_target + (n.brake_target - p.brake_target) * f

  -- Apply throttle
  cs.lastAppliedThrottle = math.min(1, math.max(0, desiredThrottle)) -- Clamp to [0,1]

  -- Apply brake
  cs.lastAppliedBrake = math.min(1, math.max(0, desiredBrake)) -- Clamp to [0,1]

  -- send into BeamNG
  input.event('throttle', cs.lastAppliedThrottle, FILTER_AI)
  electrics.values.throttle = cs.lastAppliedThrottle
  input.event('brake', cs.lastAppliedBrake, FILTER_AI)
  electrics.values.brake = cs.lastAppliedBrake

  -- latency metrics (ring-buffer of size N=100)
  do
    local pm = common.performanceMetrics
    local lat = nowSim - pm.lastCommandTimestamp
    pm.latency:add(lat)
    pm.avgLatency = pm.latency:average()
    pm.maxLatency = pm.latency.max
  end

  -- occasional logging
  do
    if nowClock == realLastClock then -- Log once per second roughly (tied to realLastClock update)
      log(
        'I',
        logTag,
        string.format(
          'Applied: thr=%.2f br=%.2f avgLat=%.1fms SimTime=%.3f',
          cs.lastAppliedThrottle,
          cs.lastAppliedBrake,
          common.performanceMetrics.avgLatency * 1000,
          nowSim
        )
      )
    end
  end
end

-- Extension interface

function M.init(c)
  common = c
  log('I', logTag, 'Throttle Sweep controller initialized')
  nextUpdateTime = common.getSimTime()
end

function M.update(dt)
  if not common.isRunning then return end

  -- limit rate
  nowSim = common.getSimTime()
  if nowSim < nextUpdateTime then return end
  nextUpdateTime = nextUpdateTime + common.controllerRate

  -- track true update rate
  realUpdateCount = realUpdateCount + 1
  nowClock = os.clock()
  local elapsed = nowClock - realLastClock
  if elapsed >= 1 then
    log('I', logTag, string.format('Real update rate: %.1f Hz', realUpdateCount / elapsed))
    realUpdateCount = 0
    realLastClock = nowClock
  end

  -- 1) drain UDP queue, keep only last message
  if common.socketIn then
    local lastMsg, err
    repeat
      local msg
      msg, _, _, err = common.socketIn:receivefrom()
      if msg and #msg > 0 then lastMsg = msg end
    until not msg and (not err or err == 'timeout')

    if err and err ~= 'timeout' then log('E', logTag, 'socketIn error: ' .. tostring(err)) end
    if lastMsg then parseMessage(lastMsg) end
  else
    log('E', logTag, 'socketIn is nil')
  end

  -- 2) refresh gt-reading if due
  common.updateGtReading()

  -- 3) always apply controls (with interpolation) every frame
  applyTargets(dt)

  -- 4) send state message at fixed rate
  if common.socketOut then
    local s, se = createStateMessage(), nil
    _, se = common.socketOut:sendto(s, common.sendIp, common.sendPort)
    if se then log('E', logTag, 'socketOut err=' .. tostring(se)) end
  elseif not common.socketOut then
    log('E', logTag, 'socketOut is nil')
  end

  -- clear cached reading
  common.cachedGtReading = nil
end

function M.stop() log('I', logTag, 'Throttle Sweep controller cleanup') end

function M.setGtStateSensor(id) common.gtStateSensorId = id end

function M.calibrate(params)
  log('I', logTag, 'Calibrating controller')
  for k, v in pairs(params) do
    if calibration[k] ~= nil then
      calibration[k] = v
      log('I', logTag, '    ' .. k .. '=' .. tostring(v))
    else
      log('W', logTag, 'calibration entry ' .. k .. ' not found')
    end
  end
end

function M.reset()
  controllerState = {
    lastAppliedThrottle = 0,
    lastAppliedBrake = 0,
    prevTarget = { throttle_target = 0, brake_target = 0, time = 0 },
    nextTarget = { throttle_target = 0, brake_target = 0, time = 0 },
  }

  messageCounter = 0
  common.lastGtReadingTime = 0
  common.cachedGtReading = nil
  log('I', logTag, 'Controller state reset')
end

function M.getStatus()
  return {
    isRunning = common.isRunning,
    performanceMetrics = common.performanceMetrics,
    calibration = calibration,
    controllerState = controllerState,
  }
end

return M
