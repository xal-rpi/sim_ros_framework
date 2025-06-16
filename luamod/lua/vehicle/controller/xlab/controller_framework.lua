-- ./vehicle/controller/xlab/controller_framework.lua
local M = {}
local common
local logTag = 'controller_framework'

-- local throwaway to avoid global definition warning
local _ = nil

-- Module-local state
local updateAccum = 0
local messageCounter = 0
local nowSim = 0

-- for logging real update rate
local nowClock = 0
local realLastClock = os.clock()
local realUpdateCount = 0

local controllerState = {
  targetList = {}, -- Will store the array of targets from HLC
  currentVehicleS = 0.0, -- Vehicle's current longitudinal position in the Frenet frame of the current target list
}

local calibration = {}

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

  -- Expect 'targets' array
  if not data.targets or type(data.targets) ~= 'table' then
    log('E', logTag, "Incoming message does not have a 'targets' array or it's not a table.")
    return false
  end

  if #data.targets == 0 then
    log('W', logTag, "Received empty 'targets' array.")
    controllerState.targetList = {}
    controllerState.currentVehicleS = 0.0
    -- Allow processing of empty target list if that's a valid state (e.g. to stop)
    return true
  end

  -- Basic validation of the first target's structure (can be more comprehensive)
  local firstTarget = data.targets[1]
  if not firstTarget or type(firstTarget) ~= 'table' or firstTarget.s == nil then
    log('E', logTag, "First target in 'targets' array is invalid or missing 's' coordinate.")
    return false
  end

  -- New target list received, update controller state
  controllerState.targetList = data.targets
  controllerState.currentVehicleS = 0.0 -- Reset S as this is a new reference frame

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

  local state = {
    simtime = nowSim,
  }
  local ok, js = pcall(jsonEncode, state)
  if not ok then
    log('E', logTag, 'JSON encode error: ' .. tostring(js))
    return '{}'
  end
  return js
end

-- applyTargets in the framework is mostly for logging and conceptual processing
local function applyTargets(dt)
  local cs = controllerState
  local vr = common.cachedGtReading -- Assume this is populated by common.updateGtReading()

  -- Calculate vehicle's current speed (magnitude of velocity vector)
  local speed = 0
  if vr and vr.vel then
    -- Using 3D velocity magnitude; consider if 2D (horizontal) speed is more appropriate
    speed = math.sqrt((vr.vel[1] or 0) ^ 2 + (vr.vel[2] or 0) ^ 2 + (vr.vel[3] or 0) ^ 2)
  end

  -- Update vehicle's longitudinal position (s) along the path
  cs.currentVehicleS = cs.currentVehicleS + speed * dt

  -- Latency metrics (can remain as is or be adapted)
  do
    local pm = common.performanceMetrics
    local lat = nowSim - pm.lastCommandTimestamp
    pm.latency:add(lat)
    pm.avgLatency = pm.latency:average()
    pm.maxLatency = pm.latency.max
  end

  -- occasional logging
  do
    if nowClock == realLastClock then -- Log at the specified real clock interval
      if #cs.targetList == 0 then
        log(
          'I',
          logTag,
          string.format(
            'SimTime=%.3f, currentVehicleS=%.2f. No targets in list.',
            nowSim,
            cs.currentVehicleS
          )
        )
      else
        local targetA = nil
        local targetB = nil
        for i, targetPoint in ipairs(cs.targetList) do
          if targetPoint.s <= cs.currentVehicleS then
            targetA = targetPoint
          else
            targetB = targetPoint
            break
          end
        end

        local logMsg =
          string.format('SimTime=%.3f, currentVehicleS=%.2f. ', nowSim, cs.currentVehicleS)
        if targetA then
          logMsg = logMsg .. string.format('TargetA s=%.1f. ', targetA.s)
        else
          logMsg = logMsg .. 'No TargetA found (vehicle before first target). '
        end
        if targetB then
          logMsg = logMsg .. string.format('TargetB s=%.1f. ', targetB.s)
        else
          logMsg = logMsg .. 'No TargetB found (vehicle past last target). '
        end
        log(
          'I',
          logTag,
          logMsg .. string.format('AvgLat=%.1fms', common.performanceMetrics.avgLatency * 1000)
        )
      end
    end
  end
end

-- Extension interface

function M.init(c)
  common = c
  log('I', logTag, 'Framework controller initialized')
end

function M.update(dt)
  if not common.isRunning then return end

  -- limit rate
  updateAccum = updateAccum + dt
  if updateAccum >= common.controllerRate then
    updateAccum = updateAccum - common.controllerRate
  else
    return
  end

  nowSim = common.getSimTime()
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
  if not common.isBypassed then applyTargets(dt) end

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

function M.stop() log('I', logTag, 'Default controller cleanup') end

function M.setGtStateSensor(id) common.gtStateSensorId = id end

function M.calibrate(params)
  log('I', logTag, 'Calibrating controller')
  for k, v in pairs(params) do
    if calibration[k] ~= nil then
      calibration[k] = v
      log('I', logTag, 'cal.' .. k .. '=' .. tostring(v))
    else
      log('W', logTag, 'calibration entry ' .. k .. ' not found')
    end
  end
end

function M.reset()
  controllerState = {
    targetList = {},
    currentVehicleS = 0.0,
  }
  updateAccum = 0
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
