-- ./vehicle/controller/xlab/controller_nn.lua
local M = {}
local common
local logTag = 'controller_nn'

-- local throwaway to avoid global definition warning
local _ = nil

-- module‐local handle for the network
local nn = nil
local nn_model = nil

-- Module-local state
local updateAccum = 0
local messageCounter = 0
local nowSim = 0

-- for logging real update rate
local nowClock = 0
local realLastClock = os.clock()
local realUpdateCount = 0

local controllerState = {
  prevTarget = { time = 0 },
  nextTarget = { time = 0 },
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

  local reachTime = data.time

  -- shift our two‐slot target buffer
  controllerState.prevTarget = controllerState.nextTarget
  controllerState.nextTarget = {
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

-- apply computed controls by interpolating between prevTarget→nextTarget
local function applyTargets(dt)
  -- interpolation factor
  local p = controllerState.prevTarget
  local n = controllerState.nextTarget
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
  -- local desiredTorque = p.engine_torque + (n.engine_torque - p.engine_torque) * f

  -- send into BeamNG
  -- input.event('throttle', thr, FILTER_AI)

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
    if nowClock == realLastClock then
      local simTimeApplied = common.getSimTime() or 0
      log(
        'I',
        logTag,
        string.format(
          'Target received with time=%.4f, applied at SimTime=%.4f\navgLat=%.1fms\n',
          n.time,
          simTimeApplied,
          common.performanceMetrics.avgLatency * 1000
        )
      )
    end
  end
end

-- Extension interface

function M.init(c)
  common = c
  nn = require('lua/vehicle/controller/xlab/lib/nn')
  nn.init()
  nn_model = nn.loadModel('lua/vehicle/controller/xlab/models/test.json')
  log('I', logTag, 'NN controller initialized')
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

  -- TEST --
  if nn_model and common.cachedGtReading then
    local g = common.cachedGtReading
    local out = nn.run(nn_model, { 12, 5, 0.336 })
    -- map your NN outputs to control targets
    dump(out)
    controllerState.nextTarget.engine_torque = out[1]
    controllerState.nextTarget.brake = out[2]
  end

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

function M.stop()
  log('I', logTag, 'Default controller cleanup')
  nn.freeModel(nn_model)
end

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
    prevTarget = { time = 0 },
    nextTarget = { time = 0 },
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
