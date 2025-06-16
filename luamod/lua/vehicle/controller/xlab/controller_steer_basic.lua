-- controller_path_speed.lua
local M = {}
local common
local logTag = 'controller_path_speed'
local updateAccum = 0
local nowSim, nowClock = 0, 0
local realLastClock = os.clock()
local realUpdateCount = 0

local controllerState = {
  targetList = {},
  currentVehicleS = 0.0,
  currentTargetIdx = 1,
  lastAppliedSteering = 0,
  lastAppliedThrottle = 0,
  lastAppliedBrake = 0,
}

local calibration = {
  maxSteeringAngle = 0.69,
  Kp_speed = 0.2,
  defaultDesiredSpeed = 10.0,
}

local desiredSteer = 0
local _origInputEvent = input.event
input.event = function(itype, ivalue, filter, angle, lockType, osClockHP, source)
  if itype ~= 'steering' and itype ~= 'throttle' and itype ~= 'brake' then
    log('D', 'InputDebug', ('[%s]=%0.3f'):format(itype, ivalue))
  end
  return _origInputEvent(itype, ivalue, filter, angle, lockType, osClockHP, source)
end

local function parseMessage(msg)
  if not msg or msg == '' then return false end
  if not jsonDecode then
    log('E', logTag, 'JSON decoder not initialized')
    return false
  end
  local ok, data = pcall(jsonDecode, msg)
  if not ok then
    log('E', logTag, 'JSON parse error: ' .. tostring(data))
    return false
  end
  if not data.targets or type(data.targets) ~= 'table' then
    log('E', logTag, "Missing 'targets' array or wrong type")
    return false
  end
  if #data.targets > 0 then
    local f = data.targets[1]
    if
      f.s == nil
      or f.x == nil
      or f.y == nil
      or f.z == nil
      or f.road_wheel_angle == nil
      or f.desired_speed == nil
    then
      log('E', logTag, 'Each target needs x,y,z,s,road_wheel_angle,desired_speed')
      return false
    end
  else
    log('W', logTag, "Empty 'targets' -> zero/default controls")
  end
  controllerState.targetList = data.targets
  controllerState.currentVehicleS = 0.0
  controllerState.currentTargetIdx = 1
  local pm = common.performanceMetrics
  pm.lastCommandTimestamp = nowSim
  pm.commandsReceived = pm.commandsReceived + 1
  return true
end

local function createStateMessage()
  if not jsonEncode then
    log('E', logTag, 'JSON encoder not initialized')
    return '{}'
  end
  local pm = common.performanceMetrics
  pm.lastResponseTimestamp = nowSim
  local vr = common.cachedGtReading
  local state = {
    simtime = nowSim,
    realtime = os.clock(),
    avg_latency = pm.avgLatency,
    max_latency = pm.maxLatency,
    position = {
      x = (vr.pos and vr.pos[1]) or 0,
      y = (vr.pos and vr.pos[2]) or 0,
      z = (vr.pos and vr.pos[3]) or 0,
    },
    direction = {
      x = (vr.dirX and vr.dirX[1]) or 0,
      y = (vr.dirX and vr.dirX[2]) or 0,
      z = (vr.dirX and vr.dirX[3]) or 0,
    },
    velocity = {
      x = (vr.vel and vr.vel[1]) or 0,
      y = (vr.vel and vr.vel[2]) or 0,
      z = (vr.vel and vr.vel[3]) or 0,
    },
  }
  local ok, js = pcall(jsonEncode, state)
  if not ok then
    log('E', logTag, 'JSON encode error: ' .. tostring(js))
    return '{}'
  end
  return js
end

local function applyTargets(dt)
  local cs = controllerState
  local vr = common.cachedGtReading
  if not vr or not vr.vel or #cs.targetList < 1 then
    desiredSteer = 0
    cs.lastAppliedSteering = 0
    input.event('throttle', 0, FILTER_AI)
    electrics.values.throttle = 0
    cs.lastAppliedThrottle = 0
    input.event('brake', 0, FILTER_AI)
    electrics.values.brake = 0
    cs.lastAppliedBrake = 0
    return
  end

  -- 1) project vehicle position onto the piecewise-linear target curve
  local cx, cy, cz = vr.pos[1], vr.pos[2], vr.pos[3]
  local bestD2, bestIdx, bestT = 1e99, 1, 0
  for i = 1, #cs.targetList - 1 do
    local A, B = cs.targetList[i], cs.targetList[i + 1]
    local vx, vy, vz = B.x - A.x, B.y - A.y, B.z - A.z
    local wx, wy, wz = cx - A.x, cy - A.y, cz - A.z
    local L2 = vx * vx + vy * vy + vz * vz + 1e-12
    local t = (wx * vx + wy * vy + wz * vz) / L2
    t = math.max(0, math.min(1, t)) -- Simplified clamp
    local px, py, pz = A.x + t * vx, A.y + t * vy, A.z + t * vz
    local dx, dy, dz = cx - px, cy - py, cz - pz
    local d2 = dx * dx + dy * dy + dz * dz
    if d2 < bestD2 then
      bestD2, bestIdx, bestT = d2, i, t
    end
  end

  -- If we have no valid projection, fall back to the first target
  if #cs.targetList == 1 then
    bestIdx = 1
    bestT = 0
    log('W', logTag, 'No valid projection')
  end

  -- 2) Log when we pass a target for debugging
  local TA_log = cs.targetList[bestIdx]
  local TB_log = cs.targetList[math.min(bestIdx + 1, #cs.targetList)]
  local s_proj = TA_log.s + bestT * (TB_log.s - TA_log.s)
  cs.currentVehicleS = s_proj
  while cs.currentTargetIdx < #cs.targetList and s_proj > cs.targetList[cs.currentTargetIdx].s do
    -- log(
    --   'I',
    --   logTag,
    --   string.format(
    --     'Reached target #%d (s=%.2f), next is #%d (s=%.2f)',
    --     cs.currentTargetIdx,
    --     cs.targetList[cs.currentTargetIdx].s,
    --     cs.currentTargetIdx + 1,
    --     cs.targetList[cs.currentTargetIdx + 1].s
    --   )
    -- )
    cs.currentTargetIdx = cs.currentTargetIdx + 1
  end

  -- 3) Interpolate control commands based on the projection result
  local A = cs.targetList[bestIdx]
  local B = cs.targetList[math.min(bestIdx + 1, #cs.targetList)]
  local f = bestT

  -- 4) steering
  desiredSteer = A.road_wheel_angle + (B.road_wheel_angle - A.road_wheel_angle) * f
  cs.lastAppliedSteering = desiredSteer / calibration.maxSteeringAngle
  input.event('steering', cs.lastAppliedSteering, FILTER_AI)

  -- 5) speed P‐controller
  local vx3, vy3, vz3 = vr.vel[1], vr.vel[2], vr.vel[3]
  local vmag = math.sqrt(vx3 ^ 2 + vy3 ^ 2 + vz3 ^ 2)
  local tgt_spd = A.desired_speed + (B.desired_speed - A.desired_speed) * f
  local err = tgt_spd - vmag
  local cmd = calibration.Kp_speed * err
  local thr, brk = 0, 0
  if cmd > 0.01 then thr = math.min(cmd, 1) end
  if cmd < -0.01 then brk = math.min(-cmd, 1) end

  input.event('throttle', thr, FILTER_AI)
  electrics.values.throttle = thr
  cs.lastAppliedThrottle = thr

  input.event('brake', brk, FILTER_AI)
  electrics.values.brake = brk
  cs.lastAppliedBrake = brk

  -- latency metrics (optional)
  do
    local pm = common.performanceMetrics
    local lat = nowSim - pm.lastCommandTimestamp
    pm.latency:add(lat)
    pm.avgLatency = pm.latency:average()
    pm.maxLatency = pm.latency.max
  end
end

function M.init(c)
  common = c
  if hydros then
    hydros.enableVirtualWheel(true, function() return desiredSteer end, obj.sendForceFeedback)
  else
    log('E', logTag, 'Hydros not found')
  end
  log('I', logTag, 'Controller initialized')
end

function M.update(dt)
  if not common.isRunning then return end
  updateAccum = updateAccum + dt
  if updateAccum < common.controllerRate then return end
  updateAccum = updateAccum - common.controllerRate
  nowSim = common.getSimTime()
  nowClock = os.clock()
  realUpdateCount = realUpdateCount + 1
  if nowClock - realLastClock >= 1 then
    log(
      'I',
      logTag,
      string.format('Real rate: %.1f Hz', realUpdateCount / (nowClock - realLastClock))
    )
    realUpdateCount = 0
    realLastClock = nowClock
  end

  if common.socketIn then
    local lastMsg, err
    repeat
      local msg, _, _, e = common.socketIn:receivefrom()
      if msg and #msg > 0 then lastMsg = msg end
      err = e
    until not msg and (not err or err == 'timeout')
    if err and err ~= 'timeout' then log('E', logTag, 'sockIn err: ' .. tostring(err)) end
    if lastMsg then parseMessage(lastMsg) end
  else
    log('E', logTag, 'socketIn is nil')
  end

  common.updateGtReading()
  if not common.isBypassed then applyTargets(common.controllerRate) end

  if common.socketOut then
    local s = createStateMessage()
    local _, se = common.socketOut:sendto(s, common.sendIp, common.sendPort)
    if se then log('E', logTag, 'sockOut err: ' .. tostring(se)) end
  else
    log('E', logTag, 'socketOut is nil')
  end
  common.cachedGtReading = nil
end

function M.stop() log('I', logTag, 'Cleanup') end

function M.setGtStateSensor(id) common.gtStateSensorId = id end

function M.calibrate(params)
  for k, v in pairs(params) do
    if calibration[k] ~= nil then
      calibration[k] = v
    else
      log('W', logTag, 'cal missing: ' .. k)
    end
  end
end

function M.reset()
  controllerState = {
    targetList = {},
    currentVehicleS = 0.0,
    currentTargetIdx = 1,
    lastAppliedSteering = 0,
    lastAppliedThrottle = 0,
    lastAppliedBrake = 0,
  }
  updateAccum = 0
  common.lastGtReadingTime = 0
  common.cachedGtReading = nil
  log('I', logTag, 'State reset')
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
