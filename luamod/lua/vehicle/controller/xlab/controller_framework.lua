-- controller_framework.lua
local M = {}
local common
local logTag = 'controller_framework'

-- timing & rate
local updateAccum = 0
local messageCounter = 0
local nowSim = 0.0

-- real‐time logging
local nowClock = 0.0
local realLastClock = os.clock()
local realUpdateCount = 0

-- controller state
local controllerState = {
  targetList = {},
  currentVehicleS = 0.0,
  currentTargetIdx = 1,
  projectionNeeded = true,
  lastAppliedThrottle = 0,
  lastAppliedBrake = 0,
  lastAppliedSteering = 0,
  torqueErrorIntegral = 0,
  torqueErrorPrev = 0,
}

-- calibration parameters
local calibration = {
  maxSteeringAngle = 0.69,
  wheelbase = 2.5,
  Kp_speed = 0.2,
  throttleP = 1.0,
  throttleI = 0.2,
  throttleD = 0.2,
  brakeP = 1.0,
  brakeI = 0.2,
  brakeD = 0.0,
}

-- parse incoming JSON message
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
  if type(data.targets) ~= 'table' then
    log('E', logTag, "Missing 'targets' array")
    return false
  end
  if #data.targets == 0 then
    log('W', logTag, 'Empty targets list')
    controllerState.targetList = {}
    controllerState.currentVehicleS = 0.0
    controllerState.currentTargetIdx = 1
    controllerState.projectionNeeded = true
    return true
  end
  local t0 = data.targets[1]
  if
    t0.s == nil
    or t0.x == nil
    or t0.y == nil
    or t0.z == nil
    or t0.tx == nil
    or t0.ty == nil
    or t0.tz == nil
    or t0.road_wheel_angle == nil
  then
    log('E', logTag, 'First target missing fields')
    return false
  end
  controllerState.targetList = data.targets
  controllerState.currentVehicleS = 0.0
  controllerState.currentTargetIdx = 1
  controllerState.projectionNeeded = true
  local pm = common.performanceMetrics
  pm.lastCommandTimestamp = nowSim
  pm.commandsReceived = pm.commandsReceived + 1
  messageCounter = messageCounter + 1
  log('I', logTag, string.format('Received %d targets', #data.targets))
  return true
end

-- build minimal state message
local function createStateMessage()
  if not jsonEncode then
    log('E', logTag, 'JSON encoder not initialized')
    return '{}'
  end
  local pm = common.performanceMetrics
  pm.lastResponseTimestamp = nowSim
  local state = { simtime = nowSim }
  local ok, js = pcall(jsonEncode, state)
  if not ok then
    log('E', logTag, 'JSON encode error: ' .. tostring(js))
    return '{}'
  end
  return js
end

-- apply the multi‐target commands
local function applyTargets(dt)
  local cs = controllerState
  local vr = common.cachedGtReading
  local vs = common.vehicleState

  -- require valid ground truth & targets
  if not vr or not vr.vel or not vr.pos or not vr.dirX or #cs.targetList < 1 then
    cs.projectionNeeded = true
    cs.currentVehicleS = 0.0
    input.event('steering', 0, FILTER_AI)
    input.event('throttle', 0, FILTER_AI)
    input.event('brake', 0, FILTER_AI)
    return
  end

  -- compute speed
  local vx, vy, vz = vr.vel[1], vr.vel[2], vr.vel[3]
  local speed = math.sqrt(vx * vx + vy * vy + vz * vz)

  -- front axle position
  local fx, fy, fz = vr.dirX[1], vr.dirX[2], vr.dirX[3]
  local cx = vr.pos[1] + calibration.wheelbase * fx
  local cy = vr.pos[2] + calibration.wheelbase * fy
  local cz = vr.pos[3] + calibration.wheelbase * fz

  local T1, T2, f

  if cs.projectionNeeded then
    -- project front axle onto new target polyline
    local bestD2, bestIdx, bestT = 1e99, 1, 0
    for i = 1, #cs.targetList - 1 do
      local A, B = cs.targetList[i], cs.targetList[i + 1]
      local ux, uy, uz = B.x - A.x, B.y - A.y, B.z - A.z
      local wx, wy, wz = cx - A.x, cy - A.y, cz - A.z
      local L2 = ux * ux + uy * uy + uz * uz + 1e-12
      local t = (wx * ux + wy * uy + wz * uz) / L2
      if t < 0 then
        t = 0
      elseif t > 1 then
        t = 1
      end
      local px, py, pz = A.x + t * ux, A.y + t * uy, A.z + t * uz
      local dx, dy, dz = cx - px, cy - py, cz - pz
      local d2 = dx * dx + dy * dy + dz * dz
      if d2 < bestD2 then
        bestD2, bestIdx, bestT = d2, i, t
      end
    end
    cs.currentTargetIdx = bestIdx
    T1 = cs.targetList[bestIdx]
    T2 = cs.targetList[math.min(bestIdx + 1, #cs.targetList)]
    cs.currentVehicleS = T1.s + bestT * (T2.s - T1.s)
    f = bestT
    cs.projectionNeeded = false
  else
    -- advance along path by speed*tangent
    T1 = cs.targetList[cs.currentTargetIdx]
    T2 = cs.targetList[math.min(cs.currentTargetIdx + 1, #cs.targetList)]
    local ds = 0
    do
      local ds_s = T2.s - T1.s
      local frac = (ds_s > 1e-6) and ((cs.currentVehicleS - T1.s) / ds_s) or 0
      frac = math.max(0, math.min(1, frac))
      local tx = T1.tx + frac * (T2.tx - T1.tx)
      local ty = T1.ty + frac * (T2.ty - T1.ty)
      local tz = T1.tz + frac * (T2.tz - T1.tz)
      local nn = math.sqrt(tx * tx + ty * ty + tz * tz)
      if nn > 1e-6 then
        tx, ty, tz = tx / nn, ty / nn, tz / nn
      end
      ds = (vx * tx + vy * ty + vz * tz) * dt
    end
    cs.currentVehicleS = cs.currentVehicleS + ds
    while
      cs.currentTargetIdx < #cs.targetList
      and cs.currentVehicleS > cs.targetList[cs.currentTargetIdx + 1].s
    do
      cs.currentTargetIdx = cs.currentTargetIdx + 1
    end
    T1 = cs.targetList[cs.currentTargetIdx]
    T2 = cs.targetList[math.min(cs.currentTargetIdx + 1, #cs.targetList)]
    do
      local ds_s = T2.s - T1.s
      f = (ds_s > 1e-6) and ((cs.currentVehicleS - T1.s) / ds_s) or 0
      f = math.max(0, math.min(1, f))
    end
  end

  -- interpolate steering
  local phi = T1.road_wheel_angle + f * (T2.road_wheel_angle - T1.road_wheel_angle)
  local steerCmd = phi / calibration.maxSteeringAngle
  cs.lastAppliedSteering = steerCmd
  input.event('steering', steerCmd, FILTER_AI)

  -- interpolate desired_speed if present, use P-control
  if T1.desired_speed then
    local v_tgt = T1.desired_speed + f * (T2.desired_speed - T1.desired_speed)
    local err = v_tgt - speed
    local u = calibration.Kp_speed * err
    local thr = (u > 0.01) and math.min(u, 1) or 0
    cs.lastAppliedThrottle = thr
    input.event('throttle', thr, FILTER_AI)
  end

  -- interpolate brake torque if present
  if T1.brake_torque then
    local b_tgt = T1.brake_torque + f * (T2.brake_torque - T1.brake_torque)
    local br = math.min(math.max(b_tgt / (vs.mass * 10), 0), 1)
    cs.lastAppliedBrake = br
    input.event('brake', br, FILTER_AI)
  end

  -- latency metrics
  do
    local pm = common.performanceMetrics
    local lat = nowSim - pm.lastCommandTimestamp
    pm.latency:add(lat)
    pm.avgLatency = pm.latency:average()
    pm.maxLatency = pm.latency.max
  end

  -- occasional logging (1 Hz)
  nowClock = os.clock()
  realUpdateCount = realUpdateCount + 1
  if nowClock - realLastClock >= 1 then
    log(
      'I',
      logTag,
      string.format(
        'SimTime=%.3f S=%.2f thr=%.2f str=%.2f br=%.2f avgLat=%.1fms',
        nowSim,
        controllerState.currentVehicleS,
        cs.lastAppliedThrottle,
        cs.lastAppliedSteering,
        cs.lastAppliedBrake,
        common.performanceMetrics.avgLatency * 1000
      )
    )
    realLastClock = nowClock
    realUpdateCount = 0
  end
end

function M.init(c)
  common = c
  log('I', logTag, 'Framework controller initialized')
end

function M.update(dt)
  if not common.isRunning then return end
  updateAccum = updateAccum + dt
  if updateAccum < common.controllerRate then return end
  updateAccum = updateAccum - common.controllerRate
  nowSim = common.getSimTime()

  -- receive last UDP message
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

  common.updateGtReading()
  if not common.isBypassed then applyTargets(dt) end

  if common.socketOut then
    local s, se = createStateMessage()
    _, se = common.socketOut:sendto(s, common.sendIp, common.sendPort)
    if se then log('E', logTag, 'socketOut error: ' .. tostring(se)) end
  else
    log('E', logTag, 'socketOut is nil')
  end

  common.cachedGtReading = nil
end

function M.stop() log('I', logTag, 'Framework cleanup') end

function M.setGtStateSensor(id) common.gtStateSensorId = id end

function M.calibrate(params)
  for k, v in pairs(params) do
    if calibration[k] ~= nil then
      calibration[k] = v
    else
      log('W', logTag, 'Unknown cal param ' .. k)
    end
  end
end

function M.reset()
  controllerState = {
    targetList = {},
    currentVehicleS = 0.0,
    currentTargetIdx = 1,
    projectionNeeded = true,
    lastAppliedThrottle = 0,
    lastAppliedBrake = 0,
    lastAppliedSteering = 0,
    torqueErrorIntegral = 0,
    torqueErrorPrev = 0,
  }
  updateAccum = 0
  messageCounter = 0
  common.cachedGtReading = nil
  common.lastGtReadingTime = 0
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
