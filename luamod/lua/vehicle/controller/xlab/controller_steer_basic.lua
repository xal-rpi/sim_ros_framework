-- controller_steer_basic.lua
local M = {}
local common
local logTag = 'controller_steer_basic'
local updateAccum = 0
local nowSim, nowClock = 0, 0
local realLastClock = os.clock()
local realUpdateCount = 0

local controllerState = {
  targetList = {},
  -- s-coordinate relative to the start of the current targetList
  relativeS = 0.0,
  currentTargetIdx = 1,
  lastAppliedSteering = 0,
  lastAppliedThrottle = 0,
  lastAppliedBrake = 0,
  -- Flag to trigger a one-time projection for a new path
  projectionNeeded = true,
}

local calibration = {
  maxSteeringAngle = 0.69,
  Kp_speed = 0.2,
  defaultDesiredSpeed = 10.0,
  wheelbase = 2.8,
}

local desiredSteer = 0

-- Intercept input for debug (no changes)
local _origInputEvent = input.event
input.event = function(itype, ivalue, filter, angle, lockType, osClockHP, source)
  if itype ~= 'steering' and itype ~= 'throttle' and itype ~= 'brake' then
    log('D', 'InputDebug', ('[%s]=%0.3f'):format(itype, ivalue))
  end
  return _origInputEvent(itype, ivalue, filter, angle, lockType, osClockHP, source)
end

-- Parse incoming targets and reset the relative state
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
    log('E', logTag, "Missing 'targets' array or wrong type")
    return false
  end
  if #data.targets > 0 then
    for i, t in ipairs(data.targets) do
      if
        t.s == nil
        or t.x == nil
        or t.y == nil
        or t.z == nil
        or t.road_wheel_angle == nil
        or t.desired_speed == nil
        or t.tx == nil
        or t.ty == nil
        or t.tz == nil
      then
        log('E', logTag, ('Target[%d] missing fields'):format(i))
        return false
      end
    end
  else
    log('W', logTag, "Empty 'targets' -> zero/default controls")
  end

  -- New path received: reset state for this relative path
  controllerState.targetList = data.targets
  controllerState.relativeS = 0.0
  controllerState.currentTargetIdx = 1
  controllerState.projectionNeeded = true -- Force re-projection

  local pm = common.performanceMetrics
  pm.lastCommandTimestamp = nowSim
  pm.commandsReceived = pm.commandsReceived + 1
  return true
end

-- Build state JSON for telemetry (no changes)
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

-- Apply steering & speed based on targets
local function applyTargets(dt)
  local cs = controllerState
  local vr = common.cachedGtReading

  if not vr or not vr.vel or not vr.pos or not vr.dirX or #cs.targetList < 1 then
    cs.projectionNeeded = true
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

  -- This is the reference point the controller uses. The executor must match.
  local frontAxlePos = {
    vr.pos[1] + calibration.wheelbase * vr.dirX[1],
    vr.pos[2] + calibration.wheelbase * vr.dirX[2],
    vr.pos[3] + calibration.wheelbase * vr.dirX[3],
  }

  local T1, T2, f

  if cs.projectionNeeded then
    -- Project the FRONT AXLE onto the new path to find our initial relativeS
    local cx, cy, cz = frontAxlePos[1], frontAxlePos[2], frontAxlePos[3]
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
    cs.currentTargetIdx = bestIdx
    T1 = cs.targetList[cs.currentTargetIdx]
    T2 = cs.targetList[math.min(cs.currentTargetIdx + 1, #cs.targetList)]
    cs.relativeS = T1.s + bestT * (T2.s - T1.s)
    f = bestT
    cs.projectionNeeded = false
  else
    -- Progress along the current path using Frenet frame
    T1 = cs.targetList[cs.currentTargetIdx]
    T2 = cs.targetList[math.min(cs.currentTargetIdx + 1, #cs.targetList)]

    -- Recompute interpolation factor f using relative s values
    local delta_s = T2.s - T1.s
    f = 0
    if delta_s > 1e-6 then
      f = (cs.relativeS - T1.s) / delta_s
      f = math.max(0, math.min(1, f)) -- Clamp f
    end

    -- *** FIX 3: INTERPOLATE TANGENT FOR MORE ACCURATE ADVANCEMENT ***
    local t1_norm = math.sqrt(T1.tx ^ 2 + T1.ty ^ 2 + T1.tz ^ 2)
    local t2_norm = math.sqrt(T2.tx ^ 2 + T2.ty ^ 2 + T2.tz ^ 2)
    local interp_tx = T1.tx + f * (T2.tx - T1.tx)
    local interp_ty = T1.ty + f * (T2.ty - T1.ty)
    local interp_tz = T1.tz + f * (T2.tz - T1.tz)
    local interp_norm = math.sqrt(interp_tx ^ 2 + interp_ty ^ 2 + interp_tz ^ 2)
    if interp_norm > 1e-6 then
      interp_tx = interp_tx / interp_norm
      interp_ty = interp_ty / interp_norm
      interp_tz = interp_tz / interp_norm
    end

    local ds = (vr.vel[1] * interp_tx + vr.vel[2] * interp_ty + vr.vel[3] * interp_tz) * dt
    cs.relativeS = cs.relativeS + ds -- Increment relativeS

    -- Advance segment index if we passed the next target's relative s
    while
      cs.currentTargetIdx < #cs.targetList
      and cs.relativeS > cs.targetList[cs.currentTargetIdx + 1].s
    do
      cs.currentTargetIdx = cs.currentTargetIdx + 1
    end
    T1 = cs.targetList[cs.currentTargetIdx] -- T1 might have updated
    T2 = cs.targetList[math.min(cs.currentTargetIdx + 1, #cs.targetList)]
    -- Recompute interpolation factor f with potentially new T1/T2
    delta_s = T2.s - T1.s
    f = 0
    if delta_s > 1e-6 then
      f = (cs.relativeS - T1.s) / delta_s
      f = math.max(0, math.min(1, f)) -- Clamp f
    end
  end

  -- Steering (now correctly interpolates between planned commands)
  desiredSteer = T1.road_wheel_angle + (T2.road_wheel_angle - T1.road_wheel_angle) * f
  cs.lastAppliedSteering = desiredSteer / calibration.maxSteeringAngle
  input.event('steering', cs.lastAppliedSteering, FILTER_AI)

  -- Speed P-controller (logic is unchanged)
  local vmag = math.sqrt(vr.vel[1] ^ 2 + vr.vel[2] ^ 2 + vr.vel[3] ^ 2)
  local tgt = T1.desired_speed + (T2.desired_speed - T1.desired_speed) * f
  local err = tgt - vmag
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

  -- latency metrics (no changes)
  do
    local pm = common.performanceMetrics
    local lat = nowSim - pm.lastCommandTimestamp
    pm.latency:add(lat)
    pm.avgLatency = pm.latency:average()
    pm.maxLatency = pm.latency.max
  end
end

-- Boilerplate functions (M.init, M.update, etc.)
-- Only M.reset and M.getStatus are modified to use relativeS

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
    log('I', logTag, ('Real rate: %.1f Hz'):format(realUpdateCount / (nowClock - realLastClock)))
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
    relativeS = 0.0,
    currentTargetIdx = 1,
    lastAppliedSteering = 0,
    lastAppliedThrottle = 0,
    lastAppliedBrake = 0,
    projectionNeeded = true,
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
