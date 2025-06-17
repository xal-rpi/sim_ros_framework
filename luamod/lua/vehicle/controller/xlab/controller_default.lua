-- controller_default.lua
local M = {}
local common
local logTag = 'controller_default'

-- Module state
local nowSim, nowClock, realLastClock = 0.0, 0.0, os.clock()
local realUpdateCount = 0

-- Last applied commands
local desiredSteer = 0

-- Controller state
local controllerState = {
  torqueErrorIntegral = 0,
  torqueErrorPrev = 0,
  brakeError = 0,
  brakeErrorIntegral = 0,
  brakeErrorPrev = 0,
  lastAppliedThrottle = 0,
  lastAppliedBrake = 0,
  lastAppliedSteering = 0,

  -- multi‐target scheme
  targetList = {},
  currentTargetIdx = 1,
  currentVehicleS = 0.0,
  projectionNeeded = true,
}

-- Calibration (add wheelbase)
local calibration = {
  throttleP = 1.0,
  throttleI = 0.2,
  throttleD = 0.2,
  brakeP = 1.0,
  brakeI = 0.2,
  brakeD = 0.0,
  brakeMinRamp = 0.0,
  brakeMaxRamp = 50.0,
  maxSteeringAngle = 0.69,
  wheelbase = 2.5, -- [m]; must match your vehicle
}

-- Parse incoming JSON
local function parseMessage(msg)
  if not msg or msg == '' or not jsonDecode then return false end
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
    log('W', logTag, 'Empty targets → clearing')
    controllerState.targetList = {}
    controllerState.currentVehicleS = 0.0
    controllerState.currentTargetIdx = 1
    controllerState.projectionNeeded = true
    return true
  end

  -- Validate the *first* target has what we need
  local t0 = data.targets[1]
  if
    t0.s == nil
    or t0.x == nil
    or t0.y == nil
    or t0.z == nil
    or t0.tx == nil
    or t0.ty == nil
    or t0.tz == nil
    or t0.wheel_torque == nil
    or t0.brake_torque == nil
    or t0.road_wheel_angle == nil
  then
    log('E', logTag, 'Target[1] missing required fields')
    return false
  end

  -- *Accept* and reset for a new multi‐target batch
  controllerState.targetList = data.targets
  controllerState.currentVehicleS = 0.0
  controllerState.currentTargetIdx = 1
  controllerState.projectionNeeded = true

  -- perf
  local pm = common.performanceMetrics
  pm.lastCommandTimestamp = nowSim
  pm.commandsReceived = pm.commandsReceived + 1

  log('I', logTag, string.format('→ Received %d targets', #data.targets))
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
  local state = {
    simtime = nowSim,
    realtime = os.clock(),
    avg_latency = pm.avgLatency,
    max_latency = pm.maxLatency,

    -- Vehicle position and orientation
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

    -- Vehicle velocity
    velocity = {
      x = (vr.vel and vr.vel[1]) or 0,
      y = (vr.vel and vr.vel[2]) or 0,
      z = (vr.vel and vr.vel[3]) or 0,
    },

    -- Calculate speed from velocity components
    speed = vr.vel and math.sqrt(
      (vr.vel[1] or 0) ^ 2 + (vr.vel[2] or 0) ^ 2 + (vr.vel[3] or 0) ^ 2
    ) or 0,

    -- Angular velocity
    angular_velocity = (vr.angVel and vr.angVel[3]) or 0,

    -- Engine data
    engine = {
      rpm = vr.RPM,
      torque = vr.flywheelTorque,
      throttle = vr.throttle or 0,
      max_torque = vs.maxTorque,
      max_rpm = vs.maxRPM,
    },

    -- Gearbox information
    gearbox = {
      gear = vr.gear or 0,
      gear_index = vr.gearIndex or 0,
      gear_ratio = (powertrain.getDevice('gearbox').children[1].cumulativeGearRatio * vr.gearRatio)
        or 0,
    },

    -- Wheel data
    wheels = {
      FR = {
        steering_angle = (vr.wheelFR and vr.wheelFR.angle) or 0,
        angular_velocity = (vr.wheelFR and vr.wheelFR.angVel) or 0,
        torque = vr.wheelFR.propTorque or 0,
      },
      FL = {
        steering_angle = (vr.wheelFL and vr.wheelFL.angle) or 0,
        angular_velocity = (vr.wheelFL and vr.wheelFL.angVel) or 0,
        torque = vr.wheelFL.propTorque or 0,
      },
      RR = {
        angular_velocity = (vr.wheelRR and vr.wheelRR.angVel) or 0,
        torque = vr.wheelRR.propTorque or 0,
      },
      RL = {
        angular_velocity = (vr.wheelRL and vr.wheelRL.angVel) or 0,
        torque = vr.wheelRL.propTorque or 0,
      },
    },

    -- Vehicle control inputs
    controls = {
      steering = vr.steering,
      throttle = vr.throttle,
      brake = vr.brake,
      clutch = vr.clutch,
      parkingbrake = vr.pbrake,
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
  local cs, vr, vs = controllerState, common.cachedGtReading, common.vehicleState
  if not vr or not vr.pos or not vr.dirX or not vr.vel or #cs.targetList < 1 then
    -- emergency zero
    cs.projectionNeeded = true
    cs.currentVehicleS = 0.0
    input.event('steering', 0, FILTER_AI)
    input.event('throttle', 0, FILTER_AI)
    input.event('brake', 0, FILTER_AI)
    return
  end

  -- 1) Front‐axle world‐coords
  local fx, fy, fz = vr.dirX[1], vr.dirX[2], vr.dirX[3]
  local cx = vr.pos[1] + calibration.wheelbase * fx
  local cy = vr.pos[2] + calibration.wheelbase * fy
  local cz = vr.pos[3] + calibration.wheelbase * fz

  local T1, T2, f

  if cs.projectionNeeded then
    -- Project front‐axle onto the new polyline
    local bestD2, bestIdx, bestT = 1e99, 1, 0
    for i = 1, #cs.targetList - 1 do
      local A, B = cs.targetList[i], cs.targetList[i + 1]
      local vx, vy, vz = B.x - A.x, B.y - A.y, B.z - A.z
      local wx, wy, wz = cx - A.x, cy - A.y, cz - A.z
      local L2 = vx * vx + vy * vy + vz * vz + 1e-12
      local t = (wx * vx + wy * vy + wz * vz) / L2
      if t < 0 then
        t = 0
      elseif t > 1 then
        t = 1
      end
      local px, py, pz = A.x + t * vx, A.y + t * vy, A.z + t * vz
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
    -- Advance along the path by velocity·tangent
    T1 = cs.targetList[cs.currentTargetIdx]
    T2 = cs.targetList[math.min(cs.currentTargetIdx + 1, #cs.targetList)]

    -- interpolate tangent
    local delta_s = T2.s - T1.s
    local frac = (delta_s > 1e-6) and ((cs.currentVehicleS - T1.s) / delta_s) or 0
    frac = math.max(0, math.min(1, frac))
    local tx = T1.tx + frac * (T2.tx - T1.tx)
    local ty = T1.ty + frac * (T2.ty - T1.ty)
    local tz = T1.tz + frac * (T2.tz - T1.tz)
    local n = math.sqrt(tx * tx + ty * ty + tz * tz)
    if n > 1e-6 then
      tx, ty, tz = tx / n, ty / n, tz / n
    end

    -- delta_s = v · t · dt
    local ds = (vr.vel[1] * tx + vr.vel[2] * ty + vr.vel[3] * tz) * dt
    cs.currentVehicleS = cs.currentVehicleS + ds

    -- bump segment index if needed
    while
      cs.currentTargetIdx < #cs.targetList
      and cs.currentVehicleS > cs.targetList[cs.currentTargetIdx + 1].s
    do
      cs.currentTargetIdx = cs.currentTargetIdx + 1
    end

    -- recompute interpolation
    T1 = cs.targetList[cs.currentTargetIdx]
    T2 = cs.targetList[math.min(cs.currentTargetIdx + 1, #cs.targetList)]
    delta_s = T2.s - T1.s
    f = (delta_s > 1e-6) and ((cs.currentVehicleS - T1.s) / delta_s) or 0
    f = math.max(0, math.min(1, f))
  end

  -- 2) interpolate controls
  local wT = T1.wheel_torque + f * (T2.wheel_torque - T1.wheel_torque)
  local bT = T1.brake_torque + f * (T2.brake_torque - T1.brake_torque)
  local phi = T1.road_wheel_angle + f * (T2.road_wheel_angle - T1.road_wheel_angle)

  -- 3) steering
  desiredSteer = phi / calibration.maxSteeringAngle
  cs.lastAppliedSteering = desiredSteer
  input.event('steering', desiredSteer, FILTER_AI)

  ------------------------------------------------------------------------
  -- throttle
  ------------------------------------------------------------------------
  local thr = 0
  local desiredEngineT = 0
  local actualEngineT = 0
  local errorN = 0
  local dErrorN = 0
  local ff = 0
  local totalRatio = 0
  if wT > 0 then
    local pt = powertrain.getDevice
    local gearbox = pt('gearbox')
    totalRatio = gearbox.children[1].cumulativeGearRatio * vr.gearRatio

    -- raw engine torque demand
    desiredEngineT = wT / (totalRatio + 1e-6)
    actualEngineT = vr.flywheelTorque

    -- --–––  STATIC GAIN + OFFSET CORRECTION  ––––--
    -- Computed from a linear regression between actual wheel torque and
    -- theorical wheel torque (engine torque * totalRatio)
    local K_stat = 1.10789884832988
    local B_fric = 398.247968291034

    local wheelTCorr = K_stat * wT + B_fric * (wT > 0 and 1 or -1)
    desiredEngineT = wheelTCorr / (totalRatio + 1e-6)

    -- 1) feed-forward
    ff = common.torqueLookup.calculateThrottleFromTorque(desiredEngineT, vr.RPM)
    ff = math.min(1, math.max(0, ff))

    -- 2) normalized error
    errorN = (desiredEngineT - vr.flywheelTorque) / vs.maxTorque

    -- 3) integral
    cs.torqueErrorIntegral = cs.torqueErrorIntegral + errorN * dt

    -- 4) derivative
    dErrorN = (errorN - cs.torqueErrorPrev) / dt
    cs.torqueErrorPrev = errorN

    -- 5) PID on normalized error
    local P_term = calibration.throttleP * errorN
    local I_term = calibration.throttleI * cs.torqueErrorIntegral
    local D_term = calibration.throttleD * dErrorN
    local thrUnlim = ff + P_term + I_term + D_term

    -- 6) anti-windup
    if (thrUnlim > 1 and errorN > 0) or (thrUnlim < 0 and errorN < 0) then
      cs.torqueErrorIntegral = cs.torqueErrorIntegral - errorN * dt
    end

    -- 7) clamp
    thr = math.min(1, math.max(0, thrUnlim))
  end

  cs.lastAppliedThrottle = thr
  input.event('throttle', thr, FILTER_AI)
  electrics.values.throttle = thr

  ------------------------------------------------------------------------
  -- brake
  ------------------------------------------------------------------------
  do
    local estMax = vs.mass * 10
    local br = math.min(math.max(bT / estMax, 0), 1)
    cs.lastAppliedBrake = br
    input.event('brake', br, FILTER_AI)
    electrics.values.brake = br
  end

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
      local simTimeApplied = nowSim or 0
      log(
        'I',
        logTag,
        string.format(
          'Applied: thr=%.2f str=%.2f br=%.2f avgLat=%.1fms\nSimTime=%.3f',
          cs.lastAppliedThrottle,
          cs.lastAppliedSteering,
          cs.lastAppliedBrake,
          common.performanceMetrics.avgLatency * 1000,
          simTimeApplied
        )
      )
      local wheelAct = vr.wheelRL.propTorque + vr.wheelRR.propTorque
      log(
        'I',
        logTag,
        string.format(
          'ratio=%.2f |'
            .. ' TWgt=%.1f  Te_des=%.1f  Te_act=%.1f  TW_act=%.1f |'
            .. ' err=%.2f  dErr=%.2f  I=%.2f  ff=%.2f',
          totalRatio,
          wT,
          desiredEngineT,
          actualEngineT,
          wheelAct,
          errorN,
          dErrorN,
          cs.torqueErrorIntegral,
          ff
        )
      )
    end
  end
end

-- Extension interface

function M.init(c)
  common = c
  if hydros then
    hydros.enableVirtualWheel(true, function() return desiredSteer end, obj.sendForceFeedback)
  else
    log('E', logTag, 'Hydros not found')
  end
  log('I', logTag, 'Default controller initialized')
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
      log('I', logTag, '    ' .. k .. '=' .. tostring(v))
    else
      log('W', logTag, 'calibration entry ' .. k .. ' not found')
    end
  end
end

function M.reset()
  controllerState = {
    torqueErrorIntegral = 0,
    torqueErrorPrev = 0,
    brakeError = 0,
    brakeErrorIntegral = 0,
    brakeErrorPrev = 0,
    lastAppliedThrottle = 0,
    lastAppliedBrake = 0,
    lastAppliedSteering = 0,

    targetList = {},
    currentVehicleS = 0.0,
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
