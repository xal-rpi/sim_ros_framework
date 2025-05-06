-- ./vehicle/controller/xlab/controller_default.lua
local M = {}
local common
local logTag = 'controller_default'

-- Module-local state
local updateAccum = 0
local messageCounter = 0
local nowSim = 0

-- local throwaway to avoid global definition warning
local _ = nil

-- for logging real update rate
local nowClock = 0
local realLastClock = os.clock()
local realUpdateCount = 0

-- controller internal state, now with two target slots
local controllerState = {
  steeringError = 0,
  steeringErrorIntegral = 0,
  steeringErrorPrev = 0,
  brakeError = 0,
  brakeErrorIntegral = 0,
  brakeErrorPrev = 0,
  lastAppliedThrottle = 0,
  lastAppliedBrake = 0,
  lastAppliedSteering = 0,

  prevTarget = { engine_torque = 0, road_wheel_angle = 0, brake_torque = 0, time = 0 },
  nextTarget = { engine_torque = 0, road_wheel_angle = 0, brake_torque = 0, time = 0 },
}

local calibration = {
  steeringP = 1,
  steeringI = 0.0,
  steeringD = 0.0,
  steeringDeadzone = 0.01,
  throttleP = 1.0,
  throttleI = 0.2,
  throttleD = 0.05,
  throttleMinRamp = 1.0,
  throttleMaxRamp = 10.0,
  brakeP = 1.0,
  brakeI = 0.2,
  brakeD = 0.0,
  brakeMinRamp = 1.0,
  brakeMaxRamp = 5.0,
  maxSteeringAngle = 40,
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
  if not data.engine_torque or not data.road_wheel_angle or not data.brake_torque then
    log('E', logTag, 'Incomplete control message')
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
    engine_torque = data.engine_torque,
    road_wheel_angle = data.road_wheel_angle,
    brake_torque = data.brake_torque,
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
      rpm = vr.RPM or vs.rpm,
      torque = vr.engineTorque or vs.engineTorque,
      throttle = vr.throttle or 0,
      max_torque = vs.maxTorque,
      max_rpm = vs.maxRPM,
    },

    -- Gearbox information
    gearbox = {
      gear = vr.gear or 0,
      gear_index = vr.gearIndex or 0,
      gear_ratio = vr.gearRatio or vs.gearRatio or 0,
    },

    -- Wheel data
    wheels = {
      FR = {
        steering_angle = (vr.wheelFR and vr.wheelFR.angle) or 0,
        angular_velocity = (vr.wheelFR and vr.wheelFR.angVel) or 0,
        slip_ratio = (vr.wheelFR and vr.wheelFR.slip) or 0,
      },
      FL = {
        steering_angle = (vr.wheelFL and vr.wheelFL.angle) or 0,
        angular_velocity = (vr.wheelFL and vr.wheelFL.angVel) or 0,
        slip_ratio = (vr.wheelFL and vr.wheelFL.slip) or 0,
      },
      RR = {
        angular_velocity = (vr.wheelRR and vr.wheelRR.angVel) or 0,
        slip_ratio = (vr.wheelRR and vr.wheelRR.slip) or 0,
      },
      RL = {
        angular_velocity = (vr.wheelRL and vr.wheelRL.angVel) or 0,
        slip_ratio = (vr.wheelRL and vr.wheelRL.slip) or 0,
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

-- helpers for throttle/steering/brake
local function getMaxTorqueAtRPM(rpm)
  local vs = common.vehicleState
  local curve = vs.torqueCurve
  if not curve or not rpm then return vs.maxTorque or 500 end
  local key = math.floor(rpm / 50) * 50
  key = math.max(0, math.min(key, vs.maxRPM or key))
  return curve[key] or vs.maxTorque or 500
end

local function calculateThrottleFromTorque(reqT, rpm)
  local maxT = getMaxTorqueAtRPM(rpm)
  local r = reqT / (maxT + 1e-6)
  return math.min(math.max(r, 0), 1)
end

local function updateSteeringPID(target, current, dt)
  -- Compute error in degrees
  local e = target - current

  -- Deadzone: if error is small, treat as zero
  if math.abs(e) < calibration.steeringDeadzone then e = 0 end

  -- Integral term (limit accumulation when large command)
  if math.abs(controllerState.lastAppliedSteering) < 1.0 then
    controllerState.steeringErrorIntegral = controllerState.steeringErrorIntegral + e * dt
  end

  -- Derivative term
  local d = (e - controllerState.steeringErrorPrev) / (dt + 1e-6)
  controllerState.steeringErrorPrev = e

  -- PID output (still in degrees * gain)
  local steer = calibration.steeringP * e
    + calibration.steeringI * controllerState.steeringErrorIntegral
    + calibration.steeringD * d

  -- Clamp to [-1,1]
  steer = math.max(-1, math.min(1, steer))

  controllerState.lastAppliedSteering = steer
  return steer
end

local function applyRateLimit(cur, tgt, minR, maxR, dt)
  local diff = tgt - cur
  local rate = minR + (maxR - minR) * math.abs(diff)
  local d = math.min(math.abs(diff), rate * dt) * (diff < 0 and -1 or 1)
  return cur + d
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
  local desiredTorque = p.engine_torque + (n.engine_torque - p.engine_torque) * f
  local desiredSteerAng = p.road_wheel_angle + (n.road_wheel_angle - p.road_wheel_angle) * f
  local desiredSteer = desiredSteerAng / calibration.maxSteeringAngle
  local desiredBrakeT = p.brake_torque + (n.brake_torque - p.brake_torque) * f

  -- throttle
  local thr = calculateThrottleFromTorque(desiredTorque, common.vehicleState.rpm)
  thr = applyRateLimit(
    controllerState.lastAppliedThrottle,
    thr,
    calibration.throttleMinRamp,
    calibration.throttleMaxRamp,
    dt
  )
  controllerState.lastAppliedThrottle = thr

  -- steering via PID
  local vr = common.cachedGtReading or {}
  local angFR = vr.wheelFR.angle
  local angFL = vr.wheelFL.angle
  local currentWheelAng = (angFR + angFL) / 2
  local maxAng = calibration.maxSteeringAngle
  local current_steer = currentWheelAng / maxAng
  current_steer = math.max(-1, math.min(1, current_steer))
  local steer = updateSteeringPID(desiredSteer, current_steer, dt)
  controllerState.lastAppliedSteering = steer

  -- brake
  local estMax = common.vehicleState.mass * 10
  local br = math.min(math.max(desiredBrakeT / estMax, 0), 1)
  br = applyRateLimit(
    controllerState.lastAppliedBrake,
    br,
    calibration.brakeMinRamp,
    calibration.brakeMaxRamp,
    dt
  )
  controllerState.lastAppliedBrake = br

  -- send into BeamNG
  input.event('throttle', thr, FILTER_AI)
  input.event('steering', steer, FILTER_AI)
  input.event('brake', br, FILTER_AI)

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
          'Applied: thr=%.2f str=%.2f br=%.2f avgLat=%.1fms\nSimTime=%.3f',
          controllerState.lastAppliedThrottle,
          controllerState.lastAppliedSteering,
          controllerState.lastAppliedBrake,
          common.performanceMetrics.avgLatency * 1000,
          simTimeApplied
        )
      )
      log('D', logTag, 'desired ' .. desiredSteer .. ' / current ' .. current_steer)
      log('D', logTag, 'target ang ' .. desiredSteerAng .. ' / current ang ' .. currentWheelAng)
    end
  end
end

-- Extension interface

function M.init(c)
  common = c
  log('I', logTag, 'Default controller initialized')
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
  applyTargets(common.controllerRate) -- dt) fixed rate for better PID

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
    steeringError = 0,
    steeringErrorIntegral = 0,
    steeringErrorPrev = 0,
    brakeError = 0,
    brakeErrorIntegral = 0,
    brakeErrorPrev = 0,
    lastAppliedThrottle = 0,
    lastAppliedBrake = 0,
    lastAppliedSteering = 0,
    prevTarget = { engine_torque = 0, road_wheel_angle = 0, brake_torque = 0, time = 0 },
    nextTarget = { engine_torque = 0, road_wheel_angle = 0, brake_torque = 0, time = 0 },
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
