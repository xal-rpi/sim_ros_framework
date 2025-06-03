local M = {}
local common
local logTag = 'controller_pid_tuner'

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

-- For steering input
local desiredSteer = 0

-- controller internal state, now with two target slots
local controllerState = {
  torqueErrorIntegral = 0,
  torqueErrorPrev = 0,
  brakeError = 0,
  brakeErrorIntegral = 0,
  brakeErrorPrev = 0,
  lastAppliedThrottle = 0,
  lastAppliedBrake = 0,
  lastAppliedSteering = 0,

  prevTarget = { wheelTorque = 0, roadWheelAngle = 0, brakeTorque = 0, time = 0 },
  nextTarget = { wheelTorque = 0, roadWheelAngle = 0, brakeTorque = 0, time = 0 },
}

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
}

-- Enhanced message parsing to handle flat PID parameters
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
  if not data.wheel_torque or not data.road_wheel_angle or not data.brake_torque then
    log('E', logTag, 'Incomplete control message')
    return false
  end

  -- Check for PID parameter updates directly in the flat data
  local pid_param_names = {
    'throttleP',
    'throttleI',
    'throttleD',
    'brakeP',
    'brakeI',
    'brakeD',
  }

  local updated = false
  for _, param_name in ipairs(pid_param_names) do
    local param_value = data[param_name]
    if
      param_value ~= nil
      and calibration[param_name] ~= nil
      and calibration[param_name] ~= param_value
    then
      local old_value = calibration[param_name]
      calibration[param_name] = param_value
      log(
        'I',
        logTag,
        string.format('Updated %s: %.3f -> %.3f', param_name, old_value, param_value)
      )
      updated = true
    end
  end

  if updated then
    log('I', logTag, 'PID parameters updated from HLC')
    -- Reset integral terms when parameters change to avoid windup
    controllerState.torqueErrorIntegral = 0
    controllerState.brakeErrorIntegral = 0
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
    wheelTorque = data.wheel_torque,
    roadWheelAngle = data.road_wheel_angle,
    brakeTorque = data.brake_torque,
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

-- Enhanced state message to include current PID parameters
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

    -- Current PID parameters (NEW)
    current_pid_params = calibration,

    -- Controller state for debugging (NEW)
    controller_debug = {
      torque_error_integral = controllerState.torqueErrorIntegral,
      torque_error_prev = controllerState.torqueErrorPrev,
      last_applied_throttle = controllerState.lastAppliedThrottle,
      message_counter = messageCounter,
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
  local vs = common.vehicleState
  local vr = common.cachedGtReading
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
  local desiredWheelT = p.wheelTorque + (n.wheelTorque - p.wheelTorque) * f

  local desiredSteerAng = p.roadWheelAngle + (n.roadWheelAngle - p.roadWheelAngle) * f
  desiredSteer = desiredSteerAng / calibration.maxSteeringAngle

  local desiredBrakeT = p.brakeTorque + (n.brakeTorque - p.brakeTorque) * f

  ------------------------------------------------------------------------
  -- steering
  ------------------------------------------------------------------------
  cs.lastAppliedSteering = desiredSteer

  ------------------------------------------------------------------------
  -- throttle (Enhanced PID controller)
  ------------------------------------------------------------------------
  local thr = 0
  local desiredEngineT = 0
  local actualEngineT = 0
  local errorN = 0
  local dErrorN = 0
  local ff = 0
  local totalRatio = 0
  if desiredWheelT > 0 then
    local pt = powertrain.getDevice
    local gearbox = pt('gearbox')
    totalRatio = gearbox.children[1].cumulativeGearRatio * vr.gearRatio

    -- raw engine torque demand
    desiredEngineT = desiredWheelT / (totalRatio + 1e-6)
    actualEngineT = vr.flywheelTorque

    -- --–––  STATIC GAIN + OFFSET CORRECTION  ––––--
    -- Computed from a linear regression between actual wheel torque and
    -- theorical wheel torque (engine torque * totalRatio)
    local K_stat = 1.10789884832988
    local B_fric = 398.247968291034

    local wheelTCorr = K_stat * desiredWheelT + B_fric * (desiredWheelT > 0 and 1 or -1)
    desiredEngineT = wheelTCorr / (totalRatio + 1e-6)

    -- 1) feed-forward
    ff = common.torqueLookup.calculateThrottleFromTorque(desiredEngineT, vr.RPM)
    ff = math.min(1, math.max(0, ff))

    -- 2) normalized error
    errorN = (desiredEngineT - vr.flywheelTorque) / vs.maxTorque

    -- 3) integral with current parameters
    cs.torqueErrorIntegral = cs.torqueErrorIntegral + errorN * dt

    -- 4) derivative
    dErrorN = (errorN - cs.torqueErrorPrev) / dt
    cs.torqueErrorPrev = errorN

    -- 5) PID with tuned parameters
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

  ------------------------------------------------------------------------
  -- brake (Enhanced PID controller)
  ------------------------------------------------------------------------
  local estMax = vs.mass * 10
  local br = math.min(math.max(desiredBrakeT / estMax, 0), 1)
  cs.lastAppliedBrake = br

  -- send into BeamNG
  input.event('throttle', thr, FILTER_AI)
  electrics.values.throttle = thr
  input.event('brake', br, FILTER_AI)
  electrics.values.brake = br

  -- latency metrics (ring-buffer of size N=100)
  do
    local pm = common.performanceMetrics
    local lat = nowSim - pm.lastCommandTimestamp
    pm.latency:add(lat)
    pm.avgLatency = pm.latency:average()
    pm.maxLatency = pm.latency.max
  end

  -- Enhanced logging for PID tuning
  do
    if nowClock == realLastClock then
      local simTimeApplied = nowSim or 0
      log(
        'I',
        logTag,
        string.format(
          'PID Tuner - Applied: thr=%.2f str=%.2f br=%.2f avgLat=%.1fms\nSimTime=%.3f',
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
          'PID: P=%.3f I=%.3f D=%.3f | ratio=%.2f |'
            .. ' TWgt=%.1f  Te_des=%.1f  Te_act=%.1f  TW_act=%.1f |'
            .. ' err=%.2f  dErr=%.2f  Int=%.2f  ff=%.2f',
          calibration.throttleP,
          calibration.throttleI,
          calibration.throttleD,
          totalRatio,
          desiredWheelT,
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

-- Extension interface (same as original but with enhanced functionality)

function M.init(c)
  common = c
  if hydros then
    hydros.enableVirtualWheel(true, function() return desiredSteer end, obj.sendForceFeedback)
  else
    log('E', logTag, 'Hydros not found')
  end
  log('I', logTag, 'PID Tuner controller initialized')
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

function M.stop() log('I', logTag, 'PID Tuner controller cleanup') end

function M.setGtStateSensor(id) common.gtStateSensorId = id end

function M.calibrate(params)
  log('I', logTag, 'Calibrating PID tuner controller')
  for k, v in pairs(params) do
    if calibration[k] ~= nil then
      local old_value = calibration[k]
      calibration[k] = v
      log('I', logTag, '    ' .. k .. ': ' .. tostring(old_value) .. ' -> ' .. tostring(v))
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

    prevTarget = { wheelTorque = 0, roadWheelAngle = 0, brakeTorque = 0, time = 0 },
    nextTarget = { wheelTorque = 0, roadWheelAngle = 0, brakeTorque = 0, time = 0 },
  }

  messageCounter = 0
  common.lastGtReadingTime = 0
  common.cachedGtReading = nil
  log('I', logTag, 'PID Tuner controller state reset')
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
