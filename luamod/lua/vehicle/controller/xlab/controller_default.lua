-- ./vehicle/controller/xlab/controller_default.lua
local M = {}
local common
local logTag = 'controller_default'

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

-- controller internal state
local controllerState = {
  torqueErrorIntegral = 0,
  torqueErrorPrev = 0,
  brakeError = 0,
  brakeErrorIntegral = 0,
  brakeErrorPrev = 0,
  lastAppliedThrottle = 0,
  lastAppliedBrake = 0,
  lastAppliedSteering = 0,

  targetList = {}, -- Will store the array of targets from HLC
  currentVehicleS = 0.0, -- Vehicle's current longitudinal position in the Frenet frame of the current target list
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

  -- Check if targets array is empty
  if #data.targets == 0 then
    log('W', logTag, "Received empty 'targets' array. Controller will hold or use defaults.")
    -- Optionally clear existing targets or handle as per desired behavior for empty updates
    controllerState.targetList = {}
    controllerState.currentVehicleS = 0.0 -- Reset S as the frame of reference changes
    return true -- Or false if an empty list is an error for your use case
  end

  -- Validate first target structure (basic check)
  local firstTarget = data.targets[1]
  if
    not firstTarget.s
    or not firstTarget.d
    or not firstTarget.phi
    or not firstTarget.wheel_torque
    or not firstTarget.brake_torque
    or not firstTarget.road_wheel_angle
  then
    log('E', logTag, 'Target structure is invalid. Missing required Frenet or control fields.')
    return false
  end

  -- New target list received, update controller state
  controllerState.targetList = data.targets
  controllerState.currentVehicleS = 0.0 -- Reset S as this is a new reference frame

  log('I', logTag, string.format('Received %d new targets.', #controllerState.targetList))

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
  local vs = common.vehicleState
  local vr = common.cachedGtReading
  local vs = common.vehicleState
  local vr = common.cachedGtReading
  local cs = controllerState

  -- Calculate vehicle's current speed (magnitude of velocity vector)
  local speed = 0
  if vr and vr.vel then
    speed = math.sqrt((vr.vel[1] or 0) ^ 2 + (vr.vel[2] or 0) ^ 2 + (vr.vel[3] or 0) ^ 2)
  end

  -- Update vehicle's longitudinal position (s) along the path
  cs.currentVehicleS = cs.currentVehicleS + speed * dt

  local desiredWheelT = 0
  local desiredSteerAng = 0
  local desiredBrakeT = 0

  if #cs.targetList == 0 then
    -- No targets, apply zero control or maintain last values
    log('W', logTag, 'Target list is empty. Applying zero controls.')
    desiredWheelT = 0
    desiredSteerAng = 0 -- This will result in desiredSteer = 0
    desiredBrakeT = 0
  else
    local targetA = nil
    local targetB = nil

    -- Find relevant targets A and B
    for i, targetPoint in ipairs(cs.targetList) do
      if targetPoint.s <= cs.currentVehicleS then
        targetA = targetPoint
      else
        targetB = targetPoint
        break -- Found the first target B that is beyond currentVehicleS
      end
    end

    if not targetA and not targetB then
      -- Should not happen if list is not empty, but as a fallback:
      log(
        'W',
        logTag,
        'Could not find suitable targets A or B, list might be malformed. Using first target.'
      )
      targetA = cs.targetList[1]
      targetB = cs.targetList[1]
    elseif not targetA then
      -- currentVehicleS is before the first target's s value
      log('I', logTag, 'Vehicle S is before the first target. Using first target.')
      targetA = cs.targetList[1]
      targetB = cs.targetList[1]
    elseif not targetB then
      -- currentVehicleS is beyond the last target's s value
      log('I', logTag, 'Vehicle S is beyond the last target. Using last target.')
      targetA = cs.targetList[#cs.targetList]
      targetB = cs.targetList[#cs.targetList]
    end

    -- Spatial interpolation factor
    local f_spatial = 0
    if targetA and targetB and targetB.s > targetA.s then
      f_spatial = (cs.currentVehicleS - targetA.s) / (targetB.s - targetA.s)
      f_spatial = math.max(0, math.min(1, f_spatial)) -- Clamp between 0 and 1
    elseif targetA and targetB and targetA.s == targetB.s then
      -- If targetA and targetB are the same (e.g., at the end of the path or only one target)
      -- or if currentVehicleS is exactly on targetA.
      f_spatial = 0 -- Effectively use targetA's values, or 1 to use targetB if preferred.
      -- If currentVehicleS matched targetA.s, then (cs.currentVehicleS - targetA.s) is 0.
    end

    -- Interpolate control values
    -- Ensure targetA and targetB are not nil before accessing fields
    if targetA and targetB then
      desiredWheelT = (targetA.wheel_torque or 0)
        + ((targetB.wheel_torque or 0) - (targetA.wheel_torque or 0)) * f_spatial
      desiredSteerAng = (targetA.road_wheel_angle or 0)
        + ((targetB.road_wheel_angle or 0) - (targetA.road_wheel_angle or 0)) * f_spatial
      desiredBrakeT = (targetA.brake_torque or 0)
        + ((targetB.brake_torque or 0) - (targetA.brake_torque or 0)) * f_spatial
    elseif targetA then -- Fallback if targetB is somehow nil but A is not (e.g. at end of very short list)
      desiredWheelT = targetA.wheel_torque or 0
      desiredSteerAng = targetA.road_wheel_angle or 0
      desiredBrakeT = targetA.brake_torque or 0
    else -- Fallback if both are nil (should be caught by #cs.targetList == 0)
      desiredWheelT = 0
      desiredSteerAng = 0
      desiredBrakeT = 0
    end
  end

  desiredSteer = desiredSteerAng / calibration.maxSteeringAngle

  ------------------------------------------------------------------------
  -- steering
  ------------------------------------------------------------------------
  cs.lastAppliedSteering = desiredSteer

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

  ------------------------------------------------------------------------
  -- brake
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
