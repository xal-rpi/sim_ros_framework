-- ./vehicle/controller/xlab/controller_nn_torque.lua
local M = {}
local common
local logTag = 'controller_nn_torque'

-- Import necessary math functions for performance
local abs = math.abs
local sqrt = math.sqrt
local max = math.max
local min = math.min

local _ = nil -- avoid global warning

-- Controller mode states
local CONTROLLER_MODE = {
  OFF = 0,        -- Controller is off or has timed out (user has manual control)
  ACTIVE = 1,     -- Controller is active and applying commands
}

-- Timing and rate control variables
local updateAccum = 0
local messageCounter = 0
local nowSim = 0
local lastCommandTime = 0
local controllerMode = CONTROLLER_MODE.OFF -- Start in OFF mode

-- Real-time logging variables
-- local nowClock = 0
-- local realLastClock = os.clock()
-- local realUpdateCount = 0
local lastLogRealTime = os.clock()
local lastLogSimTime = 0
local updatesSinceLog = 0

-- Controller state tracking
local controllerState = {
  -- Current target values (single values, not arrays)
  targetTorque = nil,      -- Target wheel torque (Nm), nil means no command
  targetSteering = nil,    -- Target steering input [-1, 1], nil means no command
  targetBrake = nil,       -- Target brake input [0, 1], nil means no command
  
  -- PID state for torque control
  torqueErrorIntegral = 0,
  torqueErrorPrev = 0,
  
  -- Tracking
  lastAppliedThrottle = 0,
  lastAppliedSteering = 0,
  lastAppliedBrake = 0,
}

-- Controller calibration parameters
local calibration = {
  commandTimeout = 0.5,          -- Timeout in seconds before reverting to manual control
  
  -- Torque control PID gains
  torqueKp = 1.0,
  torqueKi = 0.2,
  torqueKd = 0.05,
  
  -- Steering control gains
  steeringKp = 0.0,              -- Proportional gain for steering correction
  steeringKi = 0.0,              -- Integral gain for steering correction
  steeringIntegAlpha = 0.9,      -- Leaky integrator for steering
  steeringToInput = 0.5,         -- Conversion factor: road_wheel_angle = steering_input * steeringToInput
  
  -- Brake control
  brakeDirectMode = true,        -- If true, apply brake directly; if false, use PID
  
  -- Static gain correction for torque (from linear regression)
  torqueStaticGain = 1.10789884832988,
  torqueFrictionOffset = 398.247968291034,
}

-- Reference to the gtState controller
local gtStateController = nil

--[[
    Parse incoming JSON command message.
    Expected format: {
      "torque": <number>,     -- Optional: target wheel torque in Nm
      "steering": <number>,   -- Optional: target steering input [-1, 1]
      "brake": <number>       -- Optional: target brake input [0, 1]
    }
    
    Any field can be null/missing to leave that control to the user.
    
    Parameters:
        msg (string): JSON message string
        
    Returns:
        boolean: True if parsing successful, false otherwise
]]
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

  -- Extract target values (they can be nil/null)
  controllerState.targetTorque = data.torque
  controllerState.targetSteering = data.steering
  controllerState.targetBrake = data.brake

  -- Update command timestamp
  lastCommandTime = nowSim
  controllerMode = CONTROLLER_MODE.ACTIVE

  local pm = common.performanceMetrics
  pm.lastCommandTimestamp = nowSim
  pm.commandsReceived = pm.commandsReceived + 1
  messageCounter = messageCounter + 1
  return true
end

--[[
    Apply torque control using PID + feedforward.
    Converts wheel torque target to throttle input.
    
    Parameters:
        targetWheelTorque (number): Desired wheel torque in Nm
        dt (number): Time step in seconds
]]
local function applyTorqueControl(targetWheelTorque, dt)
  log('I', logTag, string.format('Applying torque control: target=%.3f Nm', targetWheelTorque))
end

--[[
    Apply steering control.
    Optionally uses PID for correction if gains are non-zero.
    
    Parameters:
        targetSteeringInput (number): Desired steering input [-1, 1]
]]
local function applySteeringControl(targetSteeringInput)
    log('I', logTag, string.format('Applying steering control: target=%.3f', targetSteeringInput))
--   local vr = common.cachedGtReading
  
--   -- Simple direct application (optional: add PID correction later)
--   local roadwheelAngle = (vr.wheelFR.angle + vr.wheelFL.angle) / 2
--   local currentSteeringInput = roadwheelAngle / (calibration.steeringToInput + 1e-6)
  
--   -- Optional PID correction (only if gains are non-zero)
--   local steeringCmd = targetSteeringInput
--   if calibration.steeringKp ~= 0 or calibration.steeringKi ~= 0 then
--     local err = targetSteeringInput - currentSteeringInput
--     -- Could add integral/derivative here if needed
--     steeringCmd = targetSteeringInput + calibration.steeringKp * err
--   end
  
--   -- Clamp to [-1, 1]
--   steeringCmd = min(1, max(-1, steeringCmd))
  
--   controllerState.lastAppliedSteering = steeringCmd
--   input.event('steering', steeringCmd, FILTER_AI)
  
--   log('D', logTag, string.format(
--     'Steering control: target=%.3f, current=%.3f, applied=%.3f',
--     targetSteeringInput, currentSteeringInput, steeringCmd
--   ))
end

--[[
    Apply brake control.
    
    Parameters:
        targetBrakeInput (number): Desired brake input [0, 1]
]]
local function applyBrakeControl(targetBrakeInput)
  -- Clamp to [0, 1]
  local brakeCmd = min(1, max(0, targetBrakeInput))
  
  controllerState.lastAppliedBrake = brakeCmd
  input.event('brake', brakeCmd, FILTER_AI)
  electrics.values.brake = brakeCmd
end

--[[
    Main control application function.
    Applies control inputs based on received commands.
    If timeout has occurred, does NOT apply any controls (reverts to manual).
    
    Parameters:
        dt (number): Time step in seconds
]]
local function applyControls(dt)
  local cs = controllerState
  
  -- Check for timeout
  local timeSinceCommand = nowSim - lastCommandTime
  if timeSinceCommand > calibration.commandTimeout then
    -- Timeout occurred - revert to manual control
    if controllerMode == CONTROLLER_MODE.ACTIVE then
      log('I', logTag, string.format(
        'Command timeout (%.2fs since last command) - reverting to manual control',
        timeSinceCommand
      ))
      controllerMode = CONTROLLER_MODE.OFF
      
      -- Reset PID states
      cs.torqueErrorIntegral = 0
      cs.torqueErrorPrev = 0
      cs.targetTorque = nil
      cs.targetSteering = nil
      cs.targetBrake = nil
    end
    
    -- Do NOT apply any controls - user has full manual control
    return
  end
  
  -- Controller is active - apply commanded controls
  
  -- Apply torque control if specified
  if cs.targetTorque ~= nil then
    applyTorqueControl(cs.targetTorque, dt)
  end
  
  -- Apply steering control if specified
  if cs.targetSteering ~= nil then
    applySteeringControl(cs.targetSteering)
  end
  
  -- Apply brake control if specified
  if cs.targetBrake ~= nil then
    applyBrakeControl(cs.targetBrake)
  end
  
  -- Update latency metrics
  local pm = common.performanceMetrics
  local lat = nowSim - pm.lastCommandTimestamp
  pm.latency:add(lat)
  pm.avgLatency = pm.latency:average()
  pm.maxLatency = pm.latency.max
end

-- Module interface ----------------------

--[[
    Initializes the controller with the common interface.
    Sets up the ground truth sensor connection and registers custom fields.
    
    Parameters:
        c (table): Common interface table shared with controller_manager
        
    Returns:
        boolean: True if initialization successful, false otherwise
]]
function M.init(c)
  common = c
  
  -- Get gtState controller
  if common.gtStateManager and common.gtStateSensorId then
    gtStateController = common.gtStateManager.getGtStateController(common.gtStateSensorId)
  else
    log('E', logTag, 'gtState manager or sensor ID not available')
    return false
  end
  
  -- Register custom fields with gtState for data sharing
  if gtStateController and gtStateController.registerCustomField then
    gtStateController.registerCustomField("controller_mode", controllerMode)
    gtStateController.registerCustomField("target_torque", 0)
    gtStateController.registerCustomField("target_steering", 0)
    gtStateController.registerCustomField("target_brake", 0)
    log('I', logTag, 'Registered custom fields with gtState')
  else
    log('W', logTag, 'Could not register custom fields with gtState')
  end
  
  log('I', logTag, 'NN Torque controller initialized')
  return true
end

--[[
    Updates the controller state, processes network messages,
    and applies control inputs to the vehicle.
    
    Parameters:
        dt (number): Time step in seconds
]]
function M.update(dt)
  if not common.isRunning then return end
  
  -- Accumulate time since last update
  updateAccum = updateAccum + dt
  
  -- Check if it's time to update
  if updateAccum < common.controllerRate then return end
  updateAccum = updateAccum - common.controllerRate
  
  -- Update simulation time
  nowSim = common.getSimTime()
  
  -- Process network messages - drain UDP queue, keep only last message
  if common.socketIn then
    local lastMsg
    repeat
      local msg, _, _, err = common.socketIn:receivefrom()
      if msg and #msg > 0 then
        lastMsg = msg
      end
    until not msg and (not err or err == 'timeout')
    
    if lastMsg then
      parseMessage(lastMsg)
    end
  else
    log('E', logTag, 'socketIn is nil')
  end
  
  -- Update ground truth reading
  common.updateGtReading()
  
  -- Apply controls (or revert to manual if timeout)
  if not common.isBypassed then
    applyControls(dt)
  end
  
  -- Update custom fields in gtState
  if gtStateController then
    gtStateController.setCustomField("controller_mode", controllerMode)
    gtStateController.setCustomField("target_torque", controllerState.targetTorque or 0)
    gtStateController.setCustomField("target_steering", controllerState.targetSteering or 0)
    gtStateController.setCustomField("target_brake", controllerState.targetBrake or 0)
  end
  
  -- Periodic logging (1 Hz)
  updatesSinceLog = updatesSinceLog + 1
  if nowSim - lastLogSimTime >= 1.0 then
    local nowRealTime = os.clock()
    local realElapsed = nowRealTime - lastLogRealTime
    local simElapsed = nowSim - lastLogSimTime
    local actualRate = updatesSinceLog / realElapsed
    log('I', logTag, string.format(
      'Update rate: %.1f Hz | Mode: %s | Commands: %d | Sim/Real: %.3f | Latency: avg=%.1fms max=%.1fms',
      actualRate,
      controllerMode == CONTROLLER_MODE.ACTIVE and 'ACTIVE' or 'OFF',
      messageCounter,
      simElapsed / realElapsed,
      common.performanceMetrics.avgLatency * 1000,
      common.performanceMetrics.maxLatency * 1000
    ))
    lastLogRealTime = nowRealTime
    lastLogSimTime = nowSim
    updatesSinceLog = 0
  end
  
  -- Clear cached reading
  common.cachedGtReading = nil
end

--[[
    Cleans up resources when controller is stopped.
]]
function M.stop()
  log('I', logTag, 'NN Torque controller cleanup')
end

--[[
    Sets the ground truth state sensor ID for the controller.
    
    Parameters:
        id (number): Sensor ID to connect to
]]
function M.setGtStateSensor(id)
  common.gtStateSensorId = id
end

--[[
    Calibrates controller parameters from an input configuration.
    
    Parameters:
        params (table): Table containing calibration parameters
]]
function M.calibrate(params)
  log('I', logTag, 'Calibrating NN Torque controller')
  
  if not params then return end
  
  -- Iterate through calibration parameters
  for k, v in pairs(params) do
    if calibration[k] ~= nil then
      calibration[k] = v
      log('I', logTag, '  ' .. k .. ' = ' .. tostring(v))
    else
      log('W', logTag, 'Unknown calibration parameter: ' .. k)
    end
  end
end

--[[
    Resets controller state to initial values.
]]
function M.reset()
  controllerState = {
    targetTorque = nil,
    targetSteering = nil,
    targetBrake = nil,
    torqueErrorIntegral = 0,
    torqueErrorPrev = 0,
    lastAppliedThrottle = 0,
    lastAppliedSteering = 0,
    lastAppliedBrake = 0,
  }
  
  updateAccum = 0
  messageCounter = 0
  lastCommandTime = 0
  lastLogSimTime = 0
  lastLogRealTime = os.clock()
  updatesSinceLog = 0
  controllerMode = CONTROLLER_MODE.OFF
  common.lastGtReadingTime = 0
  common.cachedGtReading = nil
  
  log('I', logTag, 'NN Torque controller state reset')
end

--[[
    Returns the current status of the controller.
    
    Returns:
        table: Contains controller state, metrics and calibration info
]]
function M.getStatus()
  return {
    isRunning = common.isRunning,
    controllerMode = controllerMode,
    timeSinceCommand = nowSim - lastCommandTime,
    performanceMetrics = common.performanceMetrics,
    calibration = calibration,
    controllerState = controllerState,
  }
end

return M
