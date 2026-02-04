-- ./vehicle/controller/xlab/controller_nn_torque.lua
local M = {}
local common
local logTag = 'controller_nn_torque'

-- Import necessary math functions for performance
local tanh = math.tanh
local abs = math.abs
local sqrt = math.sqrt
local max = math.max
local min = math.min

-- Neural network handles
local nn = nil
local nn_model = nil

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
  targetWheelSpeed = nil,  -- Target wheel speed (m/s), nil means no command
  targetVehicleSpeed = nil, -- Target vehicle speed (m/s), nil means no command

  -- NN control state
  old_diff_rear_wheel_speed_ms = 0,  -- Previous wheel speed error for derivative
  leaky_err_wr = 0,                  -- Leaky integrator for wheel speed error
  
  -- Tracking
  lastAppliedThrottle = 0,
  lastAppliedSteering = 0,
  lastAppliedBrake = 0,
}

-- Controller calibration parameters
local calibration = {
  commandTimeout = 0.5,          -- Timeout in seconds before reverting to manual control
  
  -- Neural network model settings
  modelName = 'wheel_speed_v4.json',  -- NN model file name
  modelPath = nil,                     -- Full model path (takes precedence if set)
  
  -- Torque control PID gains
  TorqueKp = 0.1,
  TorqueKi = 0.05,
  TorqueKd = 0.01,
  
  -- Feedforward coefficient: 1.0 = pure FF, 0.0 = pure PID
  ffcoef = 0.7,
  
  -- Steering control gains
  steeringKp = 0.0,              -- Proportional gain for steering correction
  steeringKi = 0.0,              -- Integral gain for steering correction
  steeringIntegAlpha = 0.9,      -- Leaky integrator for steering
  steeringToInput = 0.5,         -- Conversion factor: road_wheel_angle = steering_input * steeringToInput

  -- Speed control scaling
  vel_time_scale = 1.0,
}

-- Reference to the gtState controller
local gtStateController = nil

--[[
    Parse incoming JSON command message.
    Expected format: {
      "torque": <number>,        -- Optional: target wheel torque in Nm
      "steering": <number>,      -- Optional: target steering input [-1, 1]
      "brake": <number>,         -- Optional: target brake input [0, 1]
      "wheel_speed": <number>,   -- Optional: target wheel speed in m/s
      "vehicle_speed": <number>  -- Optional: target vehicle speed in m/s
    }
    
    Any field can be null/missing - that field will be set to nil.
    
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

  -- Extract target values (they can be nil/null - if missing, variable becomes nil)
  controllerState.targetTorque = data.torque
  controllerState.targetSteering = data.steering
  controllerState.targetBrake = data.brake
  controllerState.targetWheelSpeed = data.wheel_speed
  controllerState.targetVehicleSpeed = data.vehicle_speed

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
    Apply torque control using NN feedforward + PID feedback.
    Converts wheel torque or wheel speed target to throttle input.
    
    Parameters:
        targetWheelTorque (number): Desired wheel torque in Nm (can be nil)
        dt (number): Time step in seconds
]]
local function applyTorqueControl(dt)
  local cs = controllerState
  local vr = common.cachedGtReading
  
  if not vr then
    log('E', logTag, 'Ground truth reading unavailable')
    return
  end
  
  -- Get current state
  local rear_wheel_speed_ms = (vr.wheelRR.speed + vr.wheelRL.speed) / 2
  local engine_speed_rad = vr.RPM * common.constants.rpmToAV
  local boost_pressure = vr.turboBoost
  local throttle = vr.throttle
  
  -- Training configuration parameters (should match NN training)
  local train_dt = 0.01
  local max_error_wr = 100
  local tau_train = 0.01
  
  if not nn_model then
    log('E', logTag, 'NN model not loaded')
    return
  end

  -- Start with feedforward throttle from NN
  local ffthrottle
  if cs.targetTorque ~= nil then
    -- Run NN to get feedforward throttle
    local out = nn.run(nn_model, {
        engine_speed_rad,
        boost_pressure,
        rear_wheel_speed_ms,
        cs.targetTorque,
    })
    ffthrottle = out[1]
  end

  -- Compute wheel speed error
  local pid_throttle
  if cs.targetWheelSpeed ~= nil then
    local diff_rear_wheel_speed_ms = rear_wheel_speed_ms - cs.targetWheelSpeed
    local alpha = train_dt / (train_dt + tau_train)
    local delta_diff = diff_rear_wheel_speed_ms - cs.old_diff_rear_wheel_speed_ms
    cs.old_diff_rear_wheel_speed_ms = diff_rear_wheel_speed_ms
    cs.leaky_err_wr = alpha * cs.leaky_err_wr + diff_rear_wheel_speed_ms * train_dt

    -- Compute PID feedback correction
    pid_throttle = calibration.TorqueKp * diff_rear_wheel_speed_ms 
                        + calibration.TorqueKi * cs.leaky_err_wr 
                        + calibration.TorqueKd * (delta_diff / train_dt)
    pid_throttle = max(-1.0, min(1.0, pid_throttle))
    pid_throttle = 0.5 * pid_throttle + 0.5  -- Map from [-1,1] to [0,1]
    -- Smooth PID application
    local pid_throttle_dot = (pid_throttle - throttle) / 1.0
    pid_throttle = throttle + 0.005 * pid_throttle_dot
  end

  -- Blend feedforward and feedback
  if pid_throttle == nil then pid_throttle = ffthrottle end
  if ffthrottle == nil then ffthrottle = pid_throttle end
  local newThrottle = calibration.ffcoef * ffthrottle + (1.0 - calibration.ffcoef) * pid_throttle
  
  -- Clamp throttle to valid range [0, 1]
  newThrottle = max(0, min(1, newThrottle))
  
  -- Apply throttle
  input.event('throttle', newThrottle, FILTER_AI)
  electrics.values.throttle = newThrottle
  electrics.values.throttle_input = newThrottle
  
  cs.lastAppliedThrottle = newThrottle
end

local function applySpeedControl(dt)
  local cs = controllerState
  if cs.targetVehicleSpeed == nil then return end
  local desiredVelocity = obj:getDirectionVector() * cs.targetVehicleSpeed
  thrusters.applyVelocity(desiredVelocity, calibration.vel_time_scale)
end

--[[
    Apply steering control.
    Optionally uses PID for correction if gains are non-zero.
    
    Parameters:
        targetSteeringInput (number): Desired steering input [-1, 1]
]]
local function applySteeringControl(targetSteeringInput)
  -- local vr = common.cachedGtReading
  
  -- -- Simple direct application (optional: add PID correction later)
  -- local roadwheelAngle = (vr.wheelFR.angleAtan2 + vr.wheelFL.angleAtan2) / 2
  -- local currentSteeringInput = roadwheelAngle / (calibration.steeringToInput + 1e-6)
  
  -- -- Optional PID correction (only if gains are non-zero)
  -- local steeringCmd = targetSteeringInput
  -- if calibration.steeringKp ~= 0 or calibration.steeringKi ~= 0 then
  --   local err = targetSteeringInput - currentSteeringInput
  --   -- Could add integral/derivative here if needed
  --   steeringCmd = targetSteeringInput + calibration.steeringKp * err
  -- end
  
  -- Clamp to [-1, 1]
  steeringCmd = min(1, max(-1, targetSteeringInput))
  
  controllerState.lastAppliedSteering = steeringCmd
  input.event('steering', steeringCmd, FILTER_AI)
  electrics.values.steering_input = steeringCmd
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
      cs.old_diff_rear_wheel_speed_ms = 0
      cs.leaky_err_wr = 0
      cs.targetTorque = nil
      cs.targetSteering = nil
      cs.targetBrake = nil
      cs.targetWheelSpeed = nil
      cs.targetVehicleSpeed = nil
    end
    
    -- Do NOT apply any controls - user has full manual control
    return
  end
  
  -- Controller is active - apply commanded controls
  
  -- Apply torque control if specified
  if cs.targetTorque ~= nil or cs.targetWheelSpeed ~= nil then
    applyTorqueControl(dt)
  end

  -- Apply speed control if specified
  if cs.targetVehicleSpeed ~= nil then
    applySpeedControl(dt)
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
    gtStateController.registerCustomField("target_wr", 0)
    gtStateController.registerCustomField("target_v", 0)
    -- gtStateController.registerCustomField("target_steering", 0)
    -- gtStateController.registerCustomField("target_brake", 0)
    log('I', logTag, 'Registered custom fields with gtState')
  else
    log('W', logTag, 'Could not register custom fields with gtState')
  end
  
  -- Initialize neural network library
  nn = require('lua/vehicle/controller/xlab/lib/nn')
  if not nn then
    log('E', logTag, 'Failed to load nn.lua library')
    return false
  end
  nn.init()
  
  -- Use model path from calibration if provided, otherwise build from model name
  local modelPath = calibration.modelPath
  if not modelPath then
    local modelName = calibration.modelName
    modelPath = 'lua/vehicle/controller/xlab/models/' .. modelName
  end
  
  nn_model = nn.loadModel(modelPath)
  if not nn_model then
    log('E', logTag, 'Failed to load NN model from ' .. modelPath)
    return false
  end
  
  log('I', logTag, 'NN Torque controller initialized with model: ' .. modelPath)
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
    gtStateController.setCustomField("target_wr", controllerState.targetWheelSpeed or 0)
    gtStateController.setCustomField("target_v", controllerState.targetVehicleSpeed or 0)
    -- gtStateController.setCustomField("target_steering", controllerState.targetSteering or 0)
    -- gtStateController.setCustomField("target_brake", controllerState.targetBrake or 0)
  end
  
  -- Periodic logging (1 Hz)
  updatesSinceLog = updatesSinceLog + 1
  if nowSim - lastLogSimTime >= 5.0 then
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
  if nn_model then
    nn.freeModel(nn_model)
    nn_model = nil
  end
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
  
  -- Handle model specification - prioritize full path over just name
  if params.modelPath ~= nil then
    calibration.modelPath = params.modelPath
    -- Extract model name from path for compatibility
    local _, filename = path.splitpath(params.modelPath)
    calibration.modelName = filename
    log('I', logTag, 'modelPath = ' .. tostring(calibration.modelPath))
  elseif params.modelName ~= nil then
    calibration.modelName = params.modelName
    calibration.modelPath = 'lua/vehicle/controller/xlab/models/' .. params.modelName
    log('I', logTag, 'modelName = ' .. tostring(calibration.modelName))
  end
  
  -- Handle numeric parameters
  local numericParams = {
    'commandTimeout', 'ctrl_type', 'TorqueKp', 'TorqueKi', 'TorqueKd',
    'ffcoef', 'steeringKp', 'steeringKi', 'steeringIntegAlpha', 'steeringToInput',
    'vel_time_scale'
  }
  
  for _, key in ipairs(numericParams) do
    if params[key] ~= nil then
      calibration[key] = tonumber(params[key]) or calibration[key]
      log('I', logTag, key .. ' = ' .. tostring(calibration[key]))
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
    targetWheelSpeed = nil,
    targetVehicleSpeed = nil,
    old_diff_rear_wheel_speed_ms = 0,
    leaky_err_wr = 0,
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
