-- ./vehicle/controller/xlab/controller_nn_mpc.lua
local M = {}
local common
local logTag = 'controller_nn_mpc'

-- Import necessary math functions for performance
local tanh = math.tanh
local atan2 = math.atan2
local abs = math.abs
local sqrt = math.sqrt
local max = math.max
local min = math.min

local _ = nil -- avoid global warning

-- Control mode enumeration
local CONTROL_MODE = {
  AUTO = 1,          -- Full autonomous control (steering and throttle)
  STEERING_AUTO = 2  -- Steering-only autonomous control (human throttle)
}

-- MPC controller mode states
local MPC_MODE = {
  OFF = 0,              -- Controller is off or has timed out
  ON = 1,               -- Controller is active and receiving commands
  TARGETCOUNT_PASSED = 2 -- All targets in current command set have been used
}

-- Neural network handles
local nn = nil
local nn_model = nil

-- Timing and rate control variables
local updateAccum = 0
local messageCounter = 0
local nowSim = 0
local gtStateSendAccum = 0
local lastMPCTime = 0
local mpc_has_timeout = true
local mpcMode = MPC_MODE.OFF -- Current MPC controller mode

-- Real-time logging variables
local nowClock = 0
local realLastClock = os.clock()
local realUpdateCount = 0

-- Controller state tracking
-- local controllerState = {
--   x = {},      -- x positions array
--   y = {},      -- y positions array
--   s = {},      -- s values array (path distance)
--   wr = {},     -- wheel speed targets
--   delta = {},  -- steering angle targets (in radians)
--   steer = {},  -- steering inputs (-1 to 1)
--   currentIdx = 1,
--   targetCount = 0,
--   old_diff_rear_wheel_speed_ms = 0,
--   leaky_err_wr = 0,
--   integ_steering_err = 0,
-- }
local controllerState = {
  x = {},      -- x positions array
  y = {},      -- y positions array
  s = {},      -- s values array (path distance)
  wr = {},     -- wheel speed targets
  steer = {},  -- steering inputs (-1 to 1)
  desired_torque = {}, -- desired torque targets
  currentIdx = 1,
  targetCount = 0,
  old_diff_rear_wheel_speed_ms = 0,
  leaky_err_wr = 0,
  integ_steering_err = 0,
}

-- Controller calibration parameters
local calibration = {
  controlMode = CONTROL_MODE.STEERING_AUTO,   -- Control mode
  gtStateSendRate = 0.01,                     -- dt between sending gtstate via UDP
  modelName = 'wheel_speed_v4.json',          -- NN model file name
  modelPath = nil,                            -- Full model path (takes precedence if set)
  commandTimeout = 1.0,                       -- Timeout between two mpc solutions before reverting to manual control
}

-- Reference to the gtState controller
local gtStateController = nil

--[[
    Packs vehicle state data for sending over UDP.
    
    Parameters:
        gtReading (table): The ground truth reading data structure
    
    Returns:
        table: A compact representation of vehicle state with key metrics
]]
local function packStateData(gtReading)
  if not gtReading then
    log('E', logTag, 'GT reading is nil')
    return {t = nowSim}
  end
  
  -- Get position (x, y)
  local pos_x = gtReading.pos[1]
  local pos_y = gtReading.pos[2]
  
  -- Get velocity (vx, vy) - these are already in local frame
  local vel_x = gtReading.vel[1] -- Forward velocity
  local vel_y = gtReading.vel[2] -- Lateral velocity
  
  -- Calculate wheel angles (average of left and right)
  local front_wheel_angle = (gtReading.wheelFR.angle + gtReading.wheelFL.angle) / 2
  
  -- Calculate wheel speeds (average of left and right)
  local front_wheel_speed = (gtReading.wheelFR.speed + gtReading.wheelFL.speed) / 2
  local rear_wheel_speed = (gtReading.wheelRR.speed + gtReading.wheelRL.speed) / 2
  
  -- Get yaw rate directly from angular velocity z-component
  local yaw_rate = gtReading.angVel[3]
  
  -- Extract quaternion components, qx, qy, qz, qw
  local qx = gtReading.quat[1]
  local qy = gtReading.quat[2]
  local qz = gtReading.quat[3]
  local qw = gtReading.quat[4]
  
  -- Calculate yaw from quaternion
  -- Formula: yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
  local yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
  
  -- Calculate sideslip angle
  -- Formula: sideslip = atan2(vy, vx) - yaw
  -- But we use velocity in local frame, so we just need atan2(vy, vx)
  local V = sqrt(vel_x*vel_x + vel_y * vel_y)
  local sideslip = 0
  if abs(V) > 2.0 then -- Avoid division by zero or erratic values at very low speeds
    sideslip = atan2(vel_y, vel_x)
  end
  
  -- Pack all data into a state table
  return {
    t = gtReading.time,        -- Simulation time
    x = pos_x, y = pos_y,      -- Position
    V = V,                     -- Vehicle speed
    beta = sideslip,           -- Sideslip angle
    wr = rear_wheel_speed,     -- Rear wheel speed
    wf = front_wheel_speed,    -- Front wheel speed
    delta = front_wheel_angle, -- Front wheel angle
    r = yaw_rate,              -- Yaw rate
    Phi = sideslip + yaw,      -- Course angle
    yaw = yaw,                 -- Yaw angle
    we = gtReading.RPM * common.constants.rpmToAV, -- Engine speed in rad/s
    pb = gtReading.turboBoost, -- Boost pressure
    throttle = gtReading.throttle, -- Throttle position
    accel_x = gtReading.accel[1], -- Longitudinal acceleration
    accel_y = gtReading.accel[2], -- Lateral acceleration
  }
end


-- Parse incoming JSON “{ targets: [ { s, x,y,z, tx,ty,tz, wheel_speed }, … ] }”
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

  -- Validate the required arrays exist and have proper length
  if not data.x or not data.y or not data.wr or not data.steer then
    log('E', logTag, 'Missing required arrays in message')
    return false
  end

  -- Check array lengths
  local count = #data.x
  if #data.y ~= count or #data.wr ~= count or #data.steer ~= count then
    log('E', logTag, 'Array lengths mismatch in target message')
    return false
  end

  -- Store the arrays directly
  -- controllerState.x = data.x
  -- controllerState.y = data.y
  -- controllerState.wr = data.wr
  -- controllerState.steer = data.steer
  -- controllerState.wr_dot = data.wr_dot
  -- controllerState.Fxr = data.Fxr
  -- controllerState.targetCount = count
  -- controllerState.s = data.s
  -- controllerState.currentIdx = 1

  controllerState.x = data.x
  controllerState.y = data.y
  controllerState.wr = data.wr
  controllerState.steer = data.steer
  controllerState.desired_torque = data.desired_torque
  controllerState.wr_dot = data.wr_dot
  controllerState.Fxr = data.Fxr
  controllerState.targetCount = count
  controllerState.s = data.s
  controllerState.currentIdx = 1

  gtStateController.setCustomField("mpc_dt", nowSim - lastMPCTime)
  lastMPCTime = nowSim
  mpcMode = MPC_MODE.ON -- Set MPC mode to ON
  mpc_has_timeout = false -- Reset MPC timeout flag

  local pm = common.performanceMetrics
  pm.lastCommandTimestamp = nowSim
  pm.commandsReceived = pm.commandsReceived + 1
  messageCounter = messageCounter + 1

  return true
end

--[[
    Finds the closest target point to the current vehicle position.
    
    Parameters:
        vehicle_x (number): Vehicle x position
        vehicle_y (number): Vehicle y position
    
    Returns:
        number: Index of the closest target point
]]
local function findClosestTargetIdx(vehicle_x, vehicle_y)
  local cs = controllerState

  local closest_idx = 1
  local min_dist_sq = 1e10
  
  for i = 1, cs.targetCount do
    local dx = cs.x[i] - vehicle_x
    local dy = cs.y[i] - vehicle_y
    local dist_sq = dx*dx + dy*dy
    
    if dist_sq < min_dist_sq then
      min_dist_sq = dist_sq
      closest_idx = i
    end
  end
  
  return closest_idx
end

--[[
  Finds closest target point and t in [0,1] the along-segment projection of the current
  target along that segment.
]]
local function  findClosestSegment(vehicle_x, vehicle_y)
  local cs = controllerState
  local N = cs.targetCount
  local best_i, best_j = 1, 2
  local best_t = 0.0
  local best_px, best_py = cs.x[1], cs.y[1]
  local best_d2 = 1e10
  for i = 1, N-1 do
    local j = (i < N) and (i+1) or 1
    local x1, y1 = cs.x[i], cs.y[i]
    local x2, y2 = cs.x[j], cs.y[j]
    local vx, vy = x2 - x1, y2 - y1
    local wx, wy = vehicle_x - x1, vehicle_y - y1
    local seg_len2 = vx*vx + vy*vy
    local t, px, py
    if seg_len2 <= 1e-12 then
      t = 0.0
      px, py = x1, y1
    else
      t = (wx*vx + wy*vy) / seg_len2
      t = max(0, min(1, t))
      px = x1 + t * vx
      py = y1 + t * vy
    end

    local dx, dy = vehicle_x - px, vehicle_y - py
    local d2 = dx*dx + dy*dy
    if d2 < best_d2 then
      best_d2 = d2
      best_i, best_j = i, j
      best_t = t
      best_px, best_py = px, py
    end
  end
  return best_i, best_j, best_t
end

--[[
    Applies target values from the trajectory to the vehicle.
    Performs path projection, target interpolation, and NN control.
    
    Parameters:
        dt (number): Time step in seconds
]]
local function applyTargets(dt)

  -- Do nothing if the high-level controller has timeout
  if mpc_has_timeout then
    return
  end

  local cs = controllerState
  local g = common.cachedGtReading

  -- Get vehicle position
  local vehicle_x = g.pos[1]
  local vehicle_y = g.pos[2]

  -- -- Find closest target index based on distance
  -- local closest_idx = findClosestTargetIdx(vehicle_x, vehicle_y)

  -- -- Check if we've reached the end of the path
  -- if closest_idx >= cs.targetCount then
  --   log('W', logTag, string.format(
  --     'End of path target list (idx %d of %d)', closest_idx, cs.targetCount)
  --   )
  --   mpcMode = MPC_MODE.TARGETCOUNT_PASSED
  --   cs.integ_steering_err = 0
  --   cs.old_diff_rear_wheel_speed_ms = 0
  -- end

  local idx1, idx2, t = findClosestSegment(vehicle_x, vehicle_y)

  -- Check if we've reached the end of the path
  if idx2 >= cs.targetCount and t >= 1.0 then
    log('W', logTag, string.format(
      'End of path target list (idx %d of %d)', idx2, cs.targetCount)
    )
    mpcMode = MPC_MODE.TARGETCOUNT_PASSED
    cs.integ_steering_err = 0
    cs.old_diff_rear_wheel_speed_ms = 0
    cs.leaky_err_wr = 0
  end

  -- -- Get indices for the forward segment, ensuring they're valid
  -- local idx1 = min(closest_idx, cs.targetCount - 1)  -- Start index (closest point)
  -- local idx2 = min(closest_idx + 1, cs.targetCount)  -- End index (next point)
  
  -- -- Extract segment endpoints
  -- local x1 = cs.x[idx1]
  -- local y1 = cs.y[idx1]
  -- local x2 = cs.x[idx2]
  -- local y2 = cs.y[idx2]
  
  -- -- Compute segment vector for projection
  -- local dx_seg = x2 - x1
  -- local dy_seg = y2 - y1
  -- local seg_length_squared = dx_seg*dx_seg + dy_seg*dy_seg
  
  -- -- Compute projection parameter
  -- local t_raw = 0
  -- if seg_length_squared > 1e-7 then -- Avoid division by near-zero
  --   t_raw = ((vehicle_x - x1) * dx_seg + (vehicle_y - y1) * dy_seg) / seg_length_squared
  -- end
  -- local t = max(0, min(1, t_raw)) -- Clamp to [0,1]

  -- Store current position in path for status reporting
  cs.currentIdx = idx1
  
  -- -- Interpolate target values using projection parameter t
  -- -- Maybe an interpolation to pick the next time or s + ds.
  -- local desiredSteer = cs.steer[idx1] + t * (cs.steer[idx2] - cs.steer[idx1])
  -- local desiredWheelSpeed = cs.wr[idx1] + t * (cs.wr[idx2] - cs.wr[idx1])
  local desired_wr_dot = cs.wr_dot[idx1] + t * (cs.wr_dot[idx2] - cs.wr_dot[idx1])
  local desiredFxr = cs.Fxr[idx1] + t * (cs.Fxr[idx2] - cs.Fxr[idx1])
  
  local desiredSteer = cs.steer[idx1] + t * (cs.steer[idx2] - cs.steer[idx1])
  local desiredWheelSpeed = cs.wr[idx1] + t * (cs.wr[idx2] - cs.wr[idx1])
  local desired_torque = cs.desired_torque[idx1] + t * (cs.desired_torque[idx2] - cs.desired_torque[idx1])

  -- local idx_next = min(idx1 + 1, cs.targetCount)
  local idx_next = idx2
  -- desiredSteer = cs.steer[idx_next]
  -- desiredWheelSpeed = cs.wr[idx_next]
  -- desired_wr_dot = cs.wr_dot[idx_next]
  -- desiredFxr = cs.Fxr[idx_next]
  desiredSteer = cs.steer[idx_next]
  desiredWheelSpeed = cs.wr[idx_next]
  desired_wr_dot = cs.wr_dot[idx_next]
  desiredFxr = cs.Fxr[idx_next]
  desired_torque = cs.desired_torque[idx_next]

  -- local curr_velocity = g.vel[1]
  -- if curr_velocity <= 4 then
  --     local index_wr = cs.targetCount
  --     desiredWheelSpeed = cs.wr[index_wr]
  --     desiredFxr = cs.Fxr[index_wr]
  --     desired_wr_dot = 0
  -- end

  -- local idx_next = min(idx1 + 1, cs.targetCount)
  -- local desiredSteer = cs.steer[idx_next]
  -- local desiredWheelSpeed = cs.wr[idx_next]
  -- local desired_wr_dot = cs.wr_dot[idx_next]
  -- local desiredFxr = cs.Fxr[idx_next]

  -- Write down the desired values into the gtState
  gtStateController.setCustomField("target_wr", desiredWheelSpeed)
  gtStateController.setCustomField("target_steer", desiredSteer)
  -- gtStateController.setCustomField("target_wr_dot", desired_wr_dot)
  -- gtStateController.setCustomField("target_Fxr", desiredFxr)
  gtStateController.setCustomField("target_torque", desired_torque)

  -- NN logic to compute throttle control
  local newThrottle
  local ffthrottle = 0.0
  local rear_wheel_speed_ms = (g.wheelRR.speed + g.wheelRL.speed) / 2
  local tau_train, max_error_wr, train_dt = 0.01, 100, 0.01 -- From training config
  if nn_model then
    -- Prepare input variables for the neural network
    local engine_speed_rad = g.RPM * common.constants.rpmToAV
    local boost_pressure = g.turboBoost
    local throttle = g.throttle
    local diff_rear_wheel_speed_ms = rear_wheel_speed_ms - desiredWheelSpeed
    local alpha = (train_dt / (train_dt + tau_train))
    if calibration.ctrl_type == 0 then
      local delta_diff = rear_wheel_speed_ms - cs.old_diff_rear_wheel_speed_ms
      cs.old_diff_rear_wheel_speed_ms = rear_wheel_speed_ms
      cs.leaky_err_wr = alpha * cs.leaky_err_wr + diff_rear_wheel_speed_ms
      cs.leaky_err_wr = max(-max_error_wr, min(max_error_wr, cs.leaky_err_wr))
      local out = nn.run(nn_model, {
        engine_speed_rad,
        boost_pressure,
        rear_wheel_speed_ms,
        throttle,
        desired_wr_dot * train_dt, -- Scale factor from training
        desiredFxr,
        delta_diff,
        cs.leaky_err_wr,
        diff_rear_wheel_speed_ms,
      })
      local ff = out[1]
      local kp = out[2]
      local throttledot = tanh(ff + kp * (diff_rear_wheel_speed_ms / 15.0)) -- TODO: Proper 15 calibration
      throttledot = 5.0 * throttledot
      newThrottle = throttle + (train_dt * throttledot)
      ffthrottle = ff
    elseif calibration.ctrl_type == 1 then
      local delta_diff = diff_rear_wheel_speed_ms - cs.old_diff_rear_wheel_speed_ms
      cs.old_diff_rear_wheel_speed_ms = delta_diff
      cs.leaky_err_wr = alpha * cs.leaky_err_wr + diff_rear_wheel_speed_ms * train_dt
      local out = nn.run(nn_model, {
        engine_speed_rad,
        boost_pressure,
        rear_wheel_speed_ms,
        desired_torque,
      })
      ffthrottle  = out[1]
      local pid_throttle = calibration.TorqueKp * diff_rear_wheel_speed_ms + calibration.TorqueKi * cs.leaky_err_wr + calibration.TorqueKd * (delta_diff / train_dt)
      pid_throttle= max(-1.0, min(1.0, pid_throttle))
      pid_throttle = 0.5* pid_throttle + 0.5
      local pid_throttle_dot = (pid_throttle - throttle) / 1.0 -- Train file
      pid_throttle = throttle + 0.005 * pid_throttle_dot -- Make 0.005 configurable?
      newThrottle = calibration.ffcoef * ffthrottle + (1.0 - calibration.ffcoef) * pid_throttle
    else
      -- Not a valid ctrl_type
      delta_diff = 0
      cs.old_diff_rear_wheel_speed_ms = 0
      cs.leaky_err_wr = 0
      log('E', logTag, 'Invalid ctrl_type in calibration: ' .. tostring(calibration.ctrl_type))
    end
  else
    log('E', logTag, 'NN model is not loaded')
  end

  -- Apply computed controls
  local roadwheel_angle = (g.wheelFR.angle + g.wheelFL.angle) / 2
  local current_steering = roadwheel_angle / calibration.steeringToInput
  local err_steer = (desiredSteer - current_steering)
  cs.integ_steering_err = calibration.steeringIntegAlpha * cs.integ_steering_err + err_steer
  local inputSteer = desiredSteer + calibration.steeringKp * err_steer + calibration.steeringKi * cs.integ_steering_err
  
  if calibration.controlMode == CONTROL_MODE.AUTO and newThrottle then
    -- Full autonomous mode - control both throttle and steering
    input.event('throttle', newThrottle, FILTER_AI)
    electrics.values.throttle = newThrottle
    electrics.values.throttle_input = newThrottle
    input.event('steering', inputSteer, FILTER_AI)
    electrics.values.steering_input = inputSteer
    -- electrics.values.steering = -4.71238898038469 * inputSteer
    -- input.event('steering', cs.lastAppliedSteering, FILTER_AI)
    
  elseif calibration.controlMode == CONTROL_MODE.STEERING_AUTO then
    -- Steering-only mode - only control steering
    input.event('steering', desiredSteer, FILTER_AI)
    -- electrics.values.steering_input = desiredSteer
    -- Throttle is left to the human driver
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
    local idx_s = cs.s[idx1] + t * (cs.s[idx2] - cs.s[idx1])
    log(
      'I',
      logTag,
      string.format(
        -- 'SimTime=%.3f Idx=%d/%d t=%.2f s=%.2f wr_t=%.2f wr=%.2f wrDot=%.2f steer=%.2f FxR=%.2f Latency=%.1fms',
        'SimTime=%.3f Idx=%d/%d t=%.2f s=%.2f wr_t=%.2f wr=%.2f tor=%.2f steer=%.2f ffthr=%.2f thr=%.2f Latency=%.1fms',
        nowSim,
        idx1,
        cs.targetCount,
        t,
        idx_s,
        rear_wheel_speed_ms,
        desiredWheelSpeed,
        desired_torque,
        -- desired_wr_dot * train_dt,
        desiredSteer-current_steering,
        -- desiredFxr,
        ffthrottle,
        newThrottle or -1,
        common.performanceMetrics.avgLatency * 1000
      )
    )
    realLastClock = nowClock
    realUpdateCount = 0
  end
end

-- Module interface ----------------------
--[[
    Initializes the controller with the common interface.
    Sets up the neural network, ground truth sensor connection,
    and registers custom fields.
    
    Parameters:
        c (table): Common interface table with shared resources
        
    Returns:
        boolean: True if initialization successful, false otherwise
]]
function M.init(c)
  common = c

  -- Get gtState controller
  if common.gtStateManager and common.gtStateSensorId then
    gtStateController = common.gtStateManager.getGtStateController(common.gtStateSensorId)
  else
    log('E', logTag, 'GT State sensor ID is not set')
    return false
  end
  
  -- Register custom fields with gtState for data sharing
  if gtStateController and gtStateController.registerCustomField then
    gtStateController.registerCustomField("target_wr", 0)
    gtStateController.registerCustomField("target_steer", 0)
    -- gtStateController.registerCustomField("target_wr_dot", 0)
    -- gtStateController.registerCustomField("target_Fxr", 0)
    gtStateController.registerCustomField("target_torque", 0)
    gtStateController.registerCustomField("mpc_mode", MPC_MODE.OFF)
    gtStateController.registerCustomField("mpc_dt", 0)
    -- gtStateController.registerCustomField("controller_mode", calibration.controlMode)
  else
    log('E', logTag, 'GT State controller not found or does not support custom fields')
    return false
  end

  -- Initialize neural network library
  nn = require('lua/vehicle/controller/xlab/lib/nn')
  assert(nn, 'nn.lua library could not be loaded')
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
  
  log('I', logTag, 'NN controller initialized with model: ' .. modelPath)

  -- Initialize PID gains if provided.

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
  gtStateSendAccum = gtStateSendAccum + dt
  
  -- Check if it's time to update
  if updateAccum < common.controllerRate then return end
  updateAccum = updateAccum - common.controllerRate
  
  -- Update simulation time and check for command timeout
  nowSim = common.getSimTime()

  -- Process network messages - drain UDP queue, keep only last message
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

  -- Check for MPC timeout
  local prevTimeout = mpc_has_timeout
  mpc_has_timeout = (nowSim - lastMPCTime) > calibration.commandTimeout
  if mpc_has_timeout and not prevTimeout and mpcMode == MPC_MODE.ON then
    -- First time timeout has occurred
    log('W', logTag, 'MPC command timeout - falling back to manual control after ' .. 
        string.format("%.2f", calibration.commandTimeout) .. 's')
    mpcMode = MPC_MODE.OFF
    controllerState.integ_steering_err = 0
    controllerState.old_diff_rear_wheel_speed_ms = 0
    controllerState.leaky_err_wr = 0
  end

  common.updateGtReading()
  if not common.isBypassed then applyTargets(dt) end

  -- send back minimal state based on gtState
  if gtStateSendAccum >= calibration.gtStateSendRate then
    gtStateSendAccum = gtStateSendAccum - calibration.gtStateSendRate
    if common.socketOut then
      local state = packStateData(common.cachedGtReading)
      -- log('I', logTag, 'Sending GT state: ' .. jsonEncode(state))
      local ok, js = pcall(jsonEncode, state)
      if ok then
        common.socketOut:sendto(js, common.sendIp, common.sendPort)
      else
        log('E', logTag, 'JSON encode error: ' .. tostring(js))
      end
    else
      log('E', logTag, 'socketOut is nil')
    end
  end

  -- Update the mpcMode based on current state
  gtStateController.setCustomField("mpc_mode", mpcMode)
  common.cachedGtReading = nil
end

--[[
    Cleans up resources when controller is stopped.
]]
function M.stop()
  log('I', logTag, 'NN controller cleanup')
  if nn_model then nn.freeModel(nn_model) end
end

--[[
    Sets the ground truth state sensor ID for the controller.
    
    Parameters:
        id (number): Sensor ID to connect to
]]
function M.setGtStateSensor(id) common.gtStateSensorId = id end

--[[
    Calibrates controller parameters from an input configuration.
    
    Parameters:
        params (table): Table containing calibration parameters
]]
function M.calibrate(params)
  log('I', logTag, 'Calibrating NN controller')
  
  if not params then return end
  
  -- Handle control mode parameter
  if params.controlMode ~= nil then
    if type(params.controlMode) == "string" then
      -- Convert string to enum
      if params.controlMode == "auto" then
        calibration.controlMode = CONTROL_MODE.AUTO
      elseif params.controlMode == "steering_auto" then
        calibration.controlMode = CONTROL_MODE.STEERING_AUTO
      end
    else
      -- Assume numeric value
      local mode = tonumber(params.controlMode)
      if mode == CONTROL_MODE.AUTO or mode == CONTROL_MODE.STEERING_AUTO then
        calibration.controlMode = mode
      end
    end
    log('I', logTag, 'controlMode = ' .. tostring(calibration.controlMode))
  end
  
  -- Handle numeric parameters
  if params.gtStateSendRate ~= nil then
    calibration.gtStateSendRate = tonumber(params.gtStateSendRate) or calibration.gtStateSendRate
    log('I', logTag, 'gtStateSendRate = ' .. tostring(calibration.gtStateSendRate))
  end
  
  if params.commandTimeout ~= nil then
    calibration.commandTimeout = tonumber(params.commandTimeout) or calibration.commandTimeout
    log('I', logTag, 'commandTimeout = ' .. tostring(calibration.commandTimeout))
  end
  
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

  -- Make sure PID gains are numbers if provided
  if params.steeringKi ~= nil then
    calibration.steeringKi = tonumber(params.steeringKi) or calibration.steeringKi
    log('I', logTag, 'steeringKi = ' .. tostring(calibration.steeringKi))
  end

  if params.TorqueKp ~= nil then
    calibration.TorqueKp = tonumber(params.TorqueKp) or calibration.TorqueKp
    log('I', logTag, 'TorqueKp = ' .. tostring(calibration.TorqueKp))
  end

  if params.TorqueKi ~= nil then
    calibration.TorqueKi = tonumber(params.TorqueKi) or calibration.TorqueKi
    log('I', logTag, 'TorqueKi = ' .. tostring(calibration.TorqueKi))
  end

  if params.TorqueKd ~= nil then
    calibration.TorqueKd = tonumber(params.TorqueKd) or calibration.TorqueKd
    log('I', logTag, 'TorqueKd = ' .. tostring(calibration.TorqueKd))
  end

  if params.steeringToInput ~= nil then
    calibration.steeringToInput = tonumber(params.steeringToInput) or calibration.steeringToInput
    log('I', logTag, 'steeringToInput = ' .. tostring(calibration.steeringToInput))
  end

  if params.steeringKp ~= nil then
    calibration.steeringKp = tonumber(params.steeringKp) or calibration.steeringKp
    log('I', logTag, 'steeringKp = ' .. tostring(calibration.steeringKp))
  end

  if params.steeringIntegAlpha ~= nil then
    calibration.steeringIntegAlpha = tonumber(params.steeringIntegAlpha) or calibration.steeringIntegAlpha
    log('I', logTag, 'steeringIntegAlpha = ' .. tostring(calibration.steeringIntegAlpha))
  end

  if params.ffcoef ~= nil then
    calibration.ffcoef = tonumber(params.ffcoef) or calibration.ffcoef
    log('I', logTag, 'ffcoef = ' .. tostring(calibration.ffcoef))
  end

  if params.ctrl_type ~= nil then
    calibration.ctrl_type = tonumber(params.ctrl_type) or calibration.ctrl_type
    log('I', logTag, 'ctrl_type = ' .. tostring(calibration.ctrl_type))
  end

end

--[[
    Resets controller state to initial values.
]]
function M.reset()
  controllerState = {
    x = {},
    y = {},
    s = {},
    wr = {},
    steer = {},
    desired_torque = {},
    currentIdx = 1,
    targetCount = 0,
    old_diff_rear_wheel_speed_ms = 0,
    integ_steering_err = 0,
    leaky_err_wr = 0,
  }
  updateAccum = 0
  gtStateSendAccum = 0
  messageCounter = 0
  common.lastGtReadingTime = 0
  common.cachedGtReading = nil
  lastMPCTime = 0
  mpc_has_timeout = true
  mpcMode = MPC_MODE.OFF -- Reset MPC mode to OFF
  log('I', logTag, 'NN controller state reset')
end

--[[
    Returns the current status of the controller.
    
    Returns:
        table: Contains controller state, metrics and calibration info
]]
function M.getStatus()
  return {
    isRunning = common.isRunning,
    performanceMetrics = common.performanceMetrics,
    calibration = calibration,
    controllerState = controllerState,
  }
end

return M
