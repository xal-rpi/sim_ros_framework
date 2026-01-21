-- Vehicle State Sensor Module
-- This module manages a vehicle state sensor, handling physics calculations,
-- orientation updates, wheel management, input processing, and data buffering
-- for graphical representation.

local M = {}

-- Import necessary math functions for performance
local sqrt, abs, acos, max, min = math.sqrt, math.abs, math.acos, math.max, math.min
local constants = { rpmToAV = 0.104719755, avToRPM = 9.549296596425384 }

local function sign(x) return max(min((x * 1e200) * 1e200, 1), -1) end

-- Logging tag for debugging purposes
local logTag = 'GtState'

-- Will store custom fields added by controllers or other modules.
local customFields = {}

-- Reference to the global state manager extension
local gtStateManager = nil

-- EMA alphas
-- local emaAlpha = {
--   vectors = {
--     accel = { x = 0.13, y = 0.18, z = 0.16 },
--     angAccel = { x = 0.15, y = 0.23, z = 0.15 },
--     angVel = { x = 0.16, y = 0.18, z = 0.12 },
--     vel = { x = 0.28, y = 0.29, z = 0.23 },
--   },
--   scalars = {
--     flywheelTorque = 0.27,
--     gearboxTorque = 0.17,
--   },
--   wheels = {
--     wheelFL = { angVel = 0.12, speed = 0.12 },
--     wheelFR = { angVel = 0.12, speed = 0.12 },
--     wheelRL = {
--       angle = 0.23,
--       angVel = 0.46,
--       angVelB = 0.31,
--       speed = 0.21,
--     },
--     wheelRR = {
--       angle = 0.23,
--       angVel = 0.46,
--       angVelB = 0.31,
--       speed = 0.21,
--     },
--   },
-- }

-- Update EMA for System ID - Less aggressive than before
local emaAlpha = {
  vectors = {
    accel = { x = 0.1, y = 0.1, z = 0.1 }, -- Unchanged
    angAccel = { x = 0.15, y = 0.23, z = 0.15 }, -- Unchanged
    angVel = { x = 0.8, y = 0.8, z = 0.8}, -- , , 0.3-0.6
    vel = { x = 0.8, y = 0.8, z = 0.8 },
  },
  scalars = {
    flywheelTorque = 0.27,
    gearboxTorque = 0.17,
  },
  wheels = {
    wheelFL = { angVel = 0.8, speed = 0.8},
    wheelFR = { angVel = 0.8, speed = 0.8},
    wheelRL = {
      -- angle = 0.5,
      angVel = 0.8,
      -- angVelB = 0.5,
      speed = 0.8,
    },
    wheelRR = {
      -- angle = 0.4,
      angVel = 0.8,
      -- angVelB = 0.5,
      speed = 0.8,
    },
  },
}
local idxOf = { x = 1, y = 2, z = 3 }

-- holds previous EMA values
local emaState = {}

--[[
Sensor Core Properties
----------------------
These variables define the sensor's unique identification, positioning,
update timings, and operational flags.
]]
local sensorId -- Unique identifier for the sensor
local GFXUpdateTime -- Time interval (seconds) between graphics updates
local nodeIndex1, nodeIndex2, nodeIndex3 -- Indices of the three nodes forming the sensor's triangle
local b1, b2, b3 -- Barycentric coordinates relative to the triangle
local w1, w2, w3 -- Non-negative interpolation weights based on barycentric coordinates
local signedProjDist -- Signed distance from the sensor to the triangle plane
local triangleSpaceForward -- Forward direction vector in triangle space
local triangleSpaceLeft -- Left direction vector in triangle space
local isVisualised = true -- Flag to indicate if sensor visualization is enabled
local isUsingGravity = false -- Flag to include gravity in acceleration calculations
local m1, m2, m3 = 0, 0, 0 -- TODO: Assume static mass for now

--[[
Timing and Buffering Variables
--------------------------------
These variables manage the timing of physics and graphics updates,
as well as buffering of sensor readings for graphical representation.
]]
local timeSinceLastPoll = 0.0 -- Time since the last graphics poll/update
local numPhysicsStepsForGFXSave = 1 -- Number of physics steps before saving data for graphics
local counterPhysicsSteps = 0 -- Counter for physics steps

-- Physics step parameters
local physicsTimer -- Timer to track physics update intervals
local physicsUpdateTime -- Time interval (seconds) between physics updates

-- Wheels information
-- Assuming 'wheels' is a global table accessible within this module
local wheelRotators, wheelIds = wheels.wheelRotators, wheels.wheelRotatorIDs -- References to wheel rotators and their IDs
local wheel_fr, wheel_fl, wheel_rr, wheel_rl = {}, {}, {}, {} -- Tables to store individual wheel data

-- Readings data
local readings = {} -- Circular buffer to store raw sensor readings
local readingIndex = 1 -- Current index in the readings buffer
local latestReading = { -- Table to store the latest sensor reading
  time = 0.0,
  dirX = { 0, 0, 0 },
  dirY = { 0, 0, 0 },
  dirZ = { 0, 0, 0 },
  accel = { 0, 0, 0 },
  angVel = { 0, 0, 0 },
  angAccel = { 0, 0, 0 },
  pos = { 0, 0, 0 },
  vel = { 0, 0, 0 },
  quat = { 0, 0, 0, 1 },
  wheel_fr = {},
  wheel_fl = {},
  wheel_rr = {},
  wheel_rl = {},
  steering = 0.0,
  throttle = 0.0,
  brake = 0.0,
  clutch = 0.0,
  pbrake = 0.0,
  steering_des = 0.0,
  throttle_des = 0.0,
  brake_des = 0.0,
  clutch_des = 0.0,
  pbrake_des = 0.0,
  -- TODO: More to add here
  -- driveStatus = nil  -- Added to store drive mode status
}

-- Extra variables for vehicle state sensor
local currVeh = nil
local sensorPos = vec3(0, 0, 0)
local currentDir = vec3(0, 0, 0)
local worldLeft = vec3(0, 0, 0)
local worldThird = vec3(0, 0, 0)

-- Store the engine and gearbox for more accurate data
local engine = nil
local gearbox = nil

--[[
    Initialize fields that can be store inside the gtstate sensor.
    Parameters:
        fieldName (string): The name of the custom field to register.
        defaultValue (any): The default value for the custom field.
    Returns:
        bool: True if the field was registered successfully
]]
local function registerCustomField(fieldName, defaultValue)
  customFields[fieldName] = defaultValue
  log('I', logTag, 'Registered custom field: ' .. fieldName)
  return true
end

--[[
    Set a custom field value.
    Parameters:
        fieldName (string): The name of the custom field to set.
        value (any): The value to assign to the custom field.
    Returns:
        bool: True if the field was set successfully, false if the field does not exist.
]]
local function setCustomField(fieldName, value)
  if customFields[fieldName] ~= nil then
    customFields[fieldName] = value
    return true
  else
    log('E', logTag, 'Custom field not found: ' .. fieldName)
  end
  return false
end

--[[
    Converts local frame unit vectors into a quaternion representing orientation.
    
    Parameters:
        dirX (vec3): The X-direction unit vector in the local frame.
        dirY (vec3): The Y-direction unit vector in the local frame.
        dirZ (vec3): The Z-direction unit vector in the local frame.
    
    Returns:
        table: A quaternion represented as {x, y, z, w}.
]]
local function getQuaternionFromDir(dirX, dirY, dirZ)
  -- Extract matrix components from direction vectors
  local m00, m01, m02 = dirX.x, dirY.x, dirZ.x
  local m10, m11, m12 = dirX.y, dirY.y, dirZ.y
  local m20, m21, m22 = dirX.z, dirY.z, dirZ.z

  -- Calculate the trace of the matrix
  local trace = m00 + m11 + m22

  if trace > 1.0e-6 then
    local s = 0.5 / sqrt(trace + 1.0)
    return {
      (m21 - m12) * s,
      (m02 - m20) * s,
      (m10 - m01) * s,
      0.25 / s,
    }
  elseif m00 > m11 and m00 > m22 then
    local s = 2.0 * sqrt(1.0 + m00 - m11 - m22)
    return {
      0.25 * s,
      (m01 + m10) / s,
      (m02 + m20) / s,
      (m21 - m12) / s,
    }
  elseif m11 > m22 then
    local s = 2.0 * sqrt(1.0 + m11 - m00 - m22)
    return {
      (m01 + m10) / s,
      0.25 * s,
      (m12 + m21) / s,
      (m02 - m20) / s,
    }
  else
    local s = 2.0 * sqrt(1.0 + m22 - m00 - m11)
    return {
      (m02 + m20) / s,
      (m12 + m21) / s,
      0.25 * s,
      (m10 - m01) / s,
    }
  end
end

--[[
Extracts and computes relevant wheel information.

Parameters:
    mWheel (table): A table containing wheel properties

Returns:
    table: A table containing processed wheel information:
        - speed (number): Wheel speed in m/s.
        - angVelB (number): Angular velocity from brake torque in rad/s.
        - angVel (number): Current angular velocity in rad/s.
        - brakeTorque (number): Net brake torque applied.
        - propTorque (number): Propulsion torque applied.
        - angle (number): Wheel angle (initialized to 0.0).
]]
local function getWheelInfos(mWheel)
  local wheelDir = mWheel.wheelDir
  local frictionTorque = mWheel.frictionTorque

  -- Some of these values are redundant, but we'll keep them for now
  return {
    speed = mWheel.wheelSpeed, -- Wheel speed with effective radius in m/s
    angVelB = mWheel.angularVelocityBrakeCouple * wheelDir, -- Angular velocity from brake torque in rad/s
    angVel = mWheel.angularVelocity * wheelDir, -- Current angular velocity in rad/s
    brakeTorque = abs(mWheel.coreData.brakeTorqueApplied) - frictionTorque, -- Net brake torque
    propTorque = mWheel.propulsionTorque * wheelDir, -- Propulsion torque applied
    angle = 0.0, -- Wheel angle initialized to zero
    downForce = mWheel.downForce or 0,
  }
end

-- Physics step update for this sensor instance.
local function update(dtSim)
  -- Cycle the physics update timer. If we are not ready for a
  -- physics step update, leave immediately.
  if physicsTimer < physicsUpdateTime then
    physicsTimer = physicsTimer + dtSim
    return
  end
  physicsTimer = physicsTimer - physicsUpdateTime

  -- Compute the current position of the nodes defining the sensor wrt ref node.
  local node1 = currVeh:getNodePosition(nodeIndex1)
  local node2 = currVeh:getNodePosition(nodeIndex2)
  local node3 = currVeh:getNodePosition(nodeIndex3)

  -- Relvant edge vectors and normal vector of the triangle.
  local edge1, edge2 = node2 - node1, node3 - node1
  local edge1Norm, edge2Norm = edge1:normalized(), edge2:normalized()
  local normal = edge1Norm:cross(edge2Norm):normalized() -- TODO: maybe no need to normalize.

  -- Convert the fixed triangle-space coordinate system to world space
  local triangleThird = edge1Norm:cross(normal):normalized() -- TODO: maybe no need to normalize
  currentDir = (
    edge1Norm * triangleSpaceForward.x
    + normal * triangleSpaceForward.y
    + triangleThird * triangleSpaceForward.z
  ):normalized()
  worldLeft = (
    edge1Norm * triangleSpaceLeft.x
    + normal * triangleSpaceLeft.y
    + triangleThird * triangleSpaceLeft.z
  ):normalized()
  worldThird = currentDir:cross(worldLeft):normalized()

  -- Relative position of the sensor from the barycenter of the triangle.
  local currentPos = node1 + b1 * edge2 + b2 * edge1 + signedProjDist * normal
  sensorPos = currentPos + currVeh:getPosition()

  -- --------------------------------------------------------------------------
  --                  [Angular] Velocity and Accelerations                   --
  -- --------------------------------------------------------------------------

  -- Compute the acceleration vectors at each node, using Newton II [a := F / m].
  local a1 = currVeh:getNodeForceVector(nodeIndex1) / m1
  local a2 = currVeh:getNodeForceVector(nodeIndex2) / m2
  local a3 = currVeh:getNodeForceVector(nodeIndex3) / m3

  -- Get the velocity vector at each node.
  local v1 = currVeh:getNodeVelocityVector(nodeIndex1)
  local v2 = currVeh:getNodeVelocityVector(nodeIndex2)
  local v3 = currVeh:getNodeVelocityVector(nodeIndex3)

  -- Compute the rotational component of each nodal acceleration vector,
  -- by subtracting the aCenter component.
  local aCenter = (a1 + a2 + a3) / 3
  local vCenter = (v1 + v2 + v3) / 3

  -- Compute the curl and divergence at the projected point (on triangle plane).
  -- vectors from the barycenter to each node.
  local baryCenter = (node1 + node2 + node3) / 3
  local r = currentPos - baryCenter
  local r1, r2, r3 = node1 - baryCenter, node2 - baryCenter, node3 - baryCenter
  local _deNom = r1:squaredLength() * w1 + r2:squaredLength() * w2 + r3:squaredLength() * w3
  local invDenom = 1.0 / (_deNom + 1e-30)

  local aRot1, aRot2, aRot3 = a1 - aCenter, a2 - aCenter, a3 - aCenter
  local vRot1, vRot2, vRot3 = v1 - vCenter, v2 - vCenter, v3 - vCenter
  local curlAcc = r1:cross(aRot1) * w1 + r2:cross(aRot2) * w2 + r3:cross(aRot3) * w3
  local curlVel = r1:cross(vRot1) * w1 + r2:cross(vRot2) * w2 + r3:cross(vRot3) * w3
  local divAcc = r1:dot(aRot1) * w1 + r2:dot(aRot2) * w2 + r3:dot(aRot3) * w3
  local divVel = r1:dot(vRot1) * w1 + r2:dot(vRot2) * w2 + r3:dot(vRot3) * w3

  -- Compute the total acceleration vector at the sensor position
  local accelWorld = aCenter + (curlAcc:cross(r) + divAcc * r) * invDenom
  local velWorld = vCenter + (curlVel:cross(r) + divVel * r) * invDenom
  if isUsingGravity then accelWorld = accelWorld + currVeh:getGravityVector() end

  -- Compute the angular velocity and angular acceleration.
  local angVel = curlVel * invDenom
  local angAccel = curlAcc * invDenom

  -- Transform the quantities in the local frame.
  local accelLocal =
    vec3(accelWorld:dot(currentDir), accelWorld:dot(worldLeft), accelWorld:dot(worldThird))

  local velLocal = vec3(velWorld:dot(currentDir), velWorld:dot(worldLeft), velWorld:dot(worldThird))

  local angVelLocal = vec3(angVel:dot(currentDir), angVel:dot(worldLeft), angVel:dot(worldThird))

  local angAccelLocal =
    vec3(angAccel:dot(currentDir), angAccel:dot(worldLeft), angAccel:dot(worldThird))
  -- -----------------------------------------------------------------------

  -- ------------------------------------------------------------------------
  --                     Quaternion/Orientation Calculation                --
  -- -----------------------------------------------------------------------

  -- Compute the orientation of the sensor.
  local quatEst = getQuaternionFromDir(currentDir, worldLeft, worldThird) -- qx, qy, qz, qw

  -- ----------------------------------------------------------------------
  --                     WheelAngle Calculation                          --
  -- ----------------------------------------------------------------------
  -- local signSteering = sign(electrics.values.steering_input)

  -- helper: planar angle (rad) between two nodes, with sign
  local function planarAngleRad(nodeA, nodeB)
    local c = obj:nodeVecPlanarCosRightForward(nodeA, nodeB)
    local s = obj:nodeVecPlanarSinRightForward(nodeA, nodeB)
    return math.atan2(s, c)
  end

  -- front right
  local wheel_fr_info = getWheelInfos(wheel_fr)
  wheel_fr_info.angle = planarAngleRad(wheel_fr.node1, wheel_fr.node2)

  -- front left (note swapped order if needed)
  local wheel_fl_info = getWheelInfos(wheel_fl)
  wheel_fl_info.angle = planarAngleRad(wheel_fl.node2, wheel_fl.node1)

  -- rear right
  local wheel_rr_info = getWheelInfos(wheel_rr)
  wheel_rr_info.angle = planarAngleRad(wheel_rr.node1, wheel_rr.node2)

  -- rear left
  local wheel_rl_info = getWheelInfos(wheel_rl)
  wheel_rl_info.angle = planarAngleRad(wheel_rl.node2, wheel_rl.node1)
  --  -------------------------------------------------------

  -- These inputs are updated at a lower frequency than the physics steps.
  local elecVals = electrics.values
  -- Extract the drive status from the state manager.
  local driveModeStatus = gtStateManager.getDriveModeStatus()

  -- Gather the latest reading data.
  latestReading = {
    time = currVeh:getSimTime(), -- Current simulation time
    dirX = currentDir:toTable(), -- In the global frame
    dirY = worldLeft:toTable(), -- In the global frame
    -- dirZ = worldThird:toTable(),
    vel = velLocal:toTable(), -- In the local frame
    accel = accelLocal:toTable(), -- In the local frame
    angVel = angVelLocal:toTable(), -- In the local frame
    angAccel = angAccelLocal:toTable(), -- Maybe be unnecessary
    pos = sensorPos:toTable(), -- In the global frame
    quat = quatEst, -- wrt global frame
    wheelFR = wheel_fr_info,
    wheelFL = wheel_fl_info,
    wheelRR = wheel_rr_info,
    wheelRL = wheel_rl_info,
    steering = elecVals.steering, -- Lower frequency update - Maybe fetch from hydro directly
    throttle = elecVals.throttle, -- Low freq, input could be altered by mainController
    brake = elecVals.brake, -- Low freq, input could be altered by mainController
    clutch = elecVals.clutch, -- Low freq, input could be altered by mainController
    pbrake = elecVals.parkingbrake, -- No need for input since they are the same, used as handbrake
    steeringInput = elecVals.steering_input, -- Should be the same as input.steering
    throttleInput = elecVals.throttle_input, -- Should be the same as input.throttle
    brakeInput = elecVals.brake_input, -- Should be the same as input.brake
    clutchInput = elecVals.clutch_input, -- Should be the same as input.clutch
    -- Relevant vehicle mode / state data.
    driveStatus = driveModeStatus,
    -- Engine relevant information
    engineLoad = engine and (engine.isDisabled and 0 or engine.instantEngineLoad) or 0,
    engineTorque = engine and engine.combustionTorque or 0,
    RPM = engine and (engine.outputAV1 * constants.avToRPM) or 0,
    flywheelTorque = engine and engine.outputTorque1 or 0,
    turboBoost = elecVals.turboBoost or -1, -- PSI
    superchargerBoost = elecVals.superchargerBoost or -1, -- PSI
    throttleValve = engine and engine.throttle,
    -- Gearbox relevant information
    gearboxTorque = gearbox and gearbox.outputTorque1 or 0,
    gearRatio = gearbox and gearbox.gearRatio or 0,
    gearIndex = elecVals.gearIndex,
  }

  -- Apply EMA filter
  -- 1) vectors
  for vecName, comps in pairs(emaAlpha.vectors) do
    local vec = latestReading[vecName]
    if vec then
      for compName, alpha in pairs(comps) do
        local i = idxOf[compName]
        local raw = vec[i]
        local key = vecName .. '_' .. compName
        local prev = emaState[key]
        local filt = prev and (alpha * raw + (1 - alpha) * prev) or raw
        emaState[key] = filt
        vec[i] = filt
      end
    end
  end

  -- 2) scalars
  for name, alpha in pairs(emaAlpha.scalars) do
    local raw = latestReading[name]
    if raw ~= nil then
      local prev = emaState[name]
      local filt = prev and (alpha * raw + (1 - alpha) * prev) or raw
      emaState[name] = filt
      latestReading[name] = filt
    end
  end

  -- 3) wheels
  for wheelName, comps in pairs(emaAlpha.wheels) do
    local wh = latestReading[wheelName]
    if wh then
      for compName, alpha in pairs(comps) do
        local raw = wh[compName]
        if raw ~= nil then
          local key = wheelName .. '_' .. compName
          local prev = emaState[key]
          local filt = prev and (alpha * raw + (1 - alpha) * prev) or raw
          emaState[key] = filt
          wh[compName] = filt
        end
      end
    end
  end

  -- 4) Add the custom fields
  for fieldName, value in pairs(customFields) do
    latestReading[fieldName] = value
  end

  -- Store the latest readings for this State sensor in the extension. 
  -- This is used for sending back on the physics step.
  gtStateManager.cacheLatestReading(sensorId, latestReading)

  -- Update the number of physics steps for the GFX save.
  counterPhysicsSteps = counterPhysicsSteps + 1
  if counterPhysicsSteps >= numPhysicsStepsForGFXSave then
    -- Add the data to the readings array, for later retrieval.
    -- This is used for sending back on the graphics step.
    readings[readingIndex] = latestReading
    readingIndex = readingIndex + 1
    counterPhysicsSteps = 0
  end
end

--[[
Initializes a vehicle state sensor instance with barycentric positioning
on a triangular surface element and configures its operational parameters.

Parameters:
  data : table - Configuration table

Dependencies:
  - Requires global 'wheelRotators' and 'wheelIds' tables for wheel references
  - Relies on 'extensions.xlab_gtState' for state management
  - Assumes 'obj' exists in parent scope with vehicle methods
]]
local function init(data)
  -- Validate critical input parameters
  assert(
    data.nodeIndex1 and data.nodeIndex2 and data.nodeIndex3,
    'Missing node indices in sensor configuration'
  )
  assert(data.u and data.v, 'Missing barycentric coordinates in config')
  -- assert(
  --   (data.u + data.v) <= 1 + 1e-6,
  --   'Invalid barycentric coordinates (u + v must be <= 1, got ' .. data.u .. ', ' .. data.v .. ')'
  -- )

  -- Initialize extension integration
  gtStateManager = extensions.xlab_gtState

  -- Core sensor configuration
  sensorId = data.sensorId
  GFXUpdateTime = data.GFXUpdateTime or 0.033 -- Default to ~30Hz

  -- Node indices for triangular mounting surface
  nodeIndex1 = data.nodeIndex1
  nodeIndex2 = data.nodeIndex2
  nodeIndex3 = data.nodeIndex3

  -- Set the vehicle object
  currVeh = obj

  -- Masses of the three nodes
  m1 = currVeh:getNodeMass(nodeIndex1)
  m2 = currVeh:getNodeMass(nodeIndex2)
  m3 = currVeh:getNodeMass(nodeIndex3)

  -- Barycentric coordinates and interpolation weights
  b1 = data.u
  b2 = data.v
  b3 = 1.0 - b1 - b2
  -- Non-negative weights for mass interpolation
  w1 = max(0, b1)
  w2 = max(0, b2)
  w3 = max(0, b3)

  -- Spatial configuration
  signedProjDist = data.signedProjDist
  triangleSpaceForward = data.triangleSpaceForward
  triangleSpaceLeft = data.triangleSpaceLeft

  -- Operational flags
  isVisualised = data.isVisualised
  isUsingGravity = data.isUsingGravity

  -- Timing configuration
  physicsUpdateTime = data.physicsUpdateTime or 0.005 -- Default to 200Hz
  numPhysicsStepsForGFXSave = data.numPhysicsStepsForGFXSave

  -- Initialize timing state
  physicsTimer = 0.0
  timeSinceLastPoll = 0.0
  counterPhysicsSteps = 0

  -- Data collection buffers
  readings = {} -- Circular buffer for graphics system
  readingIndex = 1 -- Current write position in buffer

  -- Wheel system integration
  wheel_fr = wheelRotators[wheelIds['FR']]
  wheel_fl = wheelRotators[wheelIds['FL']]
  wheel_rr = wheelRotators[wheelIds['RR']]
  wheel_rl = wheelRotators[wheelIds['RL']]

  -- Engine and gearbox references
  engine = powertrain.getDevice('mainEngine')
  gearbox = powertrain.getDevice('gearbox')

  -- Let's make sure that they are not nil
  if engine == nil then log('E', logTag, 'Engine reference is nil') end
  if gearbox == nil then log('E', logTag, 'Gearbox reference is nil') end

  -- Debug initialization
  log(
    'I',
    logTag,
    string.format(
      'Initialized sensor %d | Nodes: %d,%d,%d | Update rates: Physics=%.1fkHz GFX=%.1fHz',
      sensorId,
      nodeIndex1,
      nodeIndex2,
      nodeIndex3,
      1 / physicsUpdateTime / 1000,
      1 / GFXUpdateTime
    )
  )
end

--[[
Resets the sensor's internal state by clearing accumulated readings and resetting timers.
Note:
    - Assumes that `GFXUpdateTime` is a valid, positive number.
    - Relies on `readings`, `readingIndex`, and `timeSinceLastPoll` being properly initialized.
]]
local function reset()
  -- Clear the readings buffer by assigning a new empty table
  readings = {}
  -- Reset the reading index to 1 to start populating from the beginning
  readingIndex = 1
  -- Ensure GFXUpdateTime is positive and adjust the polling timer accordingly
  timeSinceLastPoll = timeSinceLastPoll % max(GFXUpdateTime, 1e-30)

  emaState = {}
end

--[[
Retrieves comprehensive sensor data for external use, such as graphical updates or external systems.

Returns:
    table: A table containing the following fields:
        - isVisualised (bool): Indicates whether the sensor's visualization is enabled.
        - timeSinceLastPoll (number): Time elapsed since the last poll/update.
        - GFXUpdateTime (number): Interval (in seconds) between graphical updates.
        - currentPos (vec3): The sensor's current position in world space, calculated by adding
                                the sensor's local position to the vehicle's global position.
        - currentDir (vec3): The sensor's current orientation direction vector in vehicle space.
        - rawReadings (table): A table of raw sensor readings accumulated since the last graphics update.
]]
local function getSensorData()
  return {
    isVisualised = isVisualised,
    timeSinceLastPoll = timeSinceLastPoll,
    GFXUpdateTime = GFXUpdateTime,
    currentPos = sensorPos,
    currentDir = currentDir,
    rawReadings = readings,
  }
end

--[[
Retrieves the most recent sensor reading.

Returns:
    table: A table containing the latest sensor data, including time, orientation, velocity,
            acceleration, angular dynamics, position, orientation quaternion, wheel states,
            control inputs, desired inputs, and drive mode status, ..
]]
local function getLatest() return latestReading end

--[[
Increments the internal timer tracking the time since the last polling event.
This function should be called with the simulation delta time (dtSim) each physics step.

Parameters:
    dtSim (number): The time increment (in seconds) since the last simulation step.
]]
local function incrementTimer(dtSim) timeSinceLastPoll = timeSinceLastPoll + dtSim end

-- Public interface:
M.update = update
M.init = init
M.reset = reset
M.getSensorData = getSensorData
M.getLatest = getLatest
M.incrementTimer = incrementTimer
M.registerCustomField = registerCustomField
M.setCustomField = setCustomField

return M
