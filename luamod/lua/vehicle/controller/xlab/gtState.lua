-- Vehicle State Sensor Module
-- This module manages a vehicle state sensor, handling physics calculations,
-- orientation updates, wheel management, input processing, and data buffering
-- for graphical representation.

local M = {}

-- Import necessary math functions for performance
local sqrt, abs, acos, atan2, max, min = math.sqrt, math.abs, math.acos, math.atan2, math.max, math.min
local exp = math.exp
local pi = math.pi
local constants = { rpmToAV = 0.104719755, avToRPM = 9.549296596425384 }

local function sign(x) return max(min((x * 1e200) * 1e200, 1), -1) end

local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

-- Logging tag for debugging purposes
local logTag = 'GtState'

-- Will store custom fields added by controllers or other modules.
local customFields = {}

-- Optional NN torque estimator (enabled if init(data).torqueNN is provided).
local nn = nil
local torqueNNModel = nil
local torqueNNOutputScaling = 1.0
local torqueNNFieldName = 'estimated_torque'

-- Reference to the global state manager extension
local gtStateManager = nil

-- Filtering (dt-aware). Only applied to accel and angular velocity.
-- Tune these to match your expected sensor bandwidth/latency.
local accelTauS = 0.01 -- seconds
local gyroTauS = 0.005 -- seconds

local function alphaFromTau(dt, tau)
  if tau <= 0 then return 1 end
  return 1 - exp(-dt / tau)
end

local function emaVec3Step(prev, raw, alpha)
  prev.x = prev.x + (raw.x - prev.x) * alpha
  prev.y = prev.y + (raw.y - prev.y) * alpha
  prev.z = prev.z + (raw.z - prev.z) * alpha
  return prev
end

local filterState = {
  initialized = false,
  acc1 = vec3(),
  acc2 = vec3(),
  gyro1 = vec3(),
  gyro2 = vec3(),
}

local function writeVec3Table(dst, v)
  dst[1] = v.x
  dst[2] = v.y
  dst[3] = v.z
end

local function writeQuatTable(dst, q)
  dst[1] = q[1]
  dst[2] = q[2]
  dst[3] = q[3]
  dst[4] = q[4]
end

local function cloneVec3Table(src)
  return { src[1], src[2], src[3] }
end

local function cloneQuatTable(src)
  return { src[1], src[2], src[3], src[4] }
end

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
  accelRaw = { 0, 0, 0 },
  accel = { 0, 0, 0 },
  angVelRaw = { 0, 0, 0 },
  angVel = { 0, 0, 0 },
  angAccelRaw = { 0, 0, 0 },
  pos = { 0, 0, 0 },
  vel = { 0, 0, 0 },
  quat = { 0, 0, 0, 1 },
  wheelFR = {},
  wheelFL = {},
  wheelRR = {},
  wheelRL = {},
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

-- Reused wheel info tables (avoid per-physics-step allocations)
local wheelFRInfo = { speed = 0, angVelB = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0, angleLegacy = 0, angleAtan2 = 0 }
local wheelFLInfo = { speed = 0, angVelB = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0, angleLegacy = 0, angleAtan2 = 0 }
local wheelRRInfo = { speed = 0, angVelB = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0, angleLegacy = 0, angleAtan2 = 0 }
local wheelRLInfo = { speed = 0, angVelB = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0, angleLegacy = 0, angleAtan2 = 0 }

local function fillWheelInfos(dst, mWheel)
  dst.speed = mWheel.wheelSpeed
  dst.angVelB = mWheel.angularVelocityBrakeCouple * mWheel.wheelDir
  dst.angVel = mWheel.angularVelocity * mWheel.wheelDir
  dst.brakeTorque = abs(mWheel.coreData.brakeTorqueApplied) - mWheel.frictionTorque
  dst.propTorque = mWheel.propulsionTorque * mWheel.wheelDir
  dst.downForce = mWheel.downForce or 0
  return dst
end

-- Legacy "road wheel angle" as used by BeamNG.Tech (acos(cos) + steering-input sign)
local function wheelAngleLegacyRad(nodeA, nodeB, signSteer)
  local cosAng = clamp(obj:nodeVecPlanarCosRightForward(nodeA, nodeB), -1, 1)
  return acos(cosAng) * -signSteer
end

-- Signed wheel heading angle via atan2, independent of steering input.
-- Computed in the sensor's local plane (currentDir/worldLeft), using worldThird as plane normal.
local function wheelAngleAtan2Rad(nodeA, nodeB)
  -- local axis = currVeh:getNodePosition(nodeB) - currVeh:getNodePosition(nodeA)
  -- axis = axis - worldThird * axis:dot(worldThird) -- ignore camber vs sensor plane
  -- if axis:squaredLength() < 1e-18 then return 0 end
  -- axis = axis:normalized()

  -- local roll = axis:cross(worldThird) -- rolling direction (upRef = worldThird)
  -- roll = roll - worldThird * roll:dot(worldThird)
  -- if roll:squaredLength() < 1e-18 then return 0 end
  -- roll = roll:normalized()

  -- if roll:dot(currentDir) < 0 then roll = -roll end -- remove node-order flips
  -- return atan2(roll:dot(worldLeft), roll:dot(currentDir))
  local axis = currVeh:getNodePosition(nodeB) - currVeh:getNodePosition(nodeA)
  local roll = axis:cross(worldThird) -- wheel rolling direction
  if roll:squaredLength() < 1e-12 then return 0 end
  if roll:dot(currentDir) < 0 then roll = -roll end
  return atan2(roll:dot(worldLeft), roll:dot(currentDir))
end

local function cloneWheelInfo(src)
  return {
    speed = src.speed,
    angVelB = src.angVelB,
    angVel = src.angVel,
    brakeTorque = src.brakeTorque,
    propTorque = src.propTorque,
    downForce = src.downForce,
    angle = src.angle,
    angleLegacy = src.angleLegacy,
    angleAtan2 = src.angleAtan2,
  }
end

local function cloneReading(src)
  local dst = {
    time = src.time,
    dirX = cloneVec3Table(src.dirX),
    dirY = cloneVec3Table(src.dirY),
    vel = cloneVec3Table(src.vel),
    accelRaw = cloneVec3Table(src.accelRaw),
    accel = cloneVec3Table(src.accel),
    angVelRaw = cloneVec3Table(src.angVelRaw),
    angVel = cloneVec3Table(src.angVel),
    angAccelRaw = cloneVec3Table(src.angAccelRaw),
    pos = cloneVec3Table(src.pos),
    quat = cloneQuatTable(src.quat),
    wheelFR = cloneWheelInfo(src.wheelFR),
    wheelFL = cloneWheelInfo(src.wheelFL),
    wheelRR = cloneWheelInfo(src.wheelRR),
    wheelRL = cloneWheelInfo(src.wheelRL),
    steering = src.steering,
    throttle = src.throttle,
    brake = src.brake,
    clutch = src.clutch,
    pbrake = src.pbrake,
    steeringInput = src.steeringInput,
    throttleInput = src.throttleInput,
    brakeInput = src.brakeInput,
    clutchInput = src.clutchInput,
    driveStatus = src.driveStatus,
    engineLoad = src.engineLoad,
    engineTorque = src.engineTorque,
    RPM = src.RPM,
    flywheelTorque = src.flywheelTorque,
    turboBoost = src.turboBoost,
    superchargerBoost = src.superchargerBoost,
    throttleValve = src.throttleValve,
    gearboxTorque = src.gearboxTorque,
    gearRatio = src.gearRatio,
    gearIndex = src.gearIndex,
  }

  for fieldName in pairs(customFields) do
    dst[fieldName] = src[fieldName]
  end

  return dst
end

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

-- Physics step update for this sensor instance.
local function update(dtSim)
  -- Manage the update timer. Cycle it to avoid long-term drift.
  physicsTimer = physicsTimer + dtSim
  if physicsTimer < physicsUpdateTime then return end
  local trueDt = physicsTimer
  physicsTimer = physicsTimer - physicsUpdateTime

  local accelAlpha = alphaFromTau(trueDt, accelTauS)
  local gyroAlpha = alphaFromTau(trueDt, gyroTauS)

  -- Compute the current position of the nodes defining the sensor wrt ref node.
  local node1 = currVeh:getNodePosition(nodeIndex1)
  local node2 = currVeh:getNodePosition(nodeIndex2)
  local node3 = currVeh:getNodePosition(nodeIndex3)

  -- Relvant edge vectors and normal vector of the triangle.
  local edge1, edge2 = node2 - node1, node3 - node1
  local edge1Norm, edge2Norm = edge1:normalized(), edge2:normalized()
  local normal = edge1Norm:cross(edge2Norm):normalized()

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

  -- Compute the total acceleration vector at the sensor position
  local accelWorld = aCenter + (curlAcc:cross(r) + divAcc * r) * invDenom
  if isUsingGravity then accelWorld = accelWorld + currVeh:getGravityVector() end

  -- Compute the angular velocity and angular acceleration.
  local angVel = curlVel * invDenom
  local angAccel = curlAcc * invDenom

  -- Transform the quantities in the local frame.
  local accelLocalRaw =
    vec3(accelWorld:dot(currentDir), accelWorld:dot(worldLeft), accelWorld:dot(worldThird))

  local velLocal = vec3(vCenter:dot(currentDir), vCenter:dot(worldLeft), vCenter:dot(worldThird))

  local angVelLocalRaw = vec3(angVel:dot(currentDir), angVel:dot(worldLeft), angVel:dot(worldThird))

  local angAccelLocalRaw = vec3(angAccel:dot(currentDir), angAccel:dot(worldLeft), angAccel:dot(worldThird))

  -- Filter accel and gyro only (two cascaded dt-aware EMA passes).
  if not filterState.initialized then
    filterState.acc1:set(accelLocalRaw)
    filterState.acc2:set(accelLocalRaw)
    filterState.gyro1:set(angVelLocalRaw)
    filterState.gyro2:set(angVelLocalRaw)
    filterState.initialized = true
  end

  local accelPass1 = emaVec3Step(filterState.acc1, accelLocalRaw, accelAlpha)
  local accelLocal = emaVec3Step(filterState.acc2, accelPass1, accelAlpha)
  local gyroPass1 = emaVec3Step(filterState.gyro1, angVelLocalRaw, gyroAlpha)
  local angVelLocal = emaVec3Step(filterState.gyro2, gyroPass1, gyroAlpha)
  -- -----------------------------------------------------------------------

  -- ------------------------------------------------------------------------
  --                     Quaternion/Orientation Calculation                --
  -- -----------------------------------------------------------------------

  -- Compute the orientation of the sensor.
  local quatEst = getQuaternionFromDir(currentDir, worldLeft, worldThird) -- qx, qy, qz, qw

  -- ----------------------------------------------------------------------
  --                     WheelAngle Calculation                          --
  -- ----------------------------------------------------------------------
  local signSteering = sign(electrics.values.steering_input)

  -- front right
  fillWheelInfos(wheelFRInfo, wheel_fr)
  wheelFRInfo.angleLegacy = wheelAngleLegacyRad(wheel_fr.node1, wheel_fr.node2, signSteering)
  wheelFRInfo.angleAtan2 = wheelAngleAtan2Rad(wheel_fr.node1, wheel_fr.node2)
  wheelFRInfo.angle = wheelFRInfo.angleLegacy

  -- front left (note swapped order if needed)
  fillWheelInfos(wheelFLInfo, wheel_fl)
  wheelFLInfo.angleLegacy = wheelAngleLegacyRad(wheel_fl.node2, wheel_fl.node1, signSteering)
  wheelFLInfo.angleAtan2 = wheelAngleAtan2Rad(wheel_fl.node2, wheel_fl.node1)
  wheelFLInfo.angle = wheelFLInfo.angleLegacy

  -- rear right
  fillWheelInfos(wheelRRInfo, wheel_rr)
  wheelRRInfo.angleLegacy = wheelAngleLegacyRad(wheel_rr.node1, wheel_rr.node2, signSteering)
  wheelRRInfo.angleAtan2 = wheelAngleAtan2Rad(wheel_rr.node1, wheel_rr.node2)
  wheelRRInfo.angle = wheelRRInfo.angleLegacy

  -- rear left
  fillWheelInfos(wheelRLInfo, wheel_rl)
  wheelRLInfo.angleLegacy = wheelAngleLegacyRad(wheel_rl.node2, wheel_rl.node1, signSteering)
  wheelRLInfo.angleAtan2 = wheelAngleAtan2Rad(wheel_rl.node2, wheel_rl.node1)
  wheelRLInfo.angle = wheelRLInfo.angleLegacy
  --  -------------------------------------------------------

  -- These inputs are updated at a lower frequency than the physics steps.
  local elecVals = electrics.values
  -- Extract the drive status from the state manager.
  local driveModeStatus = gtStateManager.getDriveModeStatus()

  -- Gather the latest reading data (in-place updates to reduce allocations).
  latestReading.time = currVeh:getSimTime()
  writeVec3Table(latestReading.dirX, currentDir)
  writeVec3Table(latestReading.dirY, worldLeft)
  writeVec3Table(latestReading.vel, velLocal)
  writeVec3Table(latestReading.accelRaw, accelLocalRaw)
  writeVec3Table(latestReading.accel, accelLocal)
  writeVec3Table(latestReading.angVelRaw, angVelLocalRaw)
  writeVec3Table(latestReading.angVel, angVelLocal)
  writeVec3Table(latestReading.angAccelRaw, angAccelLocalRaw)
  writeVec3Table(latestReading.pos, sensorPos)
  writeQuatTable(latestReading.quat, quatEst)
  latestReading.wheelFR = wheelFRInfo
  latestReading.wheelFL = wheelFLInfo
  latestReading.wheelRR = wheelRRInfo
  latestReading.wheelRL = wheelRLInfo
  -- Backward-compatible aliases (older consumers)
  latestReading.wheel_fr = wheelFRInfo
  latestReading.wheel_fl = wheelFLInfo
  latestReading.wheel_rr = wheelRRInfo
  latestReading.wheel_rl = wheelRLInfo

  latestReading.steering = elecVals.steering
  latestReading.throttle = elecVals.throttle
  latestReading.brake = elecVals.brake
  latestReading.clutch = elecVals.clutch
  latestReading.pbrake = elecVals.parkingbrake
  latestReading.steeringInput = elecVals.steering_input
  latestReading.throttleInput = elecVals.throttle_input
  latestReading.brakeInput = elecVals.brake_input
  latestReading.clutchInput = elecVals.clutch_input

  -- Relevant vehicle mode / state data.
  latestReading.driveStatus = driveModeStatus

  -- Engine relevant information
  latestReading.engineLoad = engine and (engine.isDisabled and 0 or engine.instantEngineLoad) or 0
  latestReading.engineTorque = engine and engine.combustionTorque or 0
  latestReading.RPM = engine and (engine.outputAV1 * constants.avToRPM) or 0
  latestReading.flywheelTorque = engine and engine.outputTorque1 or 0
  latestReading.turboBoost = elecVals.turboBoost or -1
  latestReading.superchargerBoost = elecVals.superchargerBoost or -1
  latestReading.throttleValve = engine and engine.throttle

  -- Gearbox relevant information
  latestReading.gearboxTorque = gearbox and gearbox.outputTorque1 or 0
  latestReading.gearRatio = gearbox and gearbox.gearRatio or 0
  latestReading.gearIndex = elecVals.gearIndex

  -- Optional: estimate torque from current state using NN.
  -- Inputs are intentionally hardcoded (like controller_nn_*):
  --   [engine_speed_rads, boost_pressure, throttle, rear_wheelspeed_ms]
  if torqueNNModel and nn then
    local rear_wheelspeed_ms = 0.5 * (wheelRRInfo.speed+ wheelRLInfo.speed)
    local engine_speed_rads = latestReading.RPM * constants.rpmToAV
    local boost_pressure = latestReading.turboBoost
    local throttle = latestReading.throttle

    local out = nn.run( torqueNNModel, {
      engine_speed_rads,
      boost_pressure,
      throttle,
      rear_wheelspeed_ms,
    })
    setCustomField(torqueNNFieldName, out[1] * torqueNNOutputScaling)
  end

  -- Add the custom fields
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
    readings[readingIndex] = cloneReading(latestReading)
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

  -- Wire wheel info tables into the latestReading (stable references)
  latestReading.wheelFR = wheelFRInfo
  latestReading.wheelFL = wheelFLInfo
  latestReading.wheelRR = wheelRRInfo
  latestReading.wheelRL = wheelRLInfo
  -- Backward-compatible aliases
  latestReading.wheel_fr = wheelFRInfo
  latestReading.wheel_fl = wheelFLInfo
  latestReading.wheel_rr = wheelRRInfo
  latestReading.wheel_rl = wheelRLInfo

  -- Engine and gearbox references
  engine = powertrain.getDevice('mainEngine')
  gearbox = powertrain.getDevice('gearbox')

  -- Let's make sure that they are not nil
  if engine == nil then log('E', logTag, 'Engine reference is nil') end
  if gearbox == nil then log('E', logTag, 'Gearbox reference is nil') end

  -- Optional torque estimation NN setup.
  torqueNNModel = nil
  local torqueNNCfg = data.torqueNN
  if torqueNNCfg ~= nil then
    torqueNNFieldName = torqueNNCfg.fieldName or torqueNNCfg.outputFieldName or 'estimated_torque'
    torqueNNOutputScaling = tonumber(torqueNNCfg.outputScaling or torqueNNCfg.output_scaling) or 1.0

    if customFields[torqueNNFieldName] == nil then
      registerCustomField(torqueNNFieldName, 0)
    end

    local okReq, nnLib = pcall(require, 'lua/vehicle/controller/xlab/lib/nn')
    if not okReq or not nnLib then
      log('E', logTag, 'Torque NN disabled: failed to require nn library')
    else
      nn = nnLib
      local okInit, errInit = pcall(nn.init)
      if not okInit then
        log('E', logTag, 'Torque NN disabled: nn.init failed: ' .. tostring(errInit))
      else
        local modelPath = torqueNNCfg.modelPath
        if not modelPath then
          local modelName = torqueNNCfg.modelName or torqueNNCfg.model
          if modelName then
            modelPath = 'lua/vehicle/controller/xlab/models/' .. modelName
          end
        end

        if not modelPath then
          log('E', logTag, 'Torque NN disabled: no modelPath/modelName provided in torqueNN config')
        else
          local okLoad, modelOrErr = pcall(nn.loadModel, modelPath)
          if not okLoad or not modelOrErr then
            log('E', logTag, 'Torque NN disabled: failed to load model: ' .. tostring(modelOrErr))
          else
            torqueNNModel = modelOrErr
            log('I', logTag, 'Torque NN enabled: model=' .. tostring(modelPath)
              .. ' field=' .. tostring(torqueNNFieldName)
              .. ' outputScaling=' .. tostring(torqueNNOutputScaling))
          end
        end
      end
    end
  end

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

  filterState.initialized = false
end

local function stop()
  if nn and torqueNNModel then
    pcall(nn.freeModel, torqueNNModel)
  end
  torqueNNModel = nil
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
M.stop = stop
M.getSensorData = getSensorData
M.getLatest = getLatest
M.incrementTimer = incrementTimer
M.registerCustomField = registerCustomField
M.setCustomField = setCustomField

return M