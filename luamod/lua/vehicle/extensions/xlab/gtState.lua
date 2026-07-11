--- Ground Truth State Module
-- Handles ground truth state sensors and GE poll handoff for vehicles.
-- driveStatus is sampled from electrics in the gtState controller physics step.
local logTag = 'GtState'
local M = {}

-- Module state variables
local gtStates = {} -- Collection of active Groundtruth state sensors
local latestReadings = {} -- Latest sensor readings (manager live path)
local sensorPollTimers = {} -- Timers to track time since last poll for each sensor

--- Updates ground truth state graphics step
-- @param dtSim Simulation delta time
-- @param sensorId ID of the sensor to update
local function updateGtStateGFXStep(dtSim, sensorId)
  local state = gtStates[sensorId]
  if not state then return end

  -- Check if it's time to poll this sensor based on its GFX update time.
  local gfxDt = state.data.GFXUpdateTime
  local t = sensorPollTimers[sensorId] + dtSim
  if t < gfxDt then
    sensorPollTimers[sensorId] = t
    return
  end
  sensorPollTimers[sensorId] = t - gfxDt

  -- Get the latest data from the controller.
  local controller = state.controller
  local sensorData = controller.getSensorData() -- called only at cadence

  -- Send the latest sensor readings from vlua to ge lua.
  local rawReadingsData = { sensorId = sensorId, reading = sensorData.rawReadings }
  obj:queueGameEngineLua(
    string.format(
      'xlab_sensors.updateGtStateLastReadings(%q)',
      lpack.encode(rawReadingsData)
    )
  )

  -- Draw this sensor, if requested.
  if state.data.isVisualised == true then
    obj.debugDrawProxy:drawSphere(0.05, sensorData.currentPos, color(0, 255, 0, 255))
  end
end

--- Creates a new ground truth state sensor
-- @param data Sensor configuration data
local function create(data)
  -- Create a controller instance for this GtState sensor
  local decodedData = lpack.decode(data)

  -- Controller data
  local controllerData = {
    sensorId = decodedData.sensorId,
    GFXUpdateTime = decodedData.GFXUpdateTime,
    physicsUpdateTime = decodedData.physicsUpdateTime,
    numPhysicsStepsForGFXSave = decodedData.numPhysicsStepsForGFXSave,
    nodeIndex1 = decodedData.nodeIndex1,
    nodeIndex2 = decodedData.nodeIndex2,
    nodeIndex3 = decodedData.nodeIndex3,
    u = decodedData.u,
    v = decodedData.v,
    signedProjDist = decodedData.signedProjDist,
    triangleSpaceForward = decodedData.triangleSpaceForward,
    triangleSpaceLeft = decodedData.triangleSpaceLeft,
    isVisualised = decodedData.isVisualised,
    accel_tau_s = decodedData.accel_tau_s,
    gyro_tau_s = decodedData.gyro_tau_s,
    vel_tau_s = decodedData.vel_tau_s,
    wheel_angvel_tau_s = decodedData.wheel_angvel_tau_s,
    debug_raw = decodedData.debug_raw,
    torque_map = decodedData.torque_map,
  }

  gtStates[decodedData.sensorId] = {
    data = controllerData,
    controller = controller.loadControllerExternal(
      'xlab/gtState',
      'gtState' .. decodedData.sensorId,
      controllerData
    ),
  }

  -- Store the timer for this sensor to track time since last poll.
  sensorPollTimers[decodedData.sensorId] = 0

  -- Log some info about the vehicle setup on creation of the first sensor.
  log('I', logTag, 'Creating GtState sensor with ID: ' .. tostring(decodedData.sensorId))
  log('I', logTag, 'Vehicle ID: ' .. tostring(objectId))
  log('I', logTag, 'numPhysicsStepsForGFXSave: ' .. tostring(controllerData.numPhysicsStepsForGFXSave))
end

--- Caches the latest sensor reading
-- @param sensorId ID of the sensor
-- @param latestReading The reading to cache
local function cacheLatestReading(sensorId, latestReading)
  if sensorId ~= nil then latestReadings[sensorId] = latestReading end
end

--- Gets the latest ground truth state reading
-- @param sensorId ID of the sensor
-- @return Latest reading for the sensor
local function geGtStateReading(sensorId) return latestReadings[sensorId] end

--- Removes a ground truth state sensor
-- @param sensorId ID of the sensor to remove
local function remove(sensorId)
  controller.unloadControllerExternal('GtState' .. sensorId)
  gtStates[sensorId] = nil
  sensorPollTimers[sensorId] = nil
end

--- Gets the latest sensor data
-- @param sensorId ID of the sensor
-- @return Latest sensor data
local function getLatest(sensorId) return gtStates[sensorId].controller.getLatest() end

--- Updates ground truth state graphics
-- @param dtSim Simulation delta time
local function updateGFX(dtSim)
  for sensorId, _ in pairs(gtStates) do
    updateGtStateGFXStep(dtSim, sensorId)
  end
end

--- Handles vehicle destruction cleanup
-- @param vid Vehicle ID being destroyed
local function onVehicleDestroyed(vid)
  for sensorId, _ in pairs(gtStates) do
    if vid == objectId then
      remove(sensorId)
      gtStates[sensorId] = nil
    end
  end
end

local function getGtStateController(sensorId)
  if gtStates[sensorId] then
    return gtStates[sensorId].controller
  else
    log('E', logTag, 'GtState controller not found for sensor ID: ' .. tostring(sensorId))
    return nil
  end
end

-- Public interface:
M.create = create
M.remove = remove
M.cacheLatestReading = cacheLatestReading
M.geGtStateReading = geGtStateReading
M.getLatest = getLatest
M.updateGFX = updateGFX
M.onVehicleDestroyed = onVehicleDestroyed
M.getGtStateController = getGtStateController

return M

