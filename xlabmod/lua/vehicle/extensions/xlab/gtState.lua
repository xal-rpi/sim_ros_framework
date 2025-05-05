--- Ground Truth State Module
-- Handles ground truth state sensors and differential status for vehicles
local logTag = 'GtState'
local M = {}

-- Module state variables
local gtStates = {} -- Collection of active Groundtruth state sensors
local latestReadings = {} -- Latest sensor readings
local is4WDVehicle = false -- 4WD status flag
local frontDiff = nil -- Front differential device
local rearDiff = nil -- Rear differential device
local driveModeStatus = {} -- Drive mode status values

--- Gets the current drive mode status
-- @return table containing drive mode status values
local function getDriveModeStatus() return driveModeStatus end

--- Updates the drive mode status
local function updateDriveModeStatus()
  local elecVals = electrics.values
  driveModeStatus.esc = elecVals.esc
  driveModeStatus.abs = elecVals.abs
  driveModeStatus.tcs = elecVals.tcs
  driveModeStatus.engineRunning = elecVals.engineRunning
  driveModeStatus.isRealisticDrive = elecVals.gearboxMode == 'realistic' and 1 or 0
  if is4WDVehicle then
    driveModeStatus.mode4WD = elecVals.mode4WD
    driveModeStatus.modeRangeBox = elecVals.modeRangeBox
  end
  if frontDiff then driveModeStatus.isFrontDiffLocked = frontDiff.mode == 'locked' and 1 or 0 end
  if rearDiff then driveModeStatus.isRearDiffLocked = rearDiff.mode == 'locked' and 1 or 0 end
end

--- Updates ground truth state graphics step
-- @param dtSim Simulation delta time
-- @param sensorId ID of the sensor to update
local function updateGtStateGFXStep(dtSim, sensorId)
  -- Get the latest data from the controller.
  local controller = gtStates[sensorId].controller
  local data = controller.getSensorData()
  -- Draw this sensor, if requested.
  if data.isVisualised == true then
    obj.debugDrawProxy:drawSphere(0.05, data.currentPos, color(0, 255, 0, 255))
  end
  -- If we are not ready to poll this sensor, then increment the timer and leave.
  if data.timeSinceLastPoll < data.GFXUpdateTime then
    controller.incrementTimer(dtSim)
    return
  end
  -- Send the latest sensor readings from vlua to ge lua.
  local rawReadingsData = { sensorId = sensorId, reading = data.rawReadings }
  obj:queueGameEngineLua(
    string.format('xlab_sensors.updateGtStateLastReadings(%q)', lpack.encode(rawReadingsData))
  )
  -- Reset the raw readings table, now that the GFX update step has been performed.
  controller.reset()
end

--- Sets up and configures the vehicle's differentials
-- Extracts and validates front and rear differential devices
-- Logs differential status and availability of modes
local function setupDifferentials()
  -- Extract the front and rear diffs, if the vehicle is a 4WD vehicle.
  frontDiff = powertrain.getDevice('differential_F')
  rearDiff = powertrain.getDevice('differential_R')
  if frontDiff == nil then
    log('I', logTag, 'No front differential found.')
  else
    local modesFDiff = frontDiff.availableModes
    if #modesFDiff <= 1 then
      log('I', logTag, 'No ability to toggle front differential state.')
      frontDiff = nil
    end
  end
  if rearDiff == nil then
    log('I', logTag, 'No rear differential found.')
  else
    local modesRDiff = rearDiff.availableModes
    if #modesRDiff <= 1 then
      log('I', logTag, 'No ability to toggle rear differential state.')
      rearDiff = nil
    end
  end
end

--[[
    Sets up the vehicle as a 4WD vehicle.
    If the vehicle is a 4WD vehicle, then set the flag to true.
]]
local function setup4WDVehicle()
  -- If the vehicle is a 4WD vehicle, then set the flag to true.
  local ctrl4wds = controller.getControllersByType('4wd')
  if #ctrl4wds == 0 then
    log('I', logTag, 'No 4wd controller found.')
    is4WDVehicle = false
  else
    log('I', logTag, '4wd controller found.')
    is4WDVehicle = true
  end
end

--- Creates a new ground truth state sensor
-- @param data Sensor configuration data
local function create(data)
  -- Create a controller instance for this GtState sensor
  local decodedData = lpack.decode(data)
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
    isUsingGravity = decodedData.isUsingGravity,
  }

  gtStates[decodedData.sensorId] = {
    data = controllerData,
    controller = controller.loadControllerExternal(
      'xlab/gtState',
      'gtState' .. decodedData.sensorId,
      controllerData
    ),
  }

  setup4WDVehicle()
  setupDifferentials()
  updateDriveModeStatus()
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
end

--- Gets the latest sensor data
-- @param sensorId ID of the sensor
-- @return Latest sensor data
local function getLatest(sensorId) return gtStates[sensorId].controller.getLatest() end

--- Updates ground truth state graphics
-- @param dtSim Simulation delta time
local function updateGFX(dtSim)
  -- If the vehicle is a 4WD vehicle, check the front and rear diffs for lockup.
  updateDriveModeStatus()
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

-- Public interface:
M.create = create
M.remove = remove
M.cacheLatestReading = cacheLatestReading
M.geGtStateReading = geGtStateReading
M.getLatest = getLatest
M.updateGFX = updateGFX
M.onVehicleDestroyed = onVehicleDestroyed
M.getDriveModeStatus = getDriveModeStatus

return M

