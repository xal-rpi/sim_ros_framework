local M = {}

-- Vlua sensor readings data.

-- The most-recently-read GtState data (this is a table).
local gtStateLastRawReadings = {}

local function createGtState(vid, args)
  -- Set optional parameters to defaults if they are not provided by the user.
  if args.pos == nil then args.pos = vec3(0, 0, 3) end
  if args.dir == nil then args.dir = vec3(0, -1, 0) end
  if args.left == nil then args.left = vec3(1, 0, 0) end
  if args.GFXUpdateTime == nil then args.GFXUpdateTime = 0.1 end
  if args.isUsingGravity == nil then args.isUsingGravity = false end
  if args.isVisualised == nil then args.isVisualised = true end
  if args.isSnappingDesired == nil then args.isSnappingDesired = false end
  if args.isForceInsideTriangle == nil then args.isForceInsideTriangle = false end
  if args.isAllowWheelNodes == nil then args.isAllowWheelNodes = false end
  if args.physicsUpdateTime == nil then args.physicsUpdateTime = 0.01 end
  if args.isDirWorldSpace == nil then args.isDirWorldSpace = false end
  if args.numPhysicsStepsForGFXSave == nil then args.numPhysicsStepsForGFXSave = 1 end

  -- we need to flip the up direction vector to get the orientation
  -- correct when attaching the sensor.
  -- TODO: Need to make sure this makes sense for every beamng.tech release.
  -- Still don't understand the way the sennsormanagers works.
  args.left = -args.left

  -- Attach the sensor to the vehicle.
  local sensorId = Research.SensorManager.getNewSensorId()
  Research.SensorMatrixManager.attachSensor(
    sensorId,
    args.pos,
    args.dir,
    args.left,
    vid,
    false,
    args.isSnappingDesired,
    args.isForceInsideTriangle,
    args.isAllowWheelNodes,
    args.isDirWorldSpace
  )
  local attachData = Research.SensorMatrixManager.getAttachData(sensorId)

  -- Create the GtState in vlua.
  local data = {
    sensorId = sensorId,
    GFXUpdateTime = args.GFXUpdateTime,
    physicsUpdateTime = args.physicsUpdateTime,
    numPhysicsStepsForGFXSave = args.numPhysicsStepsForGFXSave,
    isUsingGravity = args.isUsingGravity,
    nodeIndex1 = attachData['nodeIndex1'],
    nodeIndex2 = attachData['nodeIndex2'],
    nodeIndex3 = attachData['nodeIndex3'],
    u = attachData['u'],
    v = attachData['v'],
    signedProjDist = attachData['signedProjDist'],
    triangleSpaceForward = attachData['triangleSpaceForward'],
    triangleSpaceLeft = attachData['triangleSpaceUp'],
    isVisualised = args.isVisualised,
    torqueNN = args.torqueNN,
  }

  -- If torque NN is enabled, prepare the native NN shared lib path mailbox.
  -- This mirrors the logic in ge/extensions/xlab/xlabCore.lua for nn_ controllers.
  if args.torqueNN ~= nil then
    assert(
      not Engine.Sandbox.Lua.isEnabled(),
      'Torque NN can only run when the Lua security sandbox is disabled. '
        .. "You will have to restart BeamNG with the '-disable-sandbox' argument."
    )

    local mod_libpath = 'lua/vehicle/controller/xlab/lib/libnn.so'
    local fs_libpath = 'tmp/libnn.so'
    if jit and jit.os then
      if jit.os == 'Windows' then
        mod_libpath = 'lua/vehicle/controller/xlab/lib/libnn.dll'
        fs_libpath = 'tmp/libnn.dll'
      end
    end
    copyfile(mod_libpath, fs_libpath)
    be:sendToMailbox('libnnPath', FS:virtual2Native(fs_libpath))
    log('I', 'gtState', 'Torque NN enabled; using ' .. fs_libpath)
  end

  -- Let's log node indices and barycentric coordinates.
  log(
    'I',
    'gtState',
    string.format(
      'Attaching GtState sensor %d to vehicle %d with nodes (%d, %d, %d) and barycentric coords (u=%.3f, v=%.3f), signedProjDist=%.3f',
      sensorId,
      vid,
      attachData['nodeIndex1'],
      attachData['nodeIndex2'],
      attachData['nodeIndex3'],
      attachData['u'],
      attachData['v'],
      attachData['signedProjDist']
    )
  )

  -- Serialize the data and send it to the vehicle in vlua.
  local serializedData = string.format('extensions.xlab_gtState.create(%q)', lpack.encode(data))
  be:queueObjectLua(vid, serializedData)

  -- Compute maximum buffer size from parameters.
  local maxSize = math.ceil(args.GFXUpdateTime / args.physicsUpdateTime) * 5
  -- Initialize the circular buffer structure.
  gtStateLastRawReadings[sensorId] = {
    buffer = {},
    head = 0,
    maxSize = maxSize,
  }
  -- Log that the sensor was created, with infos about the sensor similar
  log(
    'I',
    'gtState',
    'Created GtState sensor '
      .. sensorId
      .. ' on vehicle '
      .. vid
      .. ' with buffer size '
      .. maxSize
  )
  return sensorId
end

local function removeGtState(vid, sensorId)
  local vehicleId = scenetree.findObject(vid):getID()
  be:queueObjectLua(vehicleId, 'extensions.xlab_gtState.remove(' .. sensorId .. ')')
  gtStateLastRawReadings[sensorId] = nil
end

local function getGtStateReadings(sensorId)
  local sensor = gtStateLastRawReadings[sensorId]
  if sensor == nil then return {} end
  -- Retrieve readings in insertion order.
  local outData = {}
  if sensor.head < sensor.maxSize or #sensor.buffer < sensor.maxSize then
    for i = 1, sensor.head do
      outData[#outData + 1] = sensor.buffer[i]
    end
  else
    for i = sensor.head + 1, sensor.maxSize do
      outData[#outData + 1] = sensor.buffer[i]
    end
    for i = 1, sensor.head do
      outData[#outData + 1] = sensor.buffer[i]
    end
  end
  -- Reset the circular buffer.
  sensor.buffer = {}
  sensor.head = 0
  return outData
end

local function updateGtStateLastReadings(data)
  local newReadings = lpack.decode(data)
  local sensor = gtStateLastRawReadings[newReadings.sensorId]
  if sensor == nil then return end
  -- Insert each new reading into the circular buffer.
  for _, v in pairs(newReadings.reading) do
    sensor.head = (sensor.head % sensor.maxSize) + 1
    sensor.buffer[sensor.head] = v
    -- if sensor.head == sensor.maxSize-1 then
    --   -- log that the buffer is full
    --   log('I', 'gtState', 'Buffer full for sensor ' .. newReadings.sensorId)
    -- end
  end
end

local function removeAllSensorsFromVehicle(vid) Research.SensorManager.removeSensorByVid(vid) end
-- local function onUpdate(dtReal, dtSim, dtRaw)
--     -- for sensorId, _ in pairs(visualisedUltrasonicSensors) do
--     --     visualiseUltrasonicSensor(sensorId, dtSim)
--     -- end
-- end

-- local function onDeserialized(data)
--     -- if Research then
--     --     Research.GpuRequestManager.reset()
--     -- end
-- end

-- local function onVehicleDestroyed(vid)
--   removeAllSensorsFromVehicle(vid)
-- end

-- Public interface:

-- -- General sensor functions.
-- M.doesSensorExist                           = doesSensorExist
-- M.removeSensor                              = removeSensor
-- M.removeAllSensorsFromVehicle               = removeAllSensorsFromVehicle

-- Advanced GtState-specific sensor functions.
M.createGtState = createGtState
M.removeGtState = removeGtState
M.getGtStateReadings = getGtStateReadings
M.updateGtStateLastReadings = updateGtStateLastReadings

-- -- Functions triggered by hooks.
-- M.onUpdate                                  = onUpdate
-- M.onDeserialized                            = onDeserialized
-- M.onVehicleDestroyed                        = onVehicleDestroyed

return M
