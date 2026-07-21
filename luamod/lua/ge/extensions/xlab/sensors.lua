local M = {}

-- GE-side gtState registry: attach sensor matrix, forward init to vlua, buffer GE polls.
local gtStateLastRawReadings = {}

local function createGtState(vid, args)
  if args.pos == nil then args.pos = vec3(0, 0, 3) end
  if args.dir == nil then args.dir = vec3(0, -1, 0) end
  if args.left == nil then args.left = vec3(1, 0, 0) end
  if args.GFXUpdateTime == nil then args.GFXUpdateTime = 0.1 end
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
    nodeIndex1 = attachData['nodeIndex1'],
    nodeIndex2 = attachData['nodeIndex2'],
    nodeIndex3 = attachData['nodeIndex3'],
    u = attachData['u'],
    v = attachData['v'],
    signedProjDist = attachData['signedProjDist'],
    triangleSpaceForward = attachData['triangleSpaceForward'],
    triangleSpaceLeft = attachData['triangleSpaceUp'],
    -- Offset (sensor FLU frame) from the attach point to the report point.
    -- Kept as a plain {x, y, z} array for lpack transport to vlua.
    reportOffset = args.report_offset,
    isVisualised = args.isVisualised,
    accel_tau_s = args.accel_tau_s,
    gyro_tau_s = args.gyro_tau_s,
    vel_tau_s = args.vel_tau_s,
    wheel_angvel_tau_s = args.wheel_angvel_tau_s,
    attitude_mode = args.attitude_mode,
    attitude_tau_s = args.attitude_tau_s,
    debug_raw = args.debug_raw,
    torque_map = args.torque_map,
  }

  -- Let's log node indices and barycentric coordinates.
  log(
    'I',
    'gtState',
    string.format(
      'Attaching GtState sensor %d to vehicle %d with nodes (%d, %d, %d) and barycentric coords (u=%.3f, v=%.3f), signedProjDist=%.3f, numPhysicsStepsForGFXSave=%d',
      sensorId,
      vid,
      attachData['nodeIndex1'],
      attachData['nodeIndex2'],
      attachData['nodeIndex3'],
      attachData['u'],
      attachData['v'],
      attachData['signedProjDist'],
      args.numPhysicsStepsForGFXSave
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
  for _, v in pairs(newReadings.reading) do
    sensor.head = (sensor.head % sensor.maxSize) + 1
    sensor.buffer[sensor.head] = v
    -- if sensor.head == sensor.maxSize-1 then
    --   -- log that the buffer is full
    --   log('I', 'gtState', 'Buffer full for sensor ' .. newReadings.sensorId)
    -- end
  end
end

M.createGtState = createGtState
M.removeGtState = removeGtState
M.getGtStateReadings = getGtStateReadings
M.updateGtStateLastReadings = updateGtStateLastReadings

return M
