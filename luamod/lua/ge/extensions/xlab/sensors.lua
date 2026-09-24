local M = {}

-- GE-side gtState registry: id + vlua init + poll buffer. No SensorMatrix.
local gtStateLastRawReadings = {}

local function createGtState(vid, args)
  if args.GFXUpdateTime == nil then args.GFXUpdateTime = 0.1 end
  if args.isVisualised == nil then args.isVisualised = true end
  if args.physicsUpdateTime == nil then args.physicsUpdateTime = 0.01 end
  if args.numPhysicsStepsForGFXSave == nil then args.numPhysicsStepsForGFXSave = 1 end

  local sensorId = Research.SensorManager.getNewSensorId()
  local data = {
    sensorId = sensorId,
    GFXUpdateTime = args.GFXUpdateTime,
    physicsUpdateTime = args.physicsUpdateTime,
    numPhysicsStepsForGFXSave = args.numPhysicsStepsForGFXSave,
    isVisualised = args.isVisualised,
    accel_tau_s = args.accel_tau_s,
    gyro_tau_s = args.gyro_tau_s,
    vel_tau_s = args.vel_tau_s,
    wheel_angvel_tau_s = args.wheel_angvel_tau_s,
    debug_raw = args.debug_raw,
    torque_map = args.torque_map,
  }

  log(
    'I',
    'gtState',
    string.format(
      'Creating GtState %d on vehicle %d (COM+RPY, no attach triangle)',
      sensorId,
      vid
    )
  )

  local serializedData = string.format('extensions.xlab_gtState.create(%q)', lpack.encode(data))
  be:queueObjectLua(vid, serializedData)

  local maxSize = math.ceil(args.GFXUpdateTime / args.physicsUpdateTime) * 5
  gtStateLastRawReadings[sensorId] = {
    buffer = {},
    head = 0,
    maxSize = maxSize,
  }
  log('I', 'gtState', 'Created GtState sensor ' .. sensorId .. ' buffer=' .. maxSize)
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
  end
end

M.createGtState = createGtState
M.removeGtState = removeGtState
M.getGtStateReadings = getGtStateReadings
M.updateGtStateLastReadings = updateGtStateLastReadings

return M
