-- ./vehicle/controller/xlab/controller_manager.lua
local M = {}
local logTag = 'ControllerManager'

-- A minimal circular‐buffer latency tracker
local latency = {
  window = 100,
  data = {}, -- will hold up to `window` samples
  idx = 1,
  max = 0,
}

function latency:add(lat)
  self.data[self.idx] = lat
  self.idx = (self.idx % self.window) + 1
  if lat > self.max then self.max = lat end
end

function latency:average()
  local n, sum = #self.data, 0
  if n == 0 then return 0 end
  for i = 1, n do
    sum = sum + self.data[i]
  end
  return sum / n
end

-- This is our “shared” state, all controllers will refer to it
local common = {
  -- flags & sockets
  isRunning = false,
  listenIp = nil,
  listenPort = nil,
  sendIp = nil,
  sendPort = nil,
  socketIn = nil,
  socketOut = nil,

  -- performance & metrics
  calibration = {
    steeringP = 1.2,
    steeringI = 0.0,
    steeringD = 0.05,
    steeringDeadzone = 0.01,
    throttleP = 1.0,
    throttleI = 0.2,
    throttleD = 0.05,
    throttleMinRamp = 1.0,
    throttleMaxRamp = 10.0,
    brakeP = 1.0,
    brakeI = 0.2,
    brakeD = 0.0,
    brakeMinRamp = 1.0,
    brakeMaxRamp = 5.0,
  },

  performanceMetrics = {
    latency = nil,
    avgLatency = 0,
    maxLatency = 0,
    lastCommandTimestamp = 0,
    lastResponseTimestamp = 0,
    commandsReceived = 0,
    missedUpdates = 0,
  },

  getSimTime = nil,

  -- gtState
  gtStateSensorId = nil,
  gtStateManager = nil,
  updateGtReading = nil,
  cachedGtReading = nil,
  lastGtReadingTime = 0,

  -- vehicle cache
  vehicleState = {
    wheelspeed = 0,
    rpm = 0,
    engineTorque = 0,
    maxTorque = 0,
    maxRPM = 0,
    currentSteeringAngle = 0,
    maxSteeringAngle = 0.7,
    wheelRadius = 0.3,
    gearRatio = 1,
    mass = 1500,
    torqueCurve = {},
  },
}

local activeController = nil

-- Cache vehicle state via gtState or fallback
local function updateVehicleAndCacheState()
  local now = os.clock()

  -- pull from gtState if available
  if common.gtStateManager and common.gtStateSensorId then
    if not common.cachedGtReading or (now - common.lastGtReadingTime > 0.1) then
      if common.gtStateManager.getGtStateReadings then
        common.cachedGtReading = common.gtStateManager.getGtStateReadings(common.gtStateSensorId)[1]
      elseif common.gtStateManager.geGtStateReading then
        common.cachedGtReading = common.gtStateManager.geGtStateReading(common.gtStateSensorId)
      end
      common.lastGtReadingTime = now
      if not common.cachedGtReading then log('W', logTag, 'No reading from gtState sensor') end
    end
  end

  local vs = common.vehicleState
  if common.cachedGtReading then
    -- engine
    vs.rpm = common.cachedGtReading.RPM or electrics.values.rpm or 0
    vs.engineTorque = common.cachedGtReading.engineTorque or 0
    -- speed
    vs.wheelspeed = common.cachedGtReading.vel
        and math.sqrt(
          (common.cachedGtReading.vel[1] or 0) ^ 2
            + (common.cachedGtReading.vel[2] or 0) ^ 2
            + (common.cachedGtReading.vel[3] or 0) ^ 2
        )
      or electrics.values.wheelspeed
      or 0
    -- steering
    vs.currentSteeringAngle = common.cachedGtReading.steering or electrics.values.steering or 0
    -- gear
    if common.cachedGtReading.gearRatio then vs.gearRatio = common.cachedGtReading.gearRatio end
  else
    -- fallback to electrics / powertrain
    local e = powertrain.getDevice('mainEngine')
    if e then
      vs.rpm = electrics.values.rpm or 0
      vs.engineTorque = e.outputTorque or 0
    end
    vs.wheelspeed = electrics.values.wheelspeed or 0
    vs.currentSteeringAngle = electrics.values.steering or 0
    local gb = powertrain.getDevice('gearbox')
    if gb then vs.gearRatio = gb.gearRatio or 1 end
  end

  -- occasional caching of static values
  if not vs.maxValuesInitialized or (now % 5 < 0.1) then
    local e = powertrain.getDevice('mainEngine')
    if e then
      vs.maxRPM = e.maxRPM or 7000
      vs.maxTorque = e.maxTorque or 500
      if e.torqueData and e.torqueData.curves and e.torqueData.finalCurveName then
        vs.torqueCurve = e.torqueData.curves[e.torqueData.finalCurveName].torque
      end
    end
    if wheels and wheels.wheels and wheels.wheels[0] then
      vs.wheelRadius = wheels.wheels[0].radius or vs.wheelRadius
    end
    if hydros and hydros.hydros then
      for _, h in pairs(hydros.hydros) do
        if h.inputSource == 'steering_input' and h.steeringWheelLock then
          vs.maxSteeringAngle = math.rad(h.steeringWheelLock) / 2
          break
        end
      end
    end
    if v and v.data and v.data.nodes then
      local sum = 0
      for _, n in pairs(v.data.nodes) do
        if n.nodeWeight then sum = sum + n.nodeWeight end
      end
      if sum > 0 then vs.mass = sum end
    end
    vs.maxValuesInitialized = true
  end

  return common.cachedGtReading
end

local function commonInit(data)
  -- JSON encoder/decoder check
  if not jsonEncode or not jsonDecode then
    log('E', logTag, 'JSON encoder/decoder not initialized')
    return false
  end

  -- Reset performance metrics
  for k, _ in pairs(common.performanceMetrics) do
    common.performanceMetrics[k] = 0
  end
  common.performanceMetrics.latency = latency

  common.getSimTime = function() return obj:getSimTime() end

  -- If data is a packed string, decode it
  if type(data) == 'string' then
    local ok, decoded = pcall(function() return lpack.decode(data) end)
    if not ok then
      log('E', logTag, 'Failed to decode init data: ' .. tostring(decoded))
      return false
    end
    data = decoded
  end

  -- Extract endpoints
  common.listenIp = data.listenIp
  common.listenPort = data.listenPort
  common.sendIp = data.sendIp
  common.sendPort = data.sendPort

  -- Override any calibration params
  if data.calibration then
    for k, v in pairs(data.calibration) do
      if common.calibration[k] ~= nil then
        common.calibration[k] = v
        log('I', logTag, string.format('calibration.%s = %s', k, tostring(v)))
      else
        log('W', logTag, 'Unknown calibration parameter: ' .. k)
      end
    end
  end

  -- gtState sensor ID
  common.gtStateSensorId = data.gtStateSensorId
  if extensions.xlab_gtState then
    common.gtStateManager = extensions.xlab_gtState
    log('I', logTag, 'Found xlab_gtState extension')
  else
    log('W', logTag, 'gtState extension not found')
  end
  common.updateGtReading = updateVehicleAndCacheState

  -- Create & bind UDP sockets
  common.socketIn = socket.udp()
  local ok, err = common.socketIn:setsockname('0.0.0.0', common.listenPort)
  if not ok then
    log('E', logTag, 'Failed to bind socketIn: ' .. tostring(err))
    return false
  end
  common.socketIn:settimeout(0)
  log('I', logTag, 'Bound UDP receive socket on *:' .. common.listenPort)

  common.socketOut = socket.udp()
  common.socketOut:settimeout(0)

  common.isRunning = true
  log('I', logTag, 'Common init complete')
  return true
end

-- BeamNG‐style entrypoint
function M.init(data)
  if not commonInit(data) then
    log('E', logTag, 'Error during common init')
    return
  end

  -- pick the controller implementation
  local ctlType = data.controllerType
  local path = 'vehicle.controller.xlab.controller_' .. ctlType
  local ok, ctl = pcall(require, path)
  if not ok then
    log('E', logTag, "Could not load controller '" .. path .. "': " .. tostring(ctl))
    return
  end

  activeController = ctl
  if activeController.init then activeController.init(common) end
end

function M.update(dt)
  if common.isRunning and activeController and activeController.update then
    activeController.update(dt, common)
  end
end

function M.stop()
  if common.isRunning then
    common.isRunning = false
    if common.socketIn then common.socketIn:close() end
    if common.socketOut then common.socketOut:close() end
    if activeController and activeController.stop then activeController.stop(common) end
    log('I', logTag, 'Controller stopped')
  end
end

function M.setGtStateSensor(id)
  common.gtStateSensorId = id
  if activeController and activeController.setGtStateSensor then
    activeController.setGtStateSensor(id, common)
  end
end

function M.calibrate(params)
  if activeController and activeController.calibrate then
    activeController.calibrate(params, common)
  end
end

function M.reset()
  if activeController and activeController.reset then activeController.reset(common) end
end

function M.getStatus()
  if activeController and activeController.getStatus then
    return activeController.getStatus(common)
  end
  return { isRunning = false }
end

return M
