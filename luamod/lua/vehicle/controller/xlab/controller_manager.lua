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
  local now = common.getSimTime()

  -- pull from gtState if available, cache every 0.1 s
  if common.gtStateManager and common.gtStateSensorId then
    if not common.cachedGtReading or (now - common.lastGtReadingTime > 0.1) then
      local mgr = common.gtStateManager
      if mgr.getGtStateReadings then
        common.cachedGtReading = mgr.getGtStateReadings(common.gtStateSensorId)[1]
      elseif mgr.geGtStateReading then
        common.cachedGtReading = mgr.geGtStateReading(common.gtStateSensorId)
      end
      common.lastGtReadingTime = now
      if not common.cachedGtReading then log('W', logTag, 'No reading from gtState sensor') end
    end
  end

  local vs = common.vehicleState
  if common.cachedGtReading then
    local r = common.cachedGtReading
    -- engine
    vs.rpm = r.RPM or electrics.values.rpm or 0
    vs.engineTorque = r.engineTorque or 0
    -- speed
    vs.wheelspeed = (
      r.vel and math.sqrt((r.vel[1] or 0) ^ 2 + (r.vel[2] or 0) ^ 2 + (r.vel[3] or 0) ^ 2)
    )
      or electrics.values.wheelspeed
      or 0
    -- steering
    vs.currentSteeringAngle = r.steering or electrics.values.steering or 0
    -- gear
    if r.gearRatio then vs.gearRatio = r.gearRatio end
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

  return common.cachedGtReading
end

local function initVehicleStaticValues()
  log('I', logTag, 'Starting static values init')
  local vs = common.vehicleState

  local e = powertrain.getDevice('mainEngine')
  if e then
    vs.maxRPM = e.maxRPM or 7000
    vs.maxTorque = e.maxTorque or 500
    if e.torqueData and e.torqueData.curves and e.torqueData.finalCurveName then
      local name = e.torqueData.finalCurveName
      vs.torqueCurve = e.torqueData.curves[name].torque
    else
      log('W', logTag, 'torque data unavailable')
    end
  else
    log('W', logTag, 'main engine unavailable')
  end

  if wheels and wheels.wheels and wheels.wheels[0] then
    vs.wheelRadius = wheels.wheels[0].radius or vs.wheelRadius
  else
    log('W', logTag, 'wheels data unavailable')
  end

  if hydros then
    for _, h in pairs(hydros.hydros) do
      --check if it's a steering hydro
      if h.inputSource == 'steering_input' then
        --if the value is present, scale the values
        if h.steeringWheelLock then
          vs.maxSteeringAngle = math.abs(h.steeringWheelLock) / 2
          log('I', logTag, 'max steering angle = ' .. vs.maxSteeringAngle)
          break
        end
      end
    end
  else
    log('W', logTag, 'hydros unavailable')
  end

  if v and v.data and v.data.nodes then
    local sum = 0
    for _, n in pairs(v.data.nodes) do
      if n.nodeWeight then sum = sum + n.nodeWeight end
    end
    if sum > 0 then
      vs.mass = sum
      log('I', logTag, 'Total vehicle mass = ' .. vs.mass)
    end
  else
    log('W', logTag, 'v.data.nodes unavailable')
  end
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
  if data.calibration then M.calibrate(data.calibration) end

  -- init vehicle state
  initVehicleStaticValues()

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
  local ok, err = common.socketIn:setsockname(common.listenIp, common.listenPort)
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
