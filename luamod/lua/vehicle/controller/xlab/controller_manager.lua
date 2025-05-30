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
  isBypassed = false,
  listenIp = nil,
  listenPort = nil,
  sendIp = nil,
  sendPort = nil,
  socketIn = nil,
  socketOut = nil,
  controllerRate = 0,

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

  -- RPM lookup
  torqueLookup = nil,

  -- vehicle cache
  vehicleState = {
    maxTorque = 0,
    maxRPM = 0,
    wheelRadius = 0.3,
    mass = 1200,
    torqueCurve = {},
  },

  constants = { rpmToAV = 0.104719755, avToRPM = 9.549296596425384 },
}

local activeController = nil

local function makeTorqueLookup(curve, vsMaxTorque)
  -- collect & sort RPM keys
  local rpms = {}
  for k, _ in pairs(curve) do
    if type(k) == 'number' then rpms[#rpms + 1] = k end
  end
  table.sort(rpms)

  -- closure captures curve, rpms, vsMaxTorque
  local function getMaxTorqueAtRPM(rpm)
    if type(rpm) ~= 'number' or #rpms == 0 then return vsMaxTorque end
    -- below range?
    if rpm <= rpms[1] then return curve[rpms[1]] end
    -- above range?
    if rpm >= rpms[#rpms] then return curve[rpms[#rpms]] end
    -- find bracket
    local lo, hi
    for i = 1, #rpms - 1 do
      if rpm >= rpms[i] and rpm <= rpms[i + 1] then
        lo, hi = rpms[i], rpms[i + 1]
        break
      end
    end
    -- interpolate
    local Tlo, Thi = curve[lo], curve[hi]
    local w = (rpm - lo) / (hi - lo)
    return Tlo * (1 - w) + Thi * w
  end

  local function calculateThrottleFromTorque(reqT, rpm)
    local maxT = getMaxTorqueAtRPM(rpm)
    local r = reqT / (maxT + 1e-6)
    return math.min(1, math.max(0, r))
  end

  return {
    getMaxTorqueAtRPM = getMaxTorqueAtRPM,
    calculateThrottleFromTorque = calculateThrottleFromTorque,
  }
end

-- Cache vehicle state via gtState or fallback
local function updateVehicleAndCacheState()
  local now = common.getSimTime()

  -- pull from gtState if available, cache every 0.1 s
  if common.gtStateManager and common.gtStateSensorId then
    if not common.cachedGtReading or (now - common.lastGtReadingTime > 0.1) then
      local mgr = common.gtStateManager
      if mgr.geGtStateReading then
        common.cachedGtReading = mgr.geGtStateReading(common.gtStateSensorId)
      end
      common.lastGtReadingTime = now
      if not common.cachedGtReading then log('W', logTag, 'No reading from gtState sensor') end
    end
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

  -- This gets steering column angle, not wheel angle
  -- if hydros then
  --   for _, h in pairs(hydros.hydros) do
  --     --check if it's a steering hydro
  --     if h.inputSource == 'steering_input' then
  --       --if the value is present, scale the values
  --       if h.steeringWheelLock then
  --         vs.maxSteeringAngle = math.abs(h.steeringWheelLock) / 2
  --         log('I', logTag, 'max steering angle = ' .. vs.maxSteeringAngle)
  --         break
  --       end
  --     end
  --   end
  -- else
  --   log('W', logTag, 'hydros unavailable')
  -- end

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

  -- Controller rate
  common.controllerRate = data.controllerRate

  -- init vehicle state
  initVehicleStaticValues()

  -- gtState sensor ID
  common.gtStateSensorId = data.gtStateSensorId
  if extensions.xlab_gtState then
    common.gtStateManager = extensions.xlab_gtState
  else
    log('W', logTag, 'gtState extension not found')
  end
  common.updateGtReading = updateVehicleAndCacheState

  -- Set up drivetrain transmission mode
  if data.drivetrain then
    local drtr = data.drivetrain
    if drtr.mode then
      -- shifterMode = 0 : realistic (manual)
      -- shifterMode = 1 : realistic (manual autoclutch)
      -- shifterMode = 2 : arcade
      -- shifterMode = 3 : realistic (automatic)
      drivetrain.setShifterMode(data.drivetrain.mode)
      log('I', logTag, 'Set shifter mode to ' .. drtr.mode)
    end
    if drtr.startGear then
      drivetrain.shiftToGear(drtr.startGear)
      log('I', logTag, 'Set gear to ' .. drtr.startGear)
    end
  end

  -- Torque RPM lookup
  common.torqueLookup =
    makeTorqueLookup(common.vehicleState.torqueCurve, common.vehicleState.maxTorque)

  -- Create & bind UDP sockets
  common.socketIn = socket.udp()
  local ok, err = common.socketIn:setsockname(common.listenIp, common.listenPort)
  if not ok then
    log('E', logTag, 'Failed to bind socketIn: ' .. tostring(err))
    return false
  end
  common.socketIn:settimeout(0)
  log('D', logTag, 'Bound UDP receive socket on ' .. common.listenIp .. ':' .. common.listenPort)

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

  -- Override any calibration params
  if data.calibration then
    M.calibrate(data.calibration)
  else
    log('W', logTag, 'No calibration data sent')
  end
  if activeController.init then activeController.init(common) end
end

function M.update(dt)
  if common.isRunning and activeController and activeController.update then
    activeController.update(dt, common)
  end
end

function M.toggleBypass(bypassActive)
  if type(bypassActive) == 'boolean' then
    common.isBypassed = bypassActive
    log('I', logTag, 'Controller bypass toggled to: ' .. tostring(common.isBypassed))
  else
    log('W', logTag, 'toggleBypass called with non-boolean value: ' .. tostring(bypassActive))
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
    activeController.calibrate(params)
  else
    log('W', logTag, 'No controller found for calibration')
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
