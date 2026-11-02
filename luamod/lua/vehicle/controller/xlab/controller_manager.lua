--[[
  controller_manager.lua — per-vehicle I/O gateway

  Role: UDP transport, message routing, plugin host. No control law here.

  Ports (vehicle/simulator-centric; see vehicle_io.py on the Python side):
    control_listen       vehicle BINDs — many clients sendto commands/tune
    control_state_send   vehicle sendto — companion BINDs for live controlStateOut
    sensor_send          vehicle sendto — companion BINDs for observation batches

  Each update tick (order matters):
    1. Drain latest command/context independently; route discrete config operations
    2. refreshControlState() once (gt._seq dedup)
    3. Stream control_state_send at controlStateRate
    4. Stream sensor_send for due sensor_broadcast streams
    5. Run actuation plugin (+ optional feedback plugin)

  Optional feedback: YAML `feedback: <stem>` → feedback_<stem>.lua
  See luamod/FEEDBACK_CONTRACT.md
]]

local abs, huge, exp = math.abs, math.huge, math.exp
local M = {}
local logTag = 'ControllerManager'

-- ============================================================================
-- §1  Shared runtime bag (passed to actuation / feedback plugins as `common`)
-- ============================================================================

local latency = { window = 100, data = {}, idx = 1, max = 0 }

function latency:add(lat)
  self.data[self.idx] = lat
  self.idx = (self.idx % self.window) + 1
  if lat > self.max then self.max = lat end
end

function latency:average()
  local n, sum = #self.data, 0
  if n == 0 then return 0 end
  for i = 1, n do sum = sum + self.data[i] end
  return sum / n
end

local common = {
  isRunning = false,
  isBypassed = false,

  controlListenIp = nil,
  controlListenPort = nil,
  controlStateSendIp = nil,
  controlStateSendPort = nil,
  socketIn = nil,
  socketOut = nil,
  sensorSendIp = nil,
  sensorSendPort = nil,
  sensorSocketOut = nil,

  controllerRate = 0,
  controlStateRate = 0,
  controlStateSendAccum = 0,
  controlStateSeq = nil,
  controlStateOut = {
    t = -1,
    x = 0, y = 0, z = 0,
    quat = { 0, 0, 0, 1 },
    yaw = 0, pitch = 0, roll = 0,
    Phi = 0, beta = 0,
    vx = 0, vy = 0, vz = 0, V = 0,
    p = 0, q = 0, r = 0,
    accel_x = 0, accel_y = 0, accel_z = 0,
    w_fl = 0, w_fr = 0, w_rl = 0, w_rr = 0,
    delta_l = 0, delta_r = 0,
    throttle = 0, brake = 0, pbrake = 0,
    gear_index = 0, gear_ratio = 0,
    we = 0, pb = 0,
    rear_wheel_torque_est = 0,
    torque_min = 0, torque_max = 0,
  },

  sensorBroadcastList = nil,
  sensorBatchOut = nil,

  performanceMetrics = {
    latency = nil, avgLatency = 0, maxLatency = 0,
    lastCommandTimestamp = 0, lastResponseTimestamp = 0,
    commandsReceived = 0, missedUpdates = 0,
  },

  torqueMapPath = nil,
  getSimTime = nil,
  gtStateSensorId = nil,
  gtStateManager = nil,
  environment = nil, -- optional model context shared with feedback/controller consumers
  rollFilter = nil,  -- optional final physical-command layer (controller_llc)
  rollInputValid = false, -- raw GT fields existed before legacy zero defaults

  constants = { rpmToAV = 0.104719755 },
}

local activeController = nil
local activeControllerType = nil
local activeFeedback = nil
local activeFeedbackStem = nil
local activeEnvironment = nil
local environmentUnavailableReported = false

-- Shared measured rates, updated only with a distinct GTState timestamp.
-- Raw differences retain the tested filter timing. The first-order smoothed
-- values are available to feedback/diagnostics, not silently substituted in QP.
local steeringRate, torqueRate = {}, {}
local rateTau, rateMaxGap = 0.02, 0.05
local function finite(x) return type(x)=='number' and x==x and abs(x)<huge end
local function updateRate(history, value, time)
  if not finite(value) or not finite(time) then
    history.t,history.value,history.raw,history.filtered=nil,nil,nil,nil
    return nil,nil,false
  end
  if time==history.t then return history.raw,history.filtered,history.raw~=nil end
  local dt=history.t and time-history.t
  local raw,filtered
  if dt and dt>0 and dt<=rateMaxGap then
    raw=(value-history.value)/dt
    if finite(raw) then
      local weight=rateTau>0 and (1-exp(-dt/rateTau)) or 1
      filtered=history.filtered and history.filtered+weight*(raw-history.filtered) or raw
    else raw=nil end
  end
  history.t,history.value,history.raw,history.filtered=time,value,raw,filtered
  return raw,filtered,raw~=nil
end
local function resetRates()
  steeringRate,torqueRate={},{}
  local cs=common.controlStateOut
  cs.steering_rate_raw,cs.steering_rate,cs.steering_rate_valid=0,0,false
  cs.torque_rate_raw,cs.torque_rate,cs.torque_rate_valid=0,0,false
end

-- ============================================================================
-- §2  control_state — live packet (control_state_send + sensor_broadcast)
-- ============================================================================

local function refreshControlState()
  if not common.gtStateManager or not common.gtStateSensorId then
    common.controlStateOut.t = -1
    resetRates()
    common.rollInputValid = false
    return false
  end

  local gt = common.gtStateManager.geGtStateReading(common.gtStateSensorId)
  if not gt or gt.time == nil then
    common.controlStateOut.t = -1
    resetRates()
    common.rollInputValid = false
    return false
  end
  if gt._seq == common.controlStateSeq then return true end
  common.controlStateSeq = gt._seq

  local cs = common.controlStateOut
  cs.t = gt.time
  cs.x, cs.y, cs.z = gt.pos[1], gt.pos[2], gt.pos[3]

  local q = cs.quat
  q[1], q[2], q[3], q[4] = gt.quat[1], gt.quat[2], gt.quat[3], gt.quat[4]

  cs.yaw = gt.yaw or 0
  cs.pitch = gt.pitch or 0
  cs.roll = gt.roll or 0
  cs.Phi = gt.Phi or 0
  cs.beta = gt.beta or 0
  cs.vx, cs.vy, cs.vz, cs.V = gt.vel[1], gt.vel[2], gt.vel[3], gt.V or 0
  cs.p, cs.q, cs.r = gt.angVel[1], gt.angVel[2], gt.angVel[3]
  cs.accel_x, cs.accel_y, cs.accel_z = gt.accel[1], gt.accel[2], gt.accel[3]

  local wfl, wfr = gt.wheelFL or {}, gt.wheelFR or {}
  local wrl, wrr = gt.wheelRL or {}, gt.wheelRR or {}
  -- Keep legacy published defaults, but never present absent measurements as
  -- valid zero wheel speeds, steering angles or torque capabilities to the filter.
  common.rollInputValid = wrl.speed ~= nil and wrr.speed ~= nil
    and wfl.angle ~= nil and wfr.angle ~= nil
    and gt.rear_wheel_torque_est ~= nil and gt.torque_min ~= nil and gt.torque_max ~= nil
    and finite(gt.brake) and gt.brake>=0 and gt.brake<=1 and finite(gt.pbrake)
  cs.w_fl = wfl.speed or 0
  cs.w_fr = wfr.speed or 0
  cs.w_rl = wrl.speed or 0
  cs.w_rr = wrr.speed or 0
  cs.delta_l = wfl.angle or 0
  cs.delta_r = wfr.angle or 0
  local raw,filtered,valid=updateRate(steeringRate,
    finite(wfl.angle) and finite(wfr.angle) and .5*(wfl.angle+wfr.angle) or nil,gt.time)
  cs.steering_rate_raw,cs.steering_rate,cs.steering_rate_valid=raw or 0,filtered or 0,valid
  raw,filtered,valid=updateRate(torqueRate,
    gt.torque_est_valid==true and gt.rear_wheel_torque_est or nil,gt.time)
  cs.torque_rate_raw,cs.torque_rate,cs.torque_rate_valid=raw or 0,filtered or 0,valid

  cs.throttle = gt.throttle or 0
  cs.brake = gt.brake or 0
  cs.pbrake = gt.pbrake or 0
  cs.gear_index = gt.gearIndex or 0
  cs.gear_ratio = gt.gearRatio or 0
  cs.we = (gt.RPM or 0) * common.constants.rpmToAV
  cs.pb = gt.turboBoost or 0
  cs.rear_wheel_torque_est = gt.rear_wheel_torque_est or 0
  cs.torque_min = gt.torque_min or 0
  cs.torque_max = gt.torque_max or 0
  return true
end

local function sendControlState(dt)
  if common.controlStateRate <= 0 then return end
  if not common.socketOut or not common.controlStateSendIp or not common.controlStateSendPort then return end

  common.controlStateSendAccum = common.controlStateSendAccum + dt
  if common.controlStateSendAccum < common.controlStateRate then return end
  common.controlStateSendAccum = common.controlStateSendAccum - common.controlStateRate

  if common.controlStateOut.t < 0 then
    log('W', logTag, 'control_state_send skipped: no gtState reading yet')
    return
  end
  common.socketOut:sendto(jsonEncode(common.controlStateOut), common.controlStateSendIp, common.controlStateSendPort)
end

-- ============================================================================
-- §3  sensor_send — observation batches (sensor_broadcast in YAML)
-- ============================================================================

local function fetchImuReading(sensorId)
  if not sensorId then return nil end
  if not extensions.tech_advancedIMU then extensions.load('tech/advancedIMU') end
  local ext = extensions.tech_advancedIMU
  if ext and ext.getLatest then return ext.getLatest(sensorId) end
  return nil
end

local function fetchGpsReading(sensorId)
  if not sensorId then return nil end
  if not extensions.tech_GPS then extensions.load('tech/GPS') end
  local ext = extensions.tech_GPS
  if ext and ext.getLatest then return ext.getLatest(sensorId) end
  return nil
end

local function packControlStateObservation(entry)
  local cs = common.controlStateOut
  if cs.t < 0 then return nil end
  return {
    sensor = 'control_state',
    name = entry.name,
    t = cs.t,
    data = cs,
  }
end

local function packImuObservation(entry)
  if not entry.sensorId then return nil end
  local reading = fetchImuReading(entry.sensorId)
  if not reading or reading.time == nil then return nil end
  return {
    sensor = 'imu', name = entry.name, t = reading.time,
    data = {
      pos = reading.pos,
      accel = reading.accSmooth or reading.accRaw,
      gyro = reading.angVelSmooth or reading.angVel,
      ang_accel = reading.angAccel,
      dir_x = reading.dirX, dir_y = reading.dirY, dir_z = reading.dirZ,
      mass = reading.mass,
    },
  }
end

local function packGpsObservation(entry)
  if not entry.sensorId then return nil end
  local reading = fetchGpsReading(entry.sensorId)
  if not reading or reading.time == nil then return nil end
  return {
    sensor = 'gps', name = entry.name, t = reading.time,
    data = { x = reading.x, y = reading.y, lon = reading.lon, lat = reading.lat },
  }
end

local SENSOR_PACK_FN = {
  control_state = packControlStateObservation,
  imu = packImuObservation,
  gps = packGpsObservation,
}

local function initSensorBroadcast(data)
  common.sensorBroadcastList = {}
  local cfg = data.sensor_broadcast
  if type(cfg) ~= 'table' then return end

  for name, entry in pairs(cfg) do
    if type(entry) == 'table' then
      local sensorType = entry.sensor
      local packFn = sensorType and SENSOR_PACK_FN[sensorType]
      if not packFn then
        log('E', logTag, string.format(
          'sensor_broadcast %s: unknown or missing sensor type %s',
          name, tostring(sensorType)
        ))
      else
        common.sensorBroadcastList[#common.sensorBroadcastList + 1] = {
          name = name,
          sensor = sensorType,
          rate = tonumber(entry.rate) or 0.05,
          accum = 0,
          sensorId = entry.sensorId,
          packFn = packFn,
        }
        local e = common.sensorBroadcastList[#common.sensorBroadcastList]
        log('I', logTag, string.format(
          'sensor_broadcast %s: type=%s id=%s rate=%.4fs',
          name, e.sensor, tostring(e.sensorId), e.rate
        ))
      end
    end
  end

  common.sensorBatchOut = common.sensorBatchOut or { sent_t = 0 }
end

local function sendSensorObservations(dt)
  if not common.sensorSocketOut or not common.sensorBroadcastList then return end

  local list = common.sensorBroadcastList
  local anyDue = false
  for i = 1, #list do
    list[i].accum = list[i].accum + dt
    if list[i].accum >= list[i].rate then anyDue = true; break end
  end
  if not anyDue then return end

  local batch = common.sensorBatchOut
  for i = 1, #list do batch[list[i].name] = nil end
  batch.sent_t = common.getSimTime()

  local count = 0
  for i = 1, #list do
    local entry = list[i]
    if entry.accum >= entry.rate then
      entry.accum = entry.accum - entry.rate
      local payload = entry.packFn(entry)
      if payload then
        batch[entry.name] = payload
        count = count + 1
      else
        log('W', logTag, 'sensor_send skipped for ' .. entry.name .. ': no reading yet')
      end
    end
  end
  if count == 0 then return end

  common.sensorSocketOut:sendto(jsonEncode(batch), common.sensorSendIp, common.sensorSendPort)
end

-- ============================================================================
-- §5  control_listen — latest per channel; one dispatcher for all message types
-- ============================================================================

local function drainControlListen(route)
  if not common.socketIn then return nil end
  local lastCommand, lastEnvironment, err
  repeat
    local msg, _, _, recvErr = common.socketIn:receivefrom()
    err = recvErr
    if msg and #msg > 0 then
      if msg:sub(1, 4) == 'XLE1' then
        lastEnvironment = msg
      elseif msg:sub(1,4)=='XLP1' or msg:sub(1,4)=='XLG1' then
        lastCommand = {type = 'plan', data = msg}
      else
        local envelope = jsonDecode(msg)
        if type(envelope) ~= 'table' then
          log('W', logTag, 'Ignoring non-table control_listen payload')
        elseif envelope.type == 'cmd' then
          lastCommand = envelope
        else
          -- Configuration/reset/bypass are discrete operations, not driving plans.
          route(envelope.type, envelope.data)
        end
      end
    end
  until not msg and (not err or err == 'timeout')
  if err and err ~= 'timeout' then
    log('E', logTag, 'control_listen error: ' .. tostring(err))
  end
  if lastEnvironment then route('environment', lastEnvironment) end
  return lastCommand
end

local function routeIncomingMessage(msgType, data)
  if msgType == 'environment' then
    if activeEnvironment then
      activeEnvironment.onPacket(data)
    elseif not environmentUnavailableReported then
      log('W', logTag, 'Ignoring environment packet: module not configured')
      environmentUnavailableReported = true
    end
    return
  end
  if msgType == 'tune' then
    local payload = data or {}
    local tuneOk = true
    if common.rollFilter and type(payload.roll_filter) == 'table' then
      tuneOk = common.rollFilter.onTune(payload.roll_filter) ~= false
    end
    if payload.feedback_config ~= nil then
      if activeFeedback and activeFeedback.onTune and type(payload.feedback_config) == 'table' then
        tuneOk = activeFeedback.onTune({ feedback_config = payload.feedback_config }) ~= false and tuneOk
      else
        log('E', logTag, 'feedback_config requires a configured, tunable feedback module')
        tuneOk = false
      end
    end
    if payload.feedback_gains ~= nil then
      log('E', logTag, 'Use feedback_config instead of feedback_gains')
      tuneOk = false
    end
    if activeController and activeController.onTune then
      tuneOk = activeController.onTune(payload) ~= false and tuneOk
    else
      M.calibrate(payload)
    end
    if not tuneOk then log('W', logTag, 'Tune rejected') end
    return
  end

  if msgType == 'bypass' then
    M.toggleBypass(data and data.enabled == true)
    return
  end

  if msgType == 'reset' then
    M.reset()
    return
  end

  if msgType == 'plan' then
    assert(activeController and activeController.onPlanPacket,'Execution plans require the LLC receiver')
    activeController.onPlanPacket(data,activeFeedback)
    return
  end

  if msgType == 'cmd' then
    assert(not (activeFeedback and activeFeedback.requiresReferencePacket),
      'TVLQR requires a paired reference/gain packet; bare cmd rejected')
    if type(data) == 'table' and activeController and activeController.onCommand then
      activeController.onCommand(data)
    end
    return
  end

  log('W', logTag, 'Ignoring unknown control_listen message type: ' .. tostring(msgType))
end

-- ============================================================================
-- §6  Actuation tick — plugin + optional feedback (FEEDBACK_CONTRACT.md)
-- ============================================================================

local function runActuationTick(dt)
  if not activeController then return end

  if activeFeedback then
    local ctl = activeController
    if not (ctl.prepareControlStep and ctl.resolveSetpoints and ctl.applySetpoints) then
      error(logTag .. ': feedback requires actuation plugin split API')
    end

    local step = ctl.prepareControlStep(dt, common)
    if not step then return end

    local resolved = ctl.resolveSetpoints(step.plant, step.dt_control)
    if resolved==false then
      if ctl.finishControlStep then ctl.finishControlStep(common,step) end
      return
    end
    -- External feedback plugins must not transform human targets after timeout.
    local sp_eff = resolved
    if not step.manual then
      sp_eff = activeFeedback.transform({
        sim_t = common.getSimTime(),
        dt = step.dt_control,
        plant = step.plant,
        resolved = resolved,
        raw = ctl.getRawTargets and ctl.getRawTargets() or nil,
      })
    end
    if sp_eff == false then
      if ctl.finishControlStep then ctl.finishControlStep(common, step) end
      return
    end
    if type(sp_eff) ~= 'table' then sp_eff = resolved end

    ctl.applySetpoints(step.plant, sp_eff, step.dt_control, step)
    if ctl.finishControlStep then ctl.finishControlStep(common, step) end
    return
  end

  if activeController.update then
    activeController.update(dt, common)
  end
end

-- ============================================================================
-- §7  Init
-- ============================================================================

local function applyDrivetrainConfig(drtr)
  if type(drtr) ~= 'table' then return end
  if drtr.mode then
    drivetrain.setShifterMode(drtr.mode)
    log('I', logTag, 'drivetrain mode=' .. drtr.mode)
  end
  if drtr.startGear then
    drivetrain.shiftToGear(drtr.startGear)
    log('I', logTag, 'drivetrain startGear=' .. drtr.startGear)
  end
  if drtr.disableSafety and extensions.xlab_xlabCore then
    extensions.xlab_xlabCore.stopSafetyFeatures(drtr.disableWhiteList or drtr.disableWhitelist)
    log('I', logTag, 'drivetrain safety disabled')
  end
end

local function commonInit(data)
  if not jsonEncode or not jsonDecode then
    log('E', logTag, 'JSON encoder/decoder not initialized')
    return false
  end
  if type(data) == 'string' then data = lpack.decode(data) end

  for k, _ in pairs(common.performanceMetrics) do common.performanceMetrics[k] = 0 end
  common.performanceMetrics.latency = latency
  common.getSimTime = function() return obj:getSimTime() end

  -- I/O (sim_manager injects control_listen / control_state_send / sensor_send)
  common.controlListenIp = data.control_listen_ip
  common.controlListenPort = data.control_listen
  common.controlStateSendIp = data.control_state_send_ip
  common.controlStateSendPort = data.control_state_send
  common.sensorSendIp = data.sensor_send_ip
  common.sensorSendPort = data.sensor_send

  if not common.controlListenIp or not common.controlListenPort then
    log('E', logTag, 'Missing control_listen_ip / control_listen')
    return false
  end
  if not common.controlStateSendIp or not common.controlStateSendPort then
    log('E', logTag, 'Missing control_state_send_ip / control_state_send')
    return false
  end

  common.controlStateRate = tonumber(data.controlStateRate) or 0
  common.controlStateSendAccum = 0
  common.controlStateSeq = nil
  common.controlStateOut.t = -1
  local rates=data.measured_rates or {}
  for key in pairs(rates) do
    assert(key=='filter_tau_s' or key=='max_gap_s','Unknown measured_rates setting: '..tostring(key))
  end
  rateTau=rates.filter_tau_s or 0.02
  rateMaxGap=rates.max_gap_s or 0.05
  assert(finite(rateTau) and rateTau>=0 and finite(rateMaxGap) and rateMaxGap>0,
    'Invalid measured_rates settings')
  resetRates()
  common.controllerRate = data.controllerRate
  if common.controlStateRate > 0 and common.controlStateRate < common.controllerRate then
    common.controlStateRate = common.controllerRate
  end

  common.gtStateSensorId = data.gtStateSensorId
  common.torqueMapPath = data.torqueMapPath
  common.gtStateManager = extensions.xlab_gtState

  applyDrivetrainConfig(data.drivetrain)
  initSensorBroadcast(data)

  common.socketIn = socket.udp()
  local ok, err = common.socketIn:setsockname(common.controlListenIp, common.controlListenPort)
  if not ok then
    log('E', logTag, 'Failed to bind control_listen: ' .. tostring(err))
    return false
  end
  common.socketIn:settimeout(0)
  common.socketOut = socket.udp()
  common.socketOut:settimeout(0)

  if common.sensorSendPort and common.sensorBroadcastList and #common.sensorBroadcastList > 0 then
    common.sensorSocketOut = socket.udp()
    common.sensorSocketOut:settimeout(0)
  end

  log('I', logTag, string.format(
    'I/O listen=%s:%s control_state_send->%s:%s sensor_send->%s:%s (%d streams)',
    common.controlListenIp, common.controlListenPort,
    common.controlStateSendIp, common.controlStateSendPort,
    common.sensorSendIp, common.sensorSendPort,
    common.sensorBroadcastList and #common.sensorBroadcastList or 0
  ))

  common.isRunning = true
  return true
end

-- ============================================================================
-- §8  Public API
-- ============================================================================

function M.init(data)
  if not commonInit(data) then return end

  activeControllerType = data.controllerType
  activeController = require('vehicle.controller.xlab.controller_' .. activeControllerType)

  if data.calibration and M.calibrate(data.calibration) == false then
    error(logTag .. ': invalid controller calibration')
  end
  if activeController.init then activeController.init(common) end

  -- Optional received-model context. Configure paths/identity under environment.
  -- Feedback consumers receive this same module through common.environment.
  activeEnvironment, common.environment = nil, nil
  environmentUnavailableReported = false
  if data.environment ~= nil then
    assert(type(data.environment) == 'table', 'environment must be a configuration table')
    activeEnvironment = require('vehicle.controller.xlab.vehicle_environment')
    assert(activeEnvironment.init(common, data.environment), 'Environment initialization failed')
    common.environment = activeEnvironment
  end

  -- Final physical-command filter: loaded only when explicitly configured.
  -- Environment alone loads no roll controller and changes no actuation.
  common.rollFilter = nil
  if data.roll_filter ~= nil then
    assert(data.controllerType == 'llc', 'roll_filter currently requires controller_llc')
    assert(type(data.roll_filter) == 'table', 'roll_filter must be a table')
    local steeringMin,steeringMax = activeController.getSteeringCapability()
    local rc = data.roll_filter
    assert(type(rc.steering_min_rad) == 'number' and type(rc.steering_max_rad) == 'number'
      and rc.steering_min_rad >= steeringMin and rc.steering_max_rad <= steeringMax,
      'roll_filter steering bounds exceed calibrated input capability')
    common.rollFilter = require('vehicle.controller.xlab.roll_filter')
    assert(common.rollFilter.init(common, data.roll_filter), 'Roll filter initialization failed')
  end
  if data.controllerType == 'llc' then
    -- Preserve driver inputs and release autonomous outputs even without a filter.
    activeController.enableManualInput(data.manual_input)
  end

  -- Optional feedback plugin (fail-fast require; see FEEDBACK_CONTRACT.md)
  activeFeedback = nil
  activeFeedbackStem = nil
  assert(data.feedback_gains == nil, 'Use feedback_config instead of feedback_gains')
  local stem = data.feedback
  if stem and stem ~= '' and stem ~= 'none' then
    activeFeedbackStem = tostring(stem)
    activeFeedback = require('vehicle.controller.xlab.feedback_' .. activeFeedbackStem)
    if activeFeedback.init then
      activeFeedback.init(common, {
        stem = activeFeedbackStem,
        settings = type(data.feedback_config) == 'table' and data.feedback_config or {},
      })
    end
    log('I', logTag, "feedback='" .. activeFeedbackStem .. "'")
  end
end

function M.update(dt)
  if not common.isRunning then return end
  local lastMsg = drainControlListen(routeIncomingMessage)
  if lastMsg then routeIncomingMessage(lastMsg.type, lastMsg.data) end
  refreshControlState()
  if common.rollFilter and not common.isBypassed then
    common.rollFilter.observe(common.controlStateOut, common.getSimTime())
  end
  sendControlState(dt)
  sendSensorObservations(dt)
  runActuationTick(dt)
end

function M.toggleBypass(enabled)
  if type(enabled) == 'boolean' then
    common.isBypassed = enabled
    if enabled and activeController and activeController.releaseInput then
      activeController.releaseInput()
    end
    log('I', logTag, 'bypass=' .. tostring(enabled))
  end
end

function M.stop()
  if not common.isRunning then return end
  common.isRunning = false
  if common.socketIn then common.socketIn:close() end
  if common.socketOut then common.socketOut:close() end
  if common.sensorSocketOut then common.sensorSocketOut:close() end
  if activeController and activeController.stop then activeController.stop(common) end
  if activeFeedback and activeFeedback.stop then activeFeedback.stop() end
  if common.rollFilter then common.rollFilter.stop(); common.rollFilter = nil end
  if activeEnvironment then activeEnvironment.stop() end
  activeEnvironment, common.environment = nil, nil
  activeFeedback, activeFeedbackStem = nil, nil
end

function M.setGtStateSensor(id)
  common.gtStateSensorId = id
end

function M.calibrate(params)
  if type(params) == 'table' and activeFeedback and activeFeedback.onTune
      and type(params.feedback_config) == 'table' then
    if activeFeedback.onTune({ feedback_config = params.feedback_config }) == false then return false end
  end
  if activeController and activeController.calibrate then
    return activeController.calibrate(params) ~= false
  end
  return true
end

function M.reset()
  common.controlStateSeq = nil
  resetRates()
  if common.rollFilter then common.rollFilter.reset() end
  if activeEnvironment then activeEnvironment.reset() end
  if activeFeedback and activeFeedback.reset then activeFeedback.reset() end
  if activeController and activeController.reset then activeController.reset(common) end
end

function M.getStatus()
  local status = activeController and activeController.getStatus
    and activeController.getStatus(common) or {isRunning = common.isRunning}
  if activeFeedback and activeFeedback.getStatus then status.feedback = activeFeedback.getStatus() end
  if activeEnvironment then status.environment = activeEnvironment.getStatus() end
  if common.rollFilter then status.roll_filter = common.rollFilter.getStatus() end
  return status
end

return M
