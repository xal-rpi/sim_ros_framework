-- Ground-truth state: published FLU + COM transport. No attach triangle.
-- FLU (RH): x forward (wheelbase), y left, z up.
-- ω from getRollPitchYawAngularVelocity; BeamNG yawAV is right-positive, so
-- ω_world += −yawAV * up. Pitch stays +pitchAV * right.

local M = {}

local sqrt, abs, ceil = math.sqrt, math.abs, math.ceil
local atan2, asin, max, min = math.atan2, math.asin, math.max, math.min
local exp = math.exp
local constants = { rpmToAV = 0.104719755, avToRPM = 9.549296596425384 }

local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

local logTag = 'GtState'
local ffi = require('ffi')
local customFields = {}
local torquePolicyLib = nil
local torqueMapEnabled = false
local torqueMapFieldName = 'rear_wheel_torque_est'
local torqueMapApi = 'legacy'
local torqueMapRKin = 0.38
-- Floor linear rear speed before / r_kin so forward + invert stay on-map at rest.
local MAP_WHEEL_SPEED_MIN_MS = 1.0
local railTmin = ffi.new('float[1]')
local railTmax = ffi.new('float[1]')
local gtStateManager = nil

local function occupancyWheelRads(rearWheelspeedMs)
  local r = torqueMapRKin
  if not r or r < 0.05 then r = 0.38 end
  local v = rearWheelspeedMs
  if v < MAP_WHEEL_SPEED_MIN_MS then v = MAP_WHEEL_SPEED_MIN_MS end
  return v / r
end

local filters = {
  params = {
    accelTauS = 0.005,
    gyroTauS = 0.005,
    velTauS = 0.01,
    wheelAngVelTauS = 0.01,
    debugRaw = false,
  },
  cache = {
    nominalDt = 0.0,
    nominalAlpha = {},
    epsDt = 1e-4,
  },
  state = {
    velWorld = { initialized = false, raw = vec3(), v = vec3() },
    gyroLocal = { initialized = false, raw = vec3(), p1 = vec3(), p2 = vec3() },
    accelWorld = { initialized = false, raw = vec3(), p1 = vec3(), p2 = vec3() },
    wheelsAngVel = {
      initialized = false,
      rawFr = 0.0, rawFl = 0.0, rawRr = 0.0, rawRl = 0.0,
      fr = 0.0, fl = 0.0, rr = 0.0, rl = 0.0,
    },
  },
  modules = {},
}

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

local function emaScalarStep(prev, raw, alpha)
  return prev + (raw - prev) * alpha
end

function filters.isNominalDt(dt)
  return abs(dt - filters.cache.nominalDt) <= filters.cache.epsDt
end

function filters.rebuildCache(dt)
  if filters.isNominalDt(dt) then return end
  log('W', logTag, string.format("Rebuilding filter cache for nominalDt=%.6f (was %.6f)", dt, filters.cache.nominalDt))
  filters.cache.nominalDt = dt
  filters.cache.nominalAlpha.accel = alphaFromTau(dt, filters.params.accelTauS)
  filters.cache.nominalAlpha.gyro = alphaFromTau(dt, filters.params.gyroTauS)
  filters.cache.nominalAlpha.vel = alphaFromTau(dt, filters.params.velTauS)
  filters.cache.nominalAlpha.wheelAngVel = alphaFromTau(dt, filters.params.wheelAngVelTauS)
end

function filters.resetAll()
  filters.state.velWorld.initialized = false
  filters.state.gyroLocal.initialized = false
  filters.state.accelWorld.initialized = false
  filters.state.wheelsAngVel.initialized = false
end

filters.modules.velWorld = {
  step = function(dt, vWorldRaw)
    local st = filters.state.velWorld
    st.raw:set(vWorldRaw)
    if not st.initialized then
      st.v:set(vWorldRaw)
      st.initialized = true
      return st.v
    end
    return emaVec3Step(st.v, vWorldRaw, filters.cache.nominalAlpha.vel)
  end,
}

filters.modules.gyroLocal = {
  step = function(dt, wLocalRaw)
    local st = filters.state.gyroLocal
    st.raw:set(wLocalRaw)
    if not st.initialized then
      st.p1:set(wLocalRaw)
      st.p2:set(wLocalRaw)
      st.initialized = true
      return st.p2
    end
    local pass1 = emaVec3Step(st.p1, wLocalRaw, filters.cache.nominalAlpha.gyro)
    return emaVec3Step(st.p2, pass1, filters.cache.nominalAlpha.gyro)
  end,
}

filters.modules.accelWorld = {
  step = function(dt, accelWorldRaw)
    local st = filters.state.accelWorld
    st.raw:set(accelWorldRaw)
    if not st.initialized then
      st.p1:set(accelWorldRaw)
      st.p2:set(accelWorldRaw)
      st.initialized = true
      return st.p2
    end
    local pass1 = emaVec3Step(st.p1, accelWorldRaw, filters.cache.nominalAlpha.accel)
    return emaVec3Step(st.p2, pass1, filters.cache.nominalAlpha.accel)
  end,
}

filters.modules.wheelsAngVel = {
  step = function(dt, frInfo, flInfo, rrInfo, rlInfo)
    local st = filters.state.wheelsAngVel
    st.rawFr = frInfo.angVel
    st.rawFl = flInfo.angVel
    st.rawRr = rrInfo.angVel
    st.rawRl = rlInfo.angVel
    if not st.initialized then
      st.fr = frInfo.angVel
      st.fl = flInfo.angVel
      st.rr = rrInfo.angVel
      st.rl = rlInfo.angVel
      st.initialized = true
    else
      local alpha = filters.cache.nominalAlpha.wheelAngVel
      st.fr = emaScalarStep(st.fr, frInfo.angVel, alpha)
      st.fl = emaScalarStep(st.fl, flInfo.angVel, alpha)
      st.rr = emaScalarStep(st.rr, rrInfo.angVel, alpha)
      st.rl = emaScalarStep(st.rl, rlInfo.angVel, alpha)
    end
    frInfo.angVel = st.fr
    flInfo.angVel = st.fl
    rrInfo.angVel = st.rr
    rlInfo.angVel = st.rl
  end,
}

local function writeVec3Table(dst, v)
  dst[1] = v.x
  dst[2] = v.y
  dst[3] = v.z
end

local function writeWheelInfoTable(dst, src)
  dst.speed = src.speed or 0.0
  dst.angVel = src.angVel or 0.0
  dst.brakeTorque = src.brakeTorque or 0.0
  dst.propTorque = src.propTorque or 0.0
  dst.downForce = src.downForce or 0.0
  dst.angle = src.angle or 0.0
end

local function ensureWheelInfoTable(dst)
  dst.speed = dst.speed or 0.0
  dst.angVel = dst.angVel or 0.0
  dst.brakeTorque = dst.brakeTorque or 0.0
  dst.propTorque = dst.propTorque or 0.0
  dst.downForce = dst.downForce or 0.0
  dst.angle = dst.angle or 0.0
end

local function writeDriveStatusFromElectrics(dst, elecVals)
  dst.esc = elecVals.esc
  dst.abs = elecVals.abs
  dst.tcs = elecVals.tcs
  dst.engineRunning = elecVals.engineRunning
  dst.isRealisticDrive = elecVals.gearboxMode == 'realistic' and 1 or 0
  dst.mode4WD = elecVals.mode4WD or 0
  dst.modeRangeBox = elecVals.modeRangeBox or 0
end

local sensorConfig = {
  id = nil,
  gfxUpdateTime = nil,
  gravity = vec3(0, 0, -9.81),
}

local ringBuffer = {
  numPhysicsStepsForGFXSave = 1,
  physicsTimer = nil,
  physicsUpdateTime = nil,
  readings = {},
  ringSize = 0,
  writeIdx = 0,
  writeSeq = 0,
  readSeq = 0,
  ringInitialized = false,
  latestReading = nil,
}

local wheelRotators, wheelIds = wheels.wheelRotators, wheels.wheelRotatorIDs
local wheel_fr, wheel_fl, wheel_rr, wheel_rl = {}, {}, {}, {}

local function deepCopyTable(src)
  if type(src) ~= 'table' then return src end
  local dst = {}
  for k, v in pairs(src) do
    dst[k] = deepCopyTable(v)
  end
  return dst
end

local function ensureDebugFields(dst)
  dst.velRaw = { 0, 0, 0 }
  dst.accelRaw = { 0, 0, 0 }
  dst.gravityBody = { 0, 0, 0 }
  dst.angVelRaw = { 0, 0, 0 }
  dst.angVelObjRPY = { 0, 0, 0 }
  dst.velRef = { 0, 0, 0 }
  dst.velOmegaR = { 0, 0, 0 }
  dst.accelComInertial = { 0, 0, 0 }
  dst.dirXBody = { 0, 0, 0 }
  dst.dirYBody = { 0, 0, 0 }
  dst.rFlu = { 0, 0, 0 }
  dst.yawAtoB = 0.0
  dst.rGeom = 0.0
  dst.wheelFR_angVelRaw = 0.0
  dst.wheelFL_angVelRaw = 0.0
  dst.wheelRR_angVelRaw = 0.0
  dst.wheelRL_angVelRaw = 0.0
end

local function ensureLatestReadingTables()
  ringBuffer.latestReading = ringBuffer.latestReading or {}
  local latest = ringBuffer.latestReading
  latest.dirX = { 0, 0, 0 }
  latest.dirY = { 0, 0, 0 }
  latest.accel = { 0, 0, 0 }
  latest.angVel = { 0, 0, 0 }
  latest.pos = { 0, 0, 0 }
  latest.vel = { 0, 0, 0 }
  latest.quat = { 0, 0, 0, 1 }
  latest.wheelFR = {}
  latest.wheelFL = {}
  latest.wheelRR = {}
  latest.wheelRL = {}
  latest.driveStatus = {}
  ensureWheelInfoTable(latest.wheelFR)
  ensureWheelInfoTable(latest.wheelFL)
  ensureWheelInfoTable(latest.wheelRR)
  ensureWheelInfoTable(latest.wheelRL)
  if filters.params.debugRaw then
    ensureDebugFields(latest)
  end
end

local vehicleState = { currVeh = nil, engine = nil, gearbox = nil }

local tmpVectors = {
  sensorPos = vec3(0, 0, 0),
  vec1 = vec3(0, 0, 0), vec2 = vec3(0, 0, 0),
  hubFl = vec3(0, 0, 0), hubFr = vec3(0, 0, 0),
  hubRl = vec3(0, 0, 0), hubRr = vec3(0, 0, 0),
  axisFl = vec3(0, 0, 0), axisFr = vec3(0, 0, 0),
  axisRl = vec3(0, 0, 0), axisRr = vec3(0, 0, 0),
  frontMid = vec3(0, 0, 0), rearMid = vec3(0, 0, 0),
  refFwd = vec3(0, 0, 0), refRight = vec3(0, 0, 0), refUp = vec3(0, 0, 0),
  bodyFwd = vec3(0, 0, 0), bodyLeft = vec3(0, 0, 0), bodyUp = vec3(0, 0, 0),
  refPos = vec3(0, 0, 0),
  rWorld = vec3(0, 0, 0),
  omegaWorld = vec3(0, 0, 0),
  velRef = vec3(0, 0, 0), omegaR = vec3(0, 0, 0), velCom = vec3(0, 0, 0),
  accelInertial = vec3(0, 0, 0), specific = vec3(0, 0, 0),
  omegaFlu = vec3(0, 0, 0),
  steeringRoll = vec3(0, 0, 0),
}

local bodyState = {
  captured = false,
  rFlu = vec3(0, 0, 0),
  prevVelWorld = vec3(0, 0, 0),
  prevVelInit = false,
  prevFwd = vec3(0, 0, 0),
  prevFwdInit = false,
  rGeom = 0.0,
  yawAtoB = 0.0,
  settleStableS = 0.0,
  settleWaitS = 0.0,
  rollAV = 0.0,
  pitchAV = 0.0,
  yawAV = 0.0,
}

local wheelInfoTables = {
  fr = { speed = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0 },
  fl = { speed = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0 },
  rr = { speed = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0 },
  rl = { speed = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0 },
}

local function fillWheelInfos(dst, mWheel)
  dst.speed = mWheel.wheelSpeed
  dst.angVel = mWheel.angularVelocity * mWheel.wheelDir
  dst.brakeTorque = abs(mWheel.coreData.brakeTorqueApplied) - mWheel.frictionTorque
  dst.propTorque = mWheel.propulsionTorque * mWheel.wheelDir
  dst.downForce = mWheel.downForce or 0
  return dst
end

-- axis = B − A already in tmpVectors (shared with hub mid).
local function wheelAngleFromAxis(axis)
  local tv = tmpVectors
  tv.steeringRoll:setCross(axis, tv.bodyUp)
  if tv.steeringRoll:squaredLength() < 1e-12 then return 0 end
  if tv.steeringRoll:dot(tv.bodyFwd) < 0 then tv.steeringRoll:setScaled(-1) end
  return atan2(tv.steeringRoll:dot(tv.bodyLeft), tv.steeringRoll:dot(tv.bodyFwd))
end

local function computeRingSize(gfxDt, physicsDt)
  local samplesPerGfx = max(1, ceil(gfxDt / physicsDt))
  return max(4, 2 * samplesPerGfx)
end

local function bootstrapRingFromFirstReading()
  ringBuffer.ringSize = computeRingSize(sensorConfig.gfxUpdateTime, ringBuffer.physicsUpdateTime)
  ringBuffer.readings = {}
  for i = 1, ringBuffer.ringSize do
    ringBuffer.readings[i] = deepCopyTable(ringBuffer.latestReading)
  end
  ringBuffer.ringInitialized = true
  ringBuffer.writeIdx = 1
  ringBuffer.writeSeq = 1
  ringBuffer.readSeq = 0
  ringBuffer.latestReading = ringBuffer.readings[ringBuffer.writeIdx]
  ringBuffer.latestReading._seq = ringBuffer.writeSeq
end

local function backfillCustomFieldAcrossRing(fieldName, defaultValue)
  if not ringBuffer.ringInitialized then
    if ringBuffer.latestReading then ringBuffer.latestReading[fieldName] = defaultValue end
    return
  end
  for i = 1, ringBuffer.ringSize do
    ringBuffer.readings[i][fieldName] = defaultValue
  end
end

local function beginNewWrite(DoMoveIndex)
  local rb = ringBuffer
  if not rb.ringInitialized then
    rb.writeSeq = 1
    rb.latestReading._seq = rb.writeSeq
    return
  end
  if DoMoveIndex then
    rb.writeIdx = (rb.writeIdx % rb.ringSize) + 1
    rb.writeSeq = rb.writeSeq + 1
    rb.latestReading = rb.readings[rb.writeIdx]
    rb.latestReading._seq = rb.writeSeq
    return
  end
  rb.latestReading = rb.readings[(rb.writeIdx % rb.ringSize) + 1]
end

local function getPendingGFXReadings()
  local rb = ringBuffer
  if not rb.ringInitialized then return {} end
  local out = {}
  local seq = rb.readSeq + rb.numPhysicsStepsForGFXSave
  local lastTried = nil
  while seq <= rb.writeSeq do
    local idx = ((seq - 1) % rb.ringSize) + 1
    local slot = rb.readings[idx]
    if slot and slot._seq == seq then
      out[#out + 1] = slot
    end
    lastTried = seq
    seq = seq + rb.numPhysicsStepsForGFXSave
  end
  if lastTried then rb.readSeq = lastTried end
  return out
end

local function registerCustomField(fieldName, defaultValue)
  customFields[fieldName] = defaultValue
  backfillCustomFieldAcrossRing(fieldName, defaultValue)
  log('I', logTag, 'Registered custom field: ' .. fieldName)
  return true
end

local function setCustomField(fieldName, value)
  if customFields[fieldName] ~= nil then
    customFields[fieldName] = value
    return true
  end
  log('E', logTag, 'Custom field not found: ' .. fieldName)
  return false
end

local function setTorqueMapLib(lib, api)
  torquePolicyLib = lib
  torqueMapApi = (api == 'occupancy_rail') and 'occupancy_rail' or 'legacy'
  if lib and torqueMapApi == 'occupancy_rail' then
    local r = tonumber(lib.drivetrain_r_kin)
    if r and r > 0.05 then
      torqueMapRKin = r
    end
  end
  if lib then
    log(
      'I',
      logTag,
      string.format('Torque map lib attached api=%s r_kin=%.5f', torqueMapApi, torqueMapRKin)
    )
  end
end

local function quatFromAxes(dirX, dirY, dirZ)
  local m00, m01, m02 = dirX.x, dirY.x, dirZ.x
  local m10, m11, m12 = dirX.y, dirY.y, dirZ.y
  local m20, m21, m22 = dirX.z, dirY.z, dirZ.z
  local trace = m00 + m11 + m22
  if trace > 1.0e-6 then
    local s = 0.5 / sqrt(trace + 1.0)
    return (m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s, 0.25 / s
  elseif m00 > m11 and m00 > m22 then
    local s = 2.0 * sqrt(1.0 + m00 - m11 - m22)
    return 0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s
  elseif m11 > m22 then
    local s = 2.0 * sqrt(1.0 + m11 - m00 - m22)
    return (m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s
  else
    local s = 2.0 * sqrt(1.0 + m22 - m00 - m11)
    return (m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s
  end
end

local function resetBodyState()
  bodyState.captured = false
  bodyState.prevVelInit = false
  bodyState.prevFwdInit = false
  bodyState.rGeom = 0.0
  bodyState.yawAtoB = 0.0
  bodyState.settleStableS = 0.0
  bodyState.settleWaitS = 0.0
  bodyState.rFlu:set(0, 0, 0)
  bodyState.prevVelWorld:set(0, 0, 0)
  bodyState.prevFwd:set(0, 0, 0)
end

local function captureComOffset(dt, tv, vs)
  if bodyState.captured then return end
  local speed = tv.velRef:length()
  local omega = sqrt(
    bodyState.rollAV * bodyState.rollAV
      + bodyState.pitchAV * bodyState.pitchAV
      + bodyState.yawAV * bodyState.yawAV
  )
  bodyState.settleWaitS = bodyState.settleWaitS + dt
  if speed < 0.02 and omega < 0.15 then
    bodyState.settleStableS = bodyState.settleStableS + dt
  else
    bodyState.settleStableS = 0.0
  end
  local ready = bodyState.settleStableS >= 0.5
  local timedOut = bodyState.settleWaitS >= 5.0
  if not ready and not timedOut then return end

  local com = vs.currVeh:calcCenterOfGravity(false)
  if not com or not com.x then return end
  tv.rWorld:set(com.x - tv.refPos.x, com.y - tv.refPos.y, com.z - tv.refPos.z)
  bodyState.rFlu:set(
    tv.rWorld:dot(tv.bodyFwd),
    tv.rWorld:dot(tv.bodyLeft),
    tv.rWorld:dot(tv.bodyUp)
  )
  bodyState.captured = true
  -- Do not Δv / rGeom across the r jump (timeout-while-moving would spike a).
  bodyState.prevVelInit = false
  bodyState.prevFwdInit = false
  filters.state.accelWorld.initialized = false
  filters.state.velWorld.initialized = false
  log(
    'I',
    logTag,
    string.format(
      'freeze r_FLU=[%.4f, %.4f, %.4f] m  |r|=%.4f  yawAtoB=%.4f rad  wait=%.2fs%s',
      bodyState.rFlu.x,
      bodyState.rFlu.y,
      bodyState.rFlu.z,
      bodyState.rFlu:length(),
      bodyState.yawAtoB,
      bodyState.settleWaitS,
      timedOut and not ready and ' (settle timeout)' or ''
    )
  )
end

local function update(dtSim)
  local rb = ringBuffer
  local vs = vehicleState
  local tv = tmpVectors
  local wi = wheelInfoTables

  rb.physicsTimer = rb.physicsTimer + dtSim
  if rb.physicsTimer < rb.physicsUpdateTime then return end
  local dt = rb.physicsTimer
  rb.physicsTimer = rb.physicsTimer - rb.physicsUpdateTime
  filters.rebuildCache(dt)
  beginNewWrite(false)

  -- One RPY / pose / vel read per step.
  local rollAV, pitchAV, yawAV = vs.currVeh:getRollPitchYawAngularVelocity()
  bodyState.rollAV, bodyState.pitchAV, bodyState.yawAV = rollAV, pitchAV, yawAV
  tv.refFwd:set(vs.currVeh:getDirectionVector())
  tv.refFwd:normalize()
  tv.refUp:set(vs.currVeh:getDirectionVectorUp())
  tv.refUp:normalize()
  tv.refRight:setCross(tv.refFwd, tv.refUp)
  tv.refRight:normalize()
  tv.refPos:set(vs.currVeh:getPosition())
  tv.velRef:set(vs.currVeh:getVelocity())

  -- One node pair per wheel: hub mid + steer axis.
  local p1 = vs.currVeh:getNodePosition(wheel_fl.node1)
  local p2 = vs.currVeh:getNodePosition(wheel_fl.node2)
  tv.hubFl:set((p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5)
  tv.axisFl:set(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z) -- node2 → node1
  p1 = vs.currVeh:getNodePosition(wheel_fr.node1)
  p2 = vs.currVeh:getNodePosition(wheel_fr.node2)
  tv.hubFr:set((p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5)
  tv.axisFr:set(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z) -- node1 → node2
  p1 = vs.currVeh:getNodePosition(wheel_rl.node1)
  p2 = vs.currVeh:getNodePosition(wheel_rl.node2)
  tv.hubRl:set((p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5)
  tv.axisRl:set(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z)
  p1 = vs.currVeh:getNodePosition(wheel_rr.node1)
  p2 = vs.currVeh:getNodePosition(wheel_rr.node2)
  tv.hubRr:set((p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5)
  tv.axisRr:set(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)

  tv.frontMid:setAdd2(tv.hubFl, tv.hubFr)
  tv.frontMid:setScaled(0.5)
  tv.rearMid:setAdd2(tv.hubRl, tv.hubRr)
  tv.rearMid:setScaled(0.5)

  -- Published triad (B): z from (A), x = wheelbase flattened.
  tv.bodyUp:set(tv.refUp)
  tv.bodyFwd:setSub2(tv.frontMid, tv.rearMid)
  tv.vec1:setScaled2(tv.bodyUp, tv.bodyFwd:dot(tv.bodyUp))
  tv.bodyFwd:setSub(tv.vec1)
  if tv.bodyFwd:squaredLength() < 1e-12 then
    tv.bodyFwd:set(tv.refFwd)
  end
  tv.bodyFwd:normalize()
  tv.bodyLeft:setCross(tv.bodyUp, tv.bodyFwd)
  tv.bodyLeft:normalize()
  tv.bodyUp:setCross(tv.bodyFwd, tv.bodyLeft)
  tv.bodyUp:normalize()

  if filters.params.debugRaw then
    tv.vec1:setCross(tv.refFwd, tv.bodyFwd)
    bodyState.yawAtoB = atan2(tv.vec1:dot(tv.bodyUp), tv.refFwd:dot(tv.bodyFwd))
    if bodyState.prevFwdInit and dt > 1e-6 then
      tv.vec1:setSub2(tv.bodyFwd, bodyState.prevFwd)
      bodyState.rGeom = tv.vec1:dot(tv.bodyLeft) / dt
    else
      bodyState.rGeom = 0.0
    end
    bodyState.prevFwd:set(tv.bodyFwd)
    bodyState.prevFwdInit = true
  end

  -- ω_world: +roll * fwd + pitch * right − yaw * up (FLU nose-left r).
  tv.omegaWorld:setScaled2(tv.refFwd, rollAV)
  tv.vec1:setScaled2(tv.refRight, pitchAV)
  tv.omegaWorld:setAdd(tv.vec1)
  tv.vec1:setScaled2(tv.refUp, -yawAV)
  tv.omegaWorld:setAdd(tv.vec1)

  captureComOffset(dt, tv, vs)

  if bodyState.captured then
    tv.rWorld:setScaled2(tv.bodyFwd, bodyState.rFlu.x)
    tv.vec1:setScaled2(tv.bodyLeft, bodyState.rFlu.y)
    tv.rWorld:setAdd(tv.vec1)
    tv.vec1:setScaled2(tv.bodyUp, bodyState.rFlu.z)
    tv.rWorld:setAdd(tv.vec1)
    tv.sensorPos:set(tv.refPos.x + tv.rWorld.x, tv.refPos.y + tv.rWorld.y, tv.refPos.z + tv.rWorld.z)
  else
    -- Until rFlu is frozen, pos is live wet COM. Transport stays off (r=0).
    tv.rWorld:set(0, 0, 0)
    local com = vs.currVeh:calcCenterOfGravity(false)
    if com and com.x then
      tv.sensorPos:set(com.x, com.y, com.z)
    else
      tv.sensorPos:set(tv.refPos)
    end
  end

  tv.omegaR:setCross(tv.omegaWorld, tv.rWorld)
  tv.velCom:setAdd2(tv.velRef, tv.omegaR)

  if bodyState.prevVelInit and dt > 1e-6 then
    tv.vec1:setSub2(tv.velCom, bodyState.prevVelWorld)
    tv.vec1:setScaled(1.0 / dt)
  else
    tv.vec1:set(0, 0, 0)
  end
  bodyState.prevVelWorld:set(tv.velCom)
  bodyState.prevVelInit = true

  local aFilt = filters.modules.accelWorld.step(dt, tv.vec1)
  tv.accelInertial:set(aFilt)
  tv.specific:setSub2(tv.accelInertial, sensorConfig.gravity)

  local velWorld = filters.modules.velWorld.step(dt, tv.velCom)
  tv.omegaFlu:set(
    tv.omegaWorld:dot(tv.bodyFwd),
    tv.omegaWorld:dot(tv.bodyLeft),
    tv.omegaWorld:dot(tv.bodyUp)
  )
  local wFilt = filters.modules.gyroLocal.step(dt, tv.omegaFlu)

  fillWheelInfos(wi.fr, wheel_fr)
  wi.fr.angle = wheelAngleFromAxis(tv.axisFr)
  fillWheelInfos(wi.fl, wheel_fl)
  wi.fl.angle = wheelAngleFromAxis(tv.axisFl)
  fillWheelInfos(wi.rr, wheel_rr)
  wi.rr.angle = wheelAngleFromAxis(tv.axisRr)
  fillWheelInfos(wi.rl, wheel_rl)
  wi.rl.angle = wheelAngleFromAxis(tv.axisRl)
  filters.modules.wheelsAngVel.step(dt, wi.fr, wi.fl, wi.rr, wi.rl)

  local latest = rb.latestReading
  local elecVals = electrics.values
  latest.time = vs.currVeh:getSimTime()
  writeVec3Table(latest.dirX, tv.bodyFwd)
  writeVec3Table(latest.dirY, tv.bodyLeft)
  writeVec3Table(latest.pos, tv.sensorPos)
  latest.vel[1] = velWorld:dot(tv.bodyFwd)
  latest.vel[2] = velWorld:dot(tv.bodyLeft)
  latest.vel[3] = velWorld:dot(tv.bodyUp)
  latest.accel[1] = tv.specific:dot(tv.bodyFwd)
  latest.accel[2] = tv.specific:dot(tv.bodyLeft)
  latest.accel[3] = tv.specific:dot(tv.bodyUp)
  latest.angVel[1] = wFilt.x
  latest.angVel[2] = wFilt.y
  latest.angVel[3] = wFilt.z
  local qx, qy, qz, qw = quatFromAxes(tv.bodyFwd, tv.bodyLeft, tv.bodyUp)
  latest.quat[1] = qx
  latest.quat[2] = qy
  latest.quat[3] = qz
  latest.quat[4] = qw

  local vx, vy = latest.vel[1], latest.vel[2]
  local V = sqrt(vx * vx + vy * vy)
  local yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
  local pitch = asin(clamp(2 * (qw * qy - qz * qx), -1, 1))
  local roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
  latest.V = V
  latest.yaw = yaw
  latest.pitch = pitch
  latest.roll = roll
  latest.beta = abs(V) > 0.5 and atan2(vy, vx) or 0
  latest.Phi = latest.beta + yaw

  writeWheelInfoTable(latest.wheelFR, wi.fr)
  writeWheelInfoTable(latest.wheelFL, wi.fl)
  writeWheelInfoTable(latest.wheelRR, wi.rr)
  writeWheelInfoTable(latest.wheelRL, wi.rl)

  latest.steering = elecVals.steering
  latest.throttle = elecVals.throttle
  latest.brake = elecVals.brake
  latest.clutch = elecVals.clutch
  latest.pbrake = elecVals.parkingbrake
  latest.steeringInput = elecVals.steering_input
  latest.throttleInput = elecVals.throttle_input
  latest.brakeInput = elecVals.brake_input
  latest.clutchInput = elecVals.clutch_input
  writeDriveStatusFromElectrics(latest.driveStatus, elecVals)

  latest.engineLoad = vs.engine and (vs.engine.isDisabled and 0 or vs.engine.instantEngineLoad) or 0
  latest.engineTorque = vs.engine and vs.engine.combustionTorque or 0
  latest.RPM = vs.engine and (vs.engine.outputAV1 * constants.avToRPM) or 0
  latest.flywheelTorque = vs.engine and vs.engine.outputTorque1 or 0
  latest.turboBoost = elecVals.turboBoost or -1
  latest.superchargerBoost = elecVals.superchargerBoost or -1
  latest.throttleValve = vs.engine and vs.engine.throttle
  latest.gearboxTorque = vs.gearbox and vs.gearbox.outputTorque1 or 0
  latest.gearRatio = vs.gearbox and vs.gearbox.gearRatio or 0
  latest.gearIndex = elecVals.gearIndex

  if filters.params.debugRaw then
    local g = sensorConfig.gravity
    latest.gravityBody[1] = g:dot(tv.bodyFwd)
    latest.gravityBody[2] = g:dot(tv.bodyLeft)
    latest.gravityBody[3] = g:dot(tv.bodyUp)
    latest.velRaw[1] = tv.velCom:dot(tv.bodyFwd)
    latest.velRaw[2] = tv.velCom:dot(tv.bodyLeft)
    latest.velRaw[3] = tv.velCom:dot(tv.bodyUp)
    local aRaw = filters.state.accelWorld.raw
    latest.accelRaw[1] = aRaw:dot(tv.bodyFwd) - latest.gravityBody[1]
    latest.accelRaw[2] = aRaw:dot(tv.bodyLeft) - latest.gravityBody[2]
    latest.accelRaw[3] = aRaw:dot(tv.bodyUp) - latest.gravityBody[3]
    latest.angVelRaw[1] = tv.omegaFlu.x
    latest.angVelRaw[2] = tv.omegaFlu.y
    latest.angVelRaw[3] = tv.omegaFlu.z
    latest.angVelObjRPY[1] = rollAV
    latest.angVelObjRPY[2] = pitchAV
    latest.angVelObjRPY[3] = yawAV
    latest.velRef[1] = tv.velRef:dot(tv.bodyFwd)
    latest.velRef[2] = tv.velRef:dot(tv.bodyLeft)
    latest.velRef[3] = tv.velRef:dot(tv.bodyUp)
    latest.velOmegaR[1] = tv.omegaR:dot(tv.bodyFwd)
    latest.velOmegaR[2] = tv.omegaR:dot(tv.bodyLeft)
    latest.velOmegaR[3] = tv.omegaR:dot(tv.bodyUp)
    latest.accelComInertial[1] = tv.accelInertial:dot(tv.bodyFwd)
    latest.accelComInertial[2] = tv.accelInertial:dot(tv.bodyLeft)
    latest.accelComInertial[3] = tv.accelInertial:dot(tv.bodyUp)
    writeVec3Table(latest.dirXBody, tv.bodyFwd)
    writeVec3Table(latest.dirYBody, tv.bodyLeft)
    writeVec3Table(latest.rFlu, bodyState.rFlu)
    latest.yawAtoB = bodyState.yawAtoB
    latest.rGeom = bodyState.rGeom
    latest.wheelFR_angVelRaw = filters.state.wheelsAngVel.rawFr
    latest.wheelFL_angVelRaw = filters.state.wheelsAngVel.rawFl
    latest.wheelRR_angVelRaw = filters.state.wheelsAngVel.rawRr
    latest.wheelRL_angVelRaw = filters.state.wheelsAngVel.rawRl
  end

  if torqueMapEnabled and torquePolicyLib then
    local rear_wheelspeed_ms = 0.5 * (wi.rr.speed + wi.rl.speed)
    local engine_speed_rads = latest.RPM * constants.rpmToAV
    local est
    if torqueMapApi == 'occupancy_rail' then
      local ww = occupancyWheelRads(rear_wheelspeed_ms)
      local boost = latest.turboBoost or 0
      est = tonumber(torquePolicyLib.drivetrain_forward_torque(
        engine_speed_rads, latest.throttle, boost, ww
      ))
      latest.torque_min = tonumber(torquePolicyLib.drivetrain_forward_torque(
        engine_speed_rads, 0, boost, ww
      )) or 0
      latest.torque_max = tonumber(torquePolicyLib.drivetrain_forward_torque(
        engine_speed_rads, 1, boost, ww
      )) or 0
    else
      est = tonumber(torquePolicyLib.drivetrain_forward_torque(
        engine_speed_rads, latest.turboBoost, rear_wheelspeed_ms, latest.throttle
      ))
      local t0 = tonumber(torquePolicyLib.drivetrain_forward_torque(
        engine_speed_rads, latest.turboBoost, rear_wheelspeed_ms, 0
      ))
      local t1 = tonumber(torquePolicyLib.drivetrain_forward_torque(
        engine_speed_rads, latest.turboBoost, rear_wheelspeed_ms, 1
      ))
      latest.torque_min = min(t0, t1)
      latest.torque_max = max(t0, t1)
    end
    setCustomField(torqueMapFieldName, est)
  else
    latest.torque_min = 0
    latest.torque_max = 0
  end

  for fieldName, value in pairs(customFields) do
    latest[fieldName] = value
  end

  beginNewWrite(true)
  if not rb.ringInitialized then
    bootstrapRingFromFirstReading()
  end
  gtStateManager.cacheLatestReading(sensorConfig.id, latest)
end

local function init(data)
  gtStateManager = extensions.xlab_gtState
  sensorConfig.id = data.sensorId
  sensorConfig.gfxUpdateTime = data.GFXUpdateTime or 0.033
  vehicleState.currVeh = obj

  local g = obj:getGravityVector()
  if g and g.x then
    sensorConfig.gravity:set(g)
  end

  ringBuffer.physicsUpdateTime = data.physicsUpdateTime or 0.005
  ringBuffer.numPhysicsStepsForGFXSave = max(1, tonumber(data.numPhysicsStepsForGFXSave) or 1)

  filters.params.accelTauS = tonumber(data.accelTauS or data.accel_tau_s) or filters.params.accelTauS
  filters.params.gyroTauS = tonumber(data.gyroTauS or data.gyro_tau_s) or filters.params.gyroTauS
  filters.params.velTauS = tonumber(data.velTauS or data.vel_tau_s) or filters.params.velTauS
  filters.params.wheelAngVelTauS = tonumber(data.wheelAngVelTauS or data.wheel_angvel_tau_s) or filters.params.wheelAngVelTauS
  filters.params.debugRaw = (data.debugRaw == true) or (data.debug == true) or (data.debug_raw == true)

  log(
    'I',
    logTag,
    string.format(
      'Filter config | accelTauS=%.3f gyroTauS=%.3f velTauS=%.3f wheelAngVelTauS=%.3f debugRaw=%s',
      filters.params.accelTauS,
      filters.params.gyroTauS,
      filters.params.velTauS,
      filters.params.wheelAngVelTauS,
      tostring(filters.params.debugRaw)
    )
  )

  filters.cache.nominalDt = 0
  filters.rebuildCache(ringBuffer.physicsUpdateTime)
  filters.resetAll()
  resetBodyState()
  ringBuffer.physicsTimer = 0.0

  wheel_fr = wheelRotators[wheelIds['FR']]
  wheel_fl = wheelRotators[wheelIds['FL']]
  wheel_rr = wheelRotators[wheelIds['RR']]
  wheel_rl = wheelRotators[wheelIds['RL']]

  vehicleState.engine = powertrain.getDevice('mainEngine')
  vehicleState.gearbox = powertrain.getDevice('gearbox')
  if vehicleState.engine == nil then log('E', logTag, 'Engine reference is nil') end
  if vehicleState.gearbox == nil then log('E', logTag, 'Gearbox reference is nil') end

  torquePolicyLib = nil
  torqueMapEnabled = false
  torqueMapApi = 'legacy'
  local torqueMapCfg = data.torque_map
  if torqueMapCfg ~= nil then
    torqueMapEnabled = true
    torqueMapFieldName = torqueMapCfg.field_name or 'rear_wheel_torque_est'
    if customFields[torqueMapFieldName] == nil then
      registerCustomField(torqueMapFieldName, 0)
    end
    log('I', logTag, 'torque_map estimate enabled field=' .. tostring(torqueMapFieldName))
  end

  ringBuffer.readings = {}
  ringBuffer.ringSize = computeRingSize(sensorConfig.gfxUpdateTime, ringBuffer.physicsUpdateTime)
  ringBuffer.writeIdx = 0
  ringBuffer.writeSeq = 0
  ringBuffer.readSeq = 0
  ringBuffer.ringInitialized = false
  ensureLatestReadingTables()

  log(
    'I',
    logTag,
    string.format(
      'Initialized sensor %d | COM+RPY path | Physics=%.1fkHz GFX=%.1fHz | RingSize=%d',
      sensorConfig.id,
      1 / ringBuffer.physicsUpdateTime / 1000,
      1 / sensorConfig.gfxUpdateTime,
      ringBuffer.ringSize
    )
  )
end

local function stop()
  torquePolicyLib = nil
  torqueMapEnabled = false
  torqueMapApi = 'legacy'
end

local function getSensorData()
  return {
    currentPos = tmpVectors.sensorPos,
    rawReadings = getPendingGFXReadings(),
  }
end

local function getLatest() return ringBuffer.latestReading end

M.update = update
M.init = init
M.stop = stop
M.getSensorData = getSensorData
M.getLatest = getLatest
M.registerCustomField = registerCustomField
M.setCustomField = setCustomField
M.setTorqueMapLib = setTorqueMapLib

return M
