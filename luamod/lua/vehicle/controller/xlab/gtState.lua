-- Vehicle State Sensor Module
-- This module manages a vehicle state sensor, handling physics calculations,
-- orientation updates, wheel management, input processing, and data buffering
-- for graphical representation.

local M = {}

-- Import necessary math functions for performance
local sqrt, abs, acos, ceil = math.sqrt, math.abs, math.acos, math.ceil
local atan2, asin, max, min = math.atan2, math.asin, math.max, math.min
local exp = math.exp
local pi = math.pi
local constants = { rpmToAV = 0.104719755, avToRPM = 9.549296596425384 }

local function sign(x) return max(min((x * 1e200) * 1e200, 1), -1) end

local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

-- Logging tag for debugging purposes
local logTag = 'GtState'

-- Will store custom fields added by controllers or other modules.
local customFields = {}

-- Optional forward torque estimate (torque map lib attached by controller_llc).
local torquePolicyLib = nil
local torqueMapEnabled = false
local torqueMapFieldName = 'rear_wheel_torque_est'

local function setTorqueMapLib(lib)
  torquePolicyLib = lib
  if lib then
    log('I', logTag, 'Torque map lib attached for forward estimate')
  end
end

-- Reference to the global state manager extension
local gtStateManager = nil

-- Filtering (dt-aware) configuration and state.
-- This is intentionally structured for extensibility: to add a new filter later,
-- add one module under filters.modules and one state under filters.state.
local filters = {
  params = {
    accelTauS = 0.005, -- specific-force world acceleration smoothing (2-pass EMA)
    gyroTauS = 0.005, -- angular velocity smoothing (local frame)
    velTauS = 0.01, -- world velocity smoothing
    wheelAngVelTauS = 0.01, -- only wheel*.angVel is filtered (in-place)
    -- Sensor-frame attitude (see attitudeStep):
    --   "triangle"  — legacy: raw attach-triangle FLU every step
    --   "integrate" — propagate q with curl ω, slow pull to triangle FLU
    attitudeMode = 'triangle',
    attitudeTauS = 0.3, -- absolute pull to triangle [s]; integrate only
    debugRaw = true, -- if true, publish additional world-frame raw/filtered fields
  },
  cache = {
    nominalDt = 0.0,
    nominalAlpha = {},
    epsDt = 1e-4,
  },
  state = {
    velWorld = { 
      initialized = false, 
      raw = vec3(), v = vec3() 
    },
    gyroLocal = {
      initialized = false, 
      raw = vec3(), p1 = vec3(), p2 = vec3() 
    },
    accelWorld = {
      initialized = false,
      raw = vec3(),
      p1 = vec3(),
      p2 = vec3(),
    },
    wheelsAngVel = {
      initialized = false,
      rawFr = 0.0, rawFl = 0.0, rawRr = 0.0, rawRl = 0.0,
      fr = 0.0, fl = 0.0, rr = 0.0, rl = 0.0 
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
  filters.cache.nominalAlpha.attitudeAbs = alphaFromTau(dt, filters.params.attitudeTauS)
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

--[[
Sensor Core Properties
----------------------
These variables define the sensor's unique identification, positioning,
update timings, and operational flags.
]]
-- Group sensor configuration to reduce upvalues
local sensorConfig = {
  id = nil, -- Unique identifier for the sensor
  gfxUpdateTime = nil, -- Time interval (seconds) between graphics updates
  nodeIndex1 = nil, nodeIndex2 = nil, nodeIndex3 = nil, -- Indices of the three nodes forming the sensor's triangle
  b1 = nil, b2 = nil, b3 = nil, -- Barycentric coordinates relative to the triangle
  w1 = nil, w2 = nil, w3 = nil, -- Non-negative interpolation weights based on barycentric coordinates
  signedProjDist = nil, -- Signed distance from the sensor to the triangle plane
  triangleSpaceForward = nil, -- Forward direction vector in triangle space
  triangleSpaceLeft = nil, -- Left direction vector in triangle space
  -- Optional vector (sensor FLU frame, meters) from the attach point to the
  -- report point. Set when the sensor was attached away from the desired
  -- report point (attach_z_offset) to select a stiffer/flatter triangle.
  -- pos/vel/accel are rigid-body-transported by this offset every physics
  -- step; angVel/angAccel/quat are point-independent. nil = disabled.
  reportOffset = nil,
  m1 = 0, m2 = 0, m3 = 0, -- TODO: Assume static mass for now
  inv_m1 = 0, inv_m2 = 0, inv_m3 = 0, -- Inverse masses for the nodes (precomputed for efficiency)
  oneThird = 1.0 / 3.0, -- Constant for averaging over three nodes
}

--[[
Timing and Buffering Variables
--------------------------------
These variables manage the timing of physics and graphics updates,
as well as buffering of sensor readings for graphical representation.
]]
-- Group ring buffer and timing state to reduce upvalues
local ringBuffer = {
  numPhysicsStepsForGFXSave = 1, -- Number of physics steps before saving data for graphics
  physicsTimer = nil, -- Timer to track physics update intervals
  physicsUpdateTime = nil, -- Time interval (seconds) between physics updates
  readings = {}, -- Circular buffer to store raw sensor readings
  ringSize = 0, -- Number of preallocated reading slots
  writeIdx = 0, -- Circular write index [1..ringSize]
  writeSeq = 0, -- Monotonic sequence incremented every physics write
  readSeq = 0, -- Last sequence consumed by GFX polling
  ringInitialized = false, -- True once slots are bootstrapped from first full reading
  latestReading = nil, -- Alias to readings[writeIdx]
}

-- Wheels information
-- Assuming 'wheels' is a global table accessible within this module
local wheelRotators, wheelIds = wheels.wheelRotators, wheels.wheelRotatorIDs -- References to wheel rotators and their IDs
local wheel_fr, wheel_fl, wheel_rr, wheel_rl = {}, {}, {}, {} -- Tables to store individual wheel data


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

  -- angVelUncorr: legacy curlVel/denom (no M^-1), sensor frame.
  -- angVelObjRPY: engine getRollPitchYawAngularVelocity() (refNode frame).
  dst.angVelUncorr = { 0, 0, 0 }
  dst.angVelObjRPY = { 0, 0, 0 }
  -- Raw triangle FLU + v_world on those axes (legacy chatty body vel).
  dst.dirXTri = { 0, 0, 0 }
  dst.dirYTri = { 0, 0, 0 }
  dst.velTri = { 0, 0, 0 }

  -- Wheel-angle debug fields.
  dst.wheelFR_angleLegacy = 0.0
  dst.wheelFL_angleLegacy = 0.0
  dst.wheelRR_angleLegacy = 0.0
  dst.wheelRL_angleLegacy = 0.0

  -- Wheel angular velocity raw fields (unfiltered, for debugging).
  dst.wheelFR_angVelRaw = 0.0
  dst.wheelFL_angVelRaw = 0.0
  dst.wheelRR_angVelRaw = 0.0
  dst.wheelRL_angVelRaw = 0.0
end

local function ensureLatestReadingTables()
  ringBuffer.latestReading = ringBuffer.latestReading or {}
  ringBuffer.latestReading.dirX = { 0, 0, 0 }
  ringBuffer.latestReading.dirY = { 0, 0, 0 }
  ringBuffer.latestReading.accel = { 0, 0, 0 }
  ringBuffer.latestReading.angVel = { 0, 0, 0 }
  ringBuffer.latestReading.angAccel = { 0, 0, 0 }
  ringBuffer.latestReading.pos = { 0, 0, 0 }
  ringBuffer.latestReading.vel = { 0, 0, 0 }
  ringBuffer.latestReading.quat = { 0, 0, 0, 1 }
  ringBuffer.latestReading.wheelFR = {}
  ringBuffer.latestReading.wheelFL = {}
  ringBuffer.latestReading.wheelRR = {}
  ringBuffer.latestReading.wheelRL = {}
  ringBuffer.latestReading.driveStatus = {}

  ensureWheelInfoTable(ringBuffer.latestReading.wheelFR)
  ensureWheelInfoTable(ringBuffer.latestReading.wheelFL)
  ensureWheelInfoTable(ringBuffer.latestReading.wheelRR)
  ensureWheelInfoTable(ringBuffer.latestReading.wheelRL)

  -- Add custom fields with default values.
  if filters.params.debugRaw then
    ensureDebugFields(ringBuffer.latestReading)
  end

end

-- Group vehicle state references to reduce upvalues
local vehicleState = {
  currVeh = nil,
  engine = nil,
  gearbox = nil,
}

-- Group all temporary vectors to reduce upvalues
local tmpVectors = {
  sensorPos = vec3(0, 0, 0),
  currentDir = vec3(0, 0, 0),
  worldLeft = vec3(0, 0, 0),
  worldThird = vec3(0, 0, 0),
  angVelLocalRaw = vec3(0, 0, 0),
  vec1 = vec3(0, 0, 0), vec2 = vec3(0, 0, 0), vec3 = vec3(0, 0, 0),
  aCenter = vec3(0, 0, 0), vCenter = vec3(0, 0, 0), baryCenter = vec3(0, 0, 0),
  r = vec3(0, 0, 0), curlAcc = vec3(0, 0, 0), curlVel = vec3(0, 0, 0),
  accelWorld = vec3(0, 0, 0), velSensorRaw = vec3(0, 0, 0),
  angVel = vec3(0, 0, 0), angAccel = vec3(0, 0, 0),
  edge1 = vec3(0, 0, 0), edge2 = vec3(0, 0, 0),
  edge1Norm = vec3(0, 0, 0), edge2Norm = vec3(0, 0, 0),
  normal = vec3(0, 0, 0), triangleThird = vec3(0, 0, 0),
  reportOffsetWorld = vec3(0, 0, 0), -- sensorConfig.reportOffset rotated to world, per step
  accelSpecificRaw = vec3(0, 0, 0), -- debug: unfiltered specific force snapshot (world frame)
  accel1 = vec3(0, 0, 0), accel2 = vec3(0, 0, 0), accel3 = vec3(0, 0, 0),
  steeringRoll = vec3(0, 0, 0),
  -- Triangle FLU snapshot (debug / velTri) before attitude overwrite.
  dirTriX = vec3(0, 0, 0), dirTriY = vec3(0, 0, 0), dirTriZ = vec3(0, 0, 0),
}

-- Group wheel info tables to reduce upvalues
local wheelInfoTables = {
  fr = { speed = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0},
  fl = { speed = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0},
  rr = { speed = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0},
  rl = { speed = 0, angVel = 0, brakeTorque = 0, propTorque = 0, downForce = 0, angle = 0},
}

local function fillWheelInfos(dst, mWheel)
  dst.speed = mWheel.wheelSpeed
  dst.angVel = mWheel.angularVelocity * mWheel.wheelDir
  dst.brakeTorque = abs(mWheel.coreData.brakeTorqueApplied) - mWheel.frictionTorque
  dst.propTorque = mWheel.propulsionTorque * mWheel.wheelDir
  dst.downForce = mWheel.downForce or 0
  return dst
end

-- Legacy "road wheel angle" as used by BeamNG.Tech (acos(cos) + steering-input sign)
local function wheelAngleLegacyRad(nodeA, nodeB, signSteer)
  local cosAng = clamp(obj:nodeVecPlanarCosRightForward(nodeA, nodeB), -1, 1)
  return acos(cosAng) * -signSteer
end

-- Signed wheel heading angle via atan2, independent of steering input.
-- Computed in the sensor's local plane (currentDir/worldLeft), using worldThird as plane normal.
local function wheelAngleAtan2Rad(nodeA, nodeB)
  -- Cache for performance
  local vs = vehicleState
  local tv = tmpVectors
  
  -- local axis = currVeh:getNodePosition(nodeB) - currVeh:getNodePosition(nodeA)
  -- axis = axis - worldThird * axis:dot(worldThird) -- ignore camber vs sensor plane
  -- if axis:squaredLength() < 1e-18 then return 0 end
  -- axis = axis:normalized()

  -- local roll = axis:cross(worldThird) -- rolling direction (upRef = worldThird)
  -- roll = roll - worldThird * roll:dot(worldThird)
  -- if roll:squaredLength() < 1e-18 then return 0 end
  -- roll = roll:normalized()

  -- if roll:dot(currentDir) < 0 then roll = -roll end -- remove node-order flips
  -- return atan2(roll:dot(worldLeft), roll:dot(currentDir))

  -- local axis = currVeh:getNodePosition(nodeB) - currVeh:getNodePosition(nodeA)
  -- local roll = axis:cross(worldThird) -- wheel rolling direction
  -- if roll:squaredLength() < 1e-12 then return 0 end
  -- if roll:dot(currentDir) < 0 then roll = -roll end
  -- return atan2(roll:dot(worldLeft), roll:dot(currentDir))

  tv.vec1:setSub2(vs.currVeh:getNodePosition(nodeB), vs.currVeh:getNodePosition(nodeA))
  tv.steeringRoll:setCross(tv.vec1, tv.worldThird)
  if tv.steeringRoll:squaredLength() < 1e-12 then return 0 end
  if tv.steeringRoll:dot(tv.currentDir) < 0 then tv.steeringRoll:setScaled(-1) end
  return atan2(tv.steeringRoll:dot(tv.worldLeft), tv.steeringRoll:dot(tv.currentDir))
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
  -- If the ring is not initialized, the latestReading may be the only table 
  -- we have, so set the field there and it will be copied to all ring 
  -- slots when we bootstrap from the first reading.
  if not ringBuffer.ringInitialized then
    if ringBuffer.latestReading then ringBuffer.latestReading[fieldName] = defaultValue end
    return
  end

  -- Fill all the ring slots with the new field
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

  -- When the data has been fully filled
  if DoMoveIndex then
    rb.writeIdx = (rb.writeIdx % rb.ringSize) + 1
    rb.writeSeq = rb.writeSeq + 1
    rb.latestReading = rb.readings[rb.writeIdx]
    rb.latestReading._seq = rb.writeSeq
    return
  end

  -- Otherwise just move where latestReading points to,
  -- without incrementing the sequence (used for multiple physics steps per gfx step)
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

--[[
    Initialize fields that can be store inside the gtstate sensor.
    Parameters:
        fieldName (string): The name of the custom field to register.
        defaultValue (any): The default value for the custom field.
    Returns:
        bool: True if the field was registered successfully
]]
local function registerCustomField(fieldName, defaultValue)
  customFields[fieldName] = defaultValue
  backfillCustomFieldAcrossRing(fieldName, defaultValue)
  log('I', logTag, 'Registered custom field: ' .. fieldName)
  return true
end

--[[
    Set a custom field value.
    Parameters:
        fieldName (string): The name of the custom field to set.
        value (any): The value to assign to the custom field.
    Returns:
        bool: True if the field was set successfully, false if the field does not exist.
]]
local function setCustomField(fieldName, value)
  if customFields[fieldName] ~= nil then
    customFields[fieldName] = value
    return true
  else
    log('E', logTag, 'Custom field not found: ' .. fieldName)
  end
  return false
end

-- Body→world quat {x,y,z,w} from FLU axes in world (no table alloc).
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

-- --------------------------------------------------------------------------
-- Sensor attitude (attach-triangle frame, HF geometric flex rejected)
-- --------------------------------------------------------------------------
-- Curl ω is the rigid rate of the 3 nodes (clean). Triangle axes chatter
-- geometrically (~30 Hz) and fake sideslip via v·left. With mode=integrate:
--   predict with ω_world, slow-correct toward the measured triangle quat.
-- That keeps the sensor/panel frame (slow pose) without the flex mode.
-- mode=triangle keeps the raw axes (legacy).
local attitude = {
  qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0,
  qInitialized = false,
}

function attitude.reset()
  attitude.qInitialized = false
  attitude.qx, attitude.qy, attitude.qz, attitude.qw = 0.0, 0.0, 0.0, 1.0
end

local function attitudeAxesFromQuat(qx, qy, qz, qw, outFwd, outLeft, outUp)
  local xx, yy, zz = qx * qx, qy * qy, qz * qz
  local xy, xz, yz = qx * qy, qx * qz, qy * qz
  local wx, wy, wz = qw * qx, qw * qy, qw * qz
  outFwd:set(1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy))
  outLeft:set(2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx))
  outUp:set(2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy))
  outFwd:normalize()
  outLeft:normalize()
  outUp:normalize()
end

-- Call after curl angVel. Overwrites tv.currentDir / worldLeft / worldThird.
local function attitudeStep(dt, tv)
  if filters.params.attitudeMode ~= 'integrate' then
    return
  end

  -- Snapshot measured triangle FLU for debug (velTri = v_world on these axes).
  if filters.params.debugRaw then
    tv.dirTriX:set(tv.currentDir)
    tv.dirTriY:set(tv.worldLeft)
    tv.dirTriZ:set(tv.worldThird)
  end

  local ax, ay, az, aw = quatFromAxes(tv.currentDir, tv.worldLeft, tv.worldThird)
  local a = attitude
  if not a.qInitialized then
    a.qx, a.qy, a.qz, a.qw = ax, ay, az, aw
    a.qInitialized = true
  else
    -- Predict: q_dot = 0.5 * ω_world ⊗ q  (use pre-update components on RHS).
    local qx, qy, qz, qw = a.qx, a.qy, a.qz, a.qw
    local ox, oy, oz = tv.angVel.x, tv.angVel.y, tv.angVel.z
    local dqX = 0.5 * (ox * qw + oy * qz - oz * qy)
    local dqY = 0.5 * (oy * qw + oz * qx - ox * qz)
    local dqZ = 0.5 * (oz * qw + ox * qy - oy * qx)
    local dqW = 0.5 * (-ox * qx - oy * qy - oz * qz)
    qx = qx + dt * dqX
    qy = qy + dt * dqY
    qz = qz + dt * dqZ
    qw = qw + dt * dqW
    -- Correct toward measured triangle quat (sign-aligned).
    local alpha = filters.cache.nominalAlpha.attitudeAbs or 1.0
    if qx * ax + qy * ay + qz * az + qw * aw < 0.0 then
      ax, ay, az, aw = -ax, -ay, -az, -aw
    end
    qx = qx + alpha * (ax - qx)
    qy = qy + alpha * (ay - qy)
    qz = qz + alpha * (az - qz)
    qw = qw + alpha * (aw - qw)
    local invN = 1.0 / (sqrt(qx * qx + qy * qy + qz * qz + qw * qw) + 1e-30)
    a.qx, a.qy, a.qz, a.qw = qx * invN, qy * invN, qz * invN, qw * invN
  end

  attitudeAxesFromQuat(a.qx, a.qy, a.qz, a.qw, tv.currentDir, tv.worldLeft, tv.worldThird)
end

local function update(dtSim)
  -- Cache frequently-accessed tables as locals for performance (reduces hash lookups)
  local rb = ringBuffer
  local sc = sensorConfig
  local vs = vehicleState
  local tv = tmpVectors
  local wi = wheelInfoTables
  
  -- Manage the update timer. Cycle it to avoid long-term drift.
  rb.physicsTimer = rb.physicsTimer + dtSim
  if rb.physicsTimer < rb.physicsUpdateTime then return end
  local dt = rb.physicsTimer
  rb.physicsTimer = rb.physicsTimer - rb.physicsUpdateTime

  -- Rebuild filter cache if dt has changed significantly 
  -- (e.g., due to lag or config changes).
  filters.rebuildCache(dt)

  -- Mark the beginning of a new write cycle.
  -- And move latestReading to point to the new slot for this physics step.
  -- TODO: Is it safe to move it now
  beginNewWrite(false)

  -- Compute the current position of the nodes defining the sensor wrt ref node.
  local node1 = vs.currVeh:getNodePosition(sc.nodeIndex1)
  local node2 = vs.currVeh:getNodePosition(sc.nodeIndex2)
  local node3 = vs.currVeh:getNodePosition(sc.nodeIndex3)

  -- Relvant edge vectors and normal vector of the triangle.
  tv.edge1:setSub2(node2, node1)
  tv.edge2:setSub2(node3, node1)
  tv.edge1Norm:set(tv.edge1)
  tv.edge1Norm:normalize()
  tv.edge2Norm:set(tv.edge2)
  tv.edge2Norm:normalize()
  tv.normal:setCross(tv.edge1Norm, tv.edge2Norm)
  tv.normal:normalize()

  -- Convert the fixed triangle-space coordinate system to world space
  tv.triangleThird:setCross(tv.edge1Norm, tv.normal)
  tv.triangleThird:normalize()

  tv.currentDir:setScaled2(tv.edge1Norm, sc.triangleSpaceForward.x)
  tv.vec1:setScaled2(tv.normal, sc.triangleSpaceForward.y)
  tv.currentDir:setAdd(tv.vec1)
  tv.vec1:setScaled2(tv.triangleThird, sc.triangleSpaceForward.z)
  tv.currentDir:setAdd(tv.vec1)
  tv.currentDir:normalize()

  tv.worldLeft:setScaled2(tv.edge1Norm, sc.triangleSpaceLeft.x)
  tv.vec1:setScaled2(tv.normal, sc.triangleSpaceLeft.y)
  tv.worldLeft:setAdd(tv.vec1)
  tv.vec1:setScaled2(tv.triangleThird, sc.triangleSpaceLeft.z)
  tv.worldLeft:setAdd(tv.vec1)
  tv.worldLeft:normalize()

  tv.worldThird:setCross(tv.currentDir, tv.worldLeft)
  tv.worldThird:normalize()

  -- Relative position of the sensor from the barycenter of the triangle.
  tv.vec1:setScaled2(tv.edge2, sc.b1)
  tv.vec2:setScaled2(tv.edge1, sc.b2)
  tv.vec3:setScaled2(tv.normal, sc.signedProjDist)
  tv.vec1:setAdd(tv.vec2)
  tv.vec1:setAdd(tv.vec3)
  tv.vec1:setAdd(node1) -- currentPos
  local currentPos = tv.vec1

  tv.sensorPos:setAdd2(currentPos, vs.currVeh:getPosition())

  -- --------------------------------------------------------------------------
  --                  [Angular] Velocity and Accelerations                   --
  -- --------------------------------------------------------------------------

  -- Compute the acceleration vectors at each node, using Newton II [a := F / m].
  tv.accel1:setScaled2(vs.currVeh:getNodeForceVector(sc.nodeIndex1), sc.inv_m1)
  tv.accel2:setScaled2(vs.currVeh:getNodeForceVector(sc.nodeIndex2), sc.inv_m2)
  tv.accel3:setScaled2(vs.currVeh:getNodeForceVector(sc.nodeIndex3), sc.inv_m3)

  -- Get the velocity vector at each node.
  local v1 = vs.currVeh:getNodeVelocityVector(sc.nodeIndex1)
  local v2 = vs.currVeh:getNodeVelocityVector(sc.nodeIndex2)
  local v3 = vs.currVeh:getNodeVelocityVector(sc.nodeIndex3)

  -- Rotational terms using reusable in-place temporaries.

  tv.aCenter:setAdd2(tv.accel1, tv.accel2)
  tv.aCenter:setAdd(tv.accel3)
  tv.aCenter:setScaled(sc.oneThird)

  tv.vCenter:setAdd2(v1, v2)
  tv.vCenter:setAdd(v3)
  tv.vCenter:setScaled(sc.oneThird)

  tv.baryCenter:setAdd2(node1, node2)
  tv.baryCenter:setAdd(node3)
  tv.baryCenter:setScaled(sc.oneThird)

  tv.r:setSub2(currentPos, tv.baryCenter)

  local denom = 0.0
  local divAcc = 0.0

  -- Accumulators for A = sum_i w_i * r_i * r_i^T (world frame, symmetric 3x3).
  -- Needed because the curl estimate below is biased when the attach-triangle
  -- normal is not aligned with the rotation axis: for a rigid triangle,
  --   curlVel = (denom * I - A) * omega
  -- so omega components in the triangle plane are attenuated and cross-coupled
  -- (observed as r being ~71% of truth on the utv, whose nearest attach
  -- triangle is strongly tilted; the sbr triangle happens to be ~flat).
  -- Solving the full 3x3 system recovers omega exactly (least-squares rigid
  -- fit, zero residual for 3 rigid points). Cost: ~70 flops/step, negligible.
  local a11, a12, a13, a22, a23, a33 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

  -- Node 1 contribution.
  tv.vec1:setSub2(node1, tv.baryCenter) -- r1
  a11 = a11 + sc.w1 * tv.vec1.x * tv.vec1.x
  a12 = a12 + sc.w1 * tv.vec1.x * tv.vec1.y
  a13 = a13 + sc.w1 * tv.vec1.x * tv.vec1.z
  a22 = a22 + sc.w1 * tv.vec1.y * tv.vec1.y
  a23 = a23 + sc.w1 * tv.vec1.y * tv.vec1.z
  a33 = a33 + sc.w1 * tv.vec1.z * tv.vec1.z
  tv.vec2:setSub2(tv.accel1, tv.aCenter) -- aRot1
  tv.vec3:setCross(tv.vec1, tv.vec2)
  tv.vec3:setScaled(sc.w1)
  tv.curlAcc:set(tv.vec3)
  divAcc = divAcc + tv.vec1:dot(tv.vec2) * sc.w1

  tv.vec2:setSub2(v1, tv.vCenter)       -- vRot1
  tv.vec3:setCross(tv.vec1, tv.vec2)
  tv.vec3:setScaled(sc.w1)
  tv.curlVel:set(tv.vec3)
  denom = denom + tv.vec1:squaredLength() * sc.w1

  -- Node 2 contribution.
  tv.vec1:setSub2(node2, tv.baryCenter) -- r2
  a11 = a11 + sc.w2 * tv.vec1.x * tv.vec1.x
  a12 = a12 + sc.w2 * tv.vec1.x * tv.vec1.y
  a13 = a13 + sc.w2 * tv.vec1.x * tv.vec1.z
  a22 = a22 + sc.w2 * tv.vec1.y * tv.vec1.y
  a23 = a23 + sc.w2 * tv.vec1.y * tv.vec1.z
  a33 = a33 + sc.w2 * tv.vec1.z * tv.vec1.z
  tv.vec2:setSub2(tv.accel2, tv.aCenter) -- aRot2
  tv.vec3:setCross(tv.vec1, tv.vec2)
  tv.vec3:setScaled(sc.w2)
  tv.curlAcc:setAdd(tv.vec3)
  divAcc = divAcc + tv.vec1:dot(tv.vec2) * sc.w2

  tv.vec2:setSub2(v2, tv.vCenter)       -- vRot2
  tv.vec3:setCross(tv.vec1, tv.vec2)
  tv.vec3:setScaled(sc.w2)
  tv.curlVel:setAdd(tv.vec3)
  denom = denom + tv.vec1:squaredLength() * sc.w2

  -- Node 3 contribution.
  tv.vec1:setSub2(node3, tv.baryCenter) -- r3
  a11 = a11 + sc.w3 * tv.vec1.x * tv.vec1.x
  a12 = a12 + sc.w3 * tv.vec1.x * tv.vec1.y
  a13 = a13 + sc.w3 * tv.vec1.x * tv.vec1.z
  a22 = a22 + sc.w3 * tv.vec1.y * tv.vec1.y
  a23 = a23 + sc.w3 * tv.vec1.y * tv.vec1.z
  a33 = a33 + sc.w3 * tv.vec1.z * tv.vec1.z
  tv.vec2:setSub2(tv.accel3, tv.aCenter) -- aRot3
  tv.vec3:setCross(tv.vec1, tv.vec2)
  tv.vec3:setScaled(sc.w3)
  tv.curlAcc:setAdd(tv.vec3)
  divAcc = divAcc + tv.vec1:dot(tv.vec2) * sc.w3

  tv.vec2:setSub2(v3, tv.vCenter)       -- vRot3
  tv.vec3:setCross(tv.vec1, tv.vec2)
  tv.vec3:setScaled(sc.w3)
  tv.curlVel:setAdd(tv.vec3)
  denom = denom + tv.vec1:squaredLength() * sc.w3

  local invDenom = 1.0 / (denom + 1e-30)

  -- total accel = aCenter + (curlAcc x r + divAcc * r) * invDenom
  -- NOTE: kept identical to BeamNG's advancedIMU reconstruction. Its
  -- rotational term uses the raw (uncorrected) curl; the dominant term is
  -- aCenter and the lever arm r is small, so we do not re-derive it here.
  tv.vec1:setCross(tv.curlAcc, tv.r)
  tv.vec2:setScaled2(tv.r, divAcc)
  tv.vec3:setAdd2(tv.vec1, tv.vec2)
  tv.vec3:setScaled(invDenom)
  tv.accelWorld:setAdd2(tv.aCenter, tv.vec3)

  -- Angular velocity/acceleration: solve M * omega = curlVel exactly, with
  -- M = denom * I - A (symmetric 3x3, world frame). The legacy estimate
  -- curlVel * invDenom drops the A term, i.e. assumes r_i . omega = 0, which
  -- only holds when the triangle normal is aligned with omega (flat triangle
  -- + pure yaw). M is invertible for any non-degenerate (non-collinear)
  -- triangle: eigenvalue denom along the normal, denom - lambda_{1,2} > 0
  -- in-plane. The det guard below is scale-invariant (det ~ denom^3) and only
  -- protects against a degenerate attach triangle -> fall back to the legacy
  -- biased estimate instead of producing NaNs.
  local m11, m22, m33 = denom - a11, denom - a22, denom - a33
  local m12, m13, m23 = -a12, -a13, -a23
  local det = m11 * (m22 * m33 - m23 * m23)
            - m12 * (m12 * m33 - m23 * m13)
            + m13 * (m12 * m23 - m22 * m13)
  if abs(det) > 1e-6 * denom * denom * denom then
    -- Inverse of symmetric M via cofactors (no allocations).
    local invDet = 1.0 / det
    local i11 = (m22 * m33 - m23 * m23) * invDet
    local i12 = (m13 * m23 - m12 * m33) * invDet
    local i13 = (m12 * m23 - m13 * m22) * invDet
    local i22 = (m11 * m33 - m13 * m13) * invDet
    local i23 = (m12 * m13 - m11 * m23) * invDet
    local i33 = (m11 * m22 - m12 * m12) * invDet
    local cvx, cvy, cvz = tv.curlVel.x, tv.curlVel.y, tv.curlVel.z
    tv.angVel:set(
      i11 * cvx + i12 * cvy + i13 * cvz,
      i12 * cvx + i22 * cvy + i23 * cvz,
      i13 * cvx + i23 * cvy + i33 * cvz
    )
    -- Same geometric bias affects the angular acceleration estimate.
    -- (curlAcc additionally contains centripetal cross-terms that this does
    -- not remove; that approximation is inherited from the stock advancedIMU.)
    local cax, cay, caz = tv.curlAcc.x, tv.curlAcc.y, tv.curlAcc.z
    tv.angAccel:set(
      i11 * cax + i12 * cay + i13 * caz,
      i12 * cax + i22 * cay + i23 * caz,
      i13 * cax + i23 * cay + i33 * caz
    )
  else
    -- Degenerate triangle: legacy estimate (biased but finite).
    tv.angVel:setScaled2(tv.curlVel, invDenom)
    tv.angAccel:setScaled2(tv.curlAcc, invDenom)
  end

  -- Published body frame (may replace triangle axes). Uses world angVel above
  -- for integrate mode; must run before report-offset transport / projections
  -- so pos/vel/accel/quat share one consistent FLU basis.
  attitudeStep(dt, tv)

  -- --------------------------------------------------------------------------
  --        Report-point transport (attach point -> report point)             --
  -- --------------------------------------------------------------------------
  -- When the sensor was attached away from the desired report point (e.g.
  -- attach_z_offset < 0 to select the stiff floor/frame triangle instead of a
  -- soft cabin panel), transport the point quantities back with the exact
  -- rigid-body relations, using the corrected omega/alpha (hence placed after
  -- the M^-1 solve above):
  --   pos_report = pos_attach + d
  --   v_report   = v_attach   + omega x d
  --   a_report   = a_attach   + alpha x d + omega x (omega x d)
  -- with d = reportOffset (constant in the sensor frame) rotated to world.
  -- Angular quantities and the quaternion are point-independent: unchanged.
  -- This is exact and lag-free (~4 cross products); the only approximation is
  -- that d is treated as rigid (chassis flex between the two points ignored).
  if sc.reportOffset then
    local ro = sc.reportOffset
    -- d (world) = ro.x * forward + ro.y * left + ro.z * up.
    tv.reportOffsetWorld:setScaled2(tv.currentDir, ro.x)
    tv.vec1:setScaled2(tv.worldLeft, ro.y)
    tv.reportOffsetWorld:setAdd(tv.vec1)
    tv.vec1:setScaled2(tv.worldThird, ro.z)
    tv.reportOffsetWorld:setAdd(tv.vec1)

    -- Position. NOTE: tv.sensorPos also feeds getSensorData().currentPos, so
    -- the debug sphere now visualises the REPORT point (e.g. the CoG), not
    -- the attach point on the mesh.
    tv.sensorPos:setAdd(tv.reportOffsetWorld)

    -- Acceleration (inertial, world frame; gravity handling happens below):
    -- Euler term (alpha x d) + centripetal term (omega x (omega x d)).
    tv.vec1:setCross(tv.angAccel, tv.reportOffsetWorld)
    tv.accelWorld:setAdd(tv.vec1)
    tv.vec1:setCross(tv.angVel, tv.reportOffsetWorld)
    tv.vec2:setCross(tv.angVel, tv.vec1)
    tv.accelWorld:setAdd(tv.vec2)
  end

  -- v_sensor = vCenter + ω × r (evaluated at the attach point; the report
  -- offset adds ω × d on top, see transport block above).
  tv.velSensorRaw:setCross(tv.angVel, tv.r)
  tv.velSensorRaw:setAdd(tv.vCenter)
  if sc.reportOffset then
    tv.vec1:setCross(tv.angVel, tv.reportOffsetWorld)
    tv.velSensorRaw:setAdd(tv.vec1)
  end
  local velWorld = filters.modules.velWorld.step(dt, tv.velSensorRaw)

  -- tv.accelWorld is the TRUE inertial acceleration v_dot (zero at rest: node
  -- forces already include gravity); convert to IMU-style specific force: f = a - g.
  tv.vec1:setSub2(tv.accelWorld, vs.currVeh:getGravityVector())
  -- Snapshot the raw specific force NOW for the debugRaw block: tv.vec1 is a
  -- shared scratch vector and gets overwritten by wheelAngleAtan2Rad() before
  -- the debug writes run (this used to make accelRaw log wheel-axis garbage).
  if filters.params.debugRaw then tv.accelSpecificRaw:set(tv.vec1) end
  local accelWorld = filters.modules.accelWorld.step(dt, tv.vec1)

  -- Cache latestReading locally for frequent writes (reduces hash lookups)
  local latest = rb.latestReading

  -- Transform quantities in the published local frame.
  latest.accel[1] = accelWorld:dot(tv.currentDir)
  latest.accel[2] = accelWorld:dot(tv.worldLeft)
  latest.accel[3] = accelWorld:dot(tv.worldThird)

  tv.angVelLocalRaw:set(tv.angVel:dot(tv.currentDir), tv.angVel:dot(tv.worldLeft), tv.angVel:dot(tv.worldThird))

  latest.vel[1] = velWorld:dot(tv.currentDir)
  latest.vel[2] = velWorld:dot(tv.worldLeft)
  latest.vel[3] = velWorld:dot(tv.worldThird)

  local angVelLocal = filters.modules.gyroLocal.step(dt, tv.angVelLocalRaw)
  latest.angVel[1] = angVelLocal.x
  latest.angVel[2] = angVelLocal.y
  latest.angVel[3] = angVelLocal.z

  latest.angAccel[1] = tv.angAccel:dot(tv.currentDir)
  latest.angAccel[2] = tv.angAccel:dot(tv.worldLeft)
  latest.angAccel[3] = tv.angAccel:dot(tv.worldThird)

  -- Quaternion: integrated state when available (matches ω kinematically).
  if filters.params.attitudeMode == 'integrate' and attitude.qInitialized then
    latest.quat[1] = attitude.qx
    latest.quat[2] = attitude.qy
    latest.quat[3] = attitude.qz
    latest.quat[4] = attitude.qw
  else
    latest.quat[1], latest.quat[2], latest.quat[3], latest.quat[4] =
      quatFromAxes(tv.currentDir, tv.worldLeft, tv.worldThird)
  end

  -- Derived kinematics (consumed by controller_manager control_state packet).
  local vx, vy = latest.vel[1], latest.vel[2]
  local V = sqrt(vx * vx + vy * vy)
  local qx, qy, qz, qw = latest.quat[1], latest.quat[2], latest.quat[3], latest.quat[4]
  local yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
  local pitch = asin(clamp(2 * (qw * qy - qz * qx), -1, 1))
  local roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
  local beta = abs(V) > 0.5 and atan2(vy, vx) or 0
  latest.V = V
  latest.yaw = yaw
  latest.pitch = pitch
  latest.roll = roll
  latest.beta = beta
  latest.Phi = beta + yaw

  -- ----------------------------------------------------------------------
  --                     WheelAngle Calculation                          --
  -- ----------------------------------------------------------------------

  -- front right
  fillWheelInfos(wi.fr, wheel_fr)
  wi.fr.angle = wheelAngleAtan2Rad(wheel_fr.node1, wheel_fr.node2)

  -- front left (note swapped order if needed)
  fillWheelInfos(wi.fl, wheel_fl)
  wi.fl.angle = wheelAngleAtan2Rad(wheel_fl.node2, wheel_fl.node1)

  -- rear right
  fillWheelInfos(wi.rr, wheel_rr)
  wi.rr.angle = wheelAngleAtan2Rad(wheel_rr.node1, wheel_rr.node2)

  -- rear left
  fillWheelInfos(wi.rl, wheel_rl)
  wi.rl.angle = wheelAngleAtan2Rad(wheel_rl.node2, wheel_rl.node1)

  -- Filter wheel angular velocities in-place (do not keep raw).
  filters.modules.wheelsAngVel.step(dt, wi.fr, wi.fl, wi.rr, wi.rl)
  --  -------------------------------------------------------

  -- Electrics (fresh every physics step; used for actuation + driveStatus logging).
  local elecVals = electrics.values

  -- Gather the latest reading data (in-place updates to reduce allocations).
  latest.time = vs.currVeh:getSimTime()

  writeVec3Table(latest.dirX, tv.currentDir)
  writeVec3Table(latest.dirY, tv.worldLeft)
  writeVec3Table(latest.pos, tv.sensorPos)

  -- Optional debug/raw fields in world frame.
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

  -- Engine relevant information
  latest.engineLoad = vs.engine and (vs.engine.isDisabled and 0 or vs.engine.instantEngineLoad) or 0
  latest.engineTorque = vs.engine and vs.engine.combustionTorque or 0
  latest.RPM = vs.engine and (vs.engine.outputAV1 * constants.avToRPM) or 0
  latest.flywheelTorque = vs.engine and vs.engine.outputTorque1 or 0
  latest.turboBoost = elecVals.turboBoost or -1
  latest.superchargerBoost = elecVals.superchargerBoost or -1
  latest.throttleValve = vs.engine and vs.engine.throttle

  -- Gearbox relevant information
  latest.gearboxTorque = vs.gearbox and vs.gearbox.outputTorque1 or 0
  latest.gearRatio = vs.gearbox and vs.gearbox.gearRatio or 0
  latest.gearIndex = elecVals.gearIndex

  -- Update fields for debugging non-filtered raw values if enabled.
  if filters.params.debugRaw then
    local g = vs.currVeh:getGravityVector()
    latest.gravityBody[1] = g:dot(tv.currentDir)
    latest.gravityBody[2] = g:dot(tv.worldLeft)
    latest.gravityBody[3] = g:dot(tv.worldThird)

    -- Unfiltered specific force (snapshot right after f = a - g; tv.vec1 is
    -- stale here — clobbered by wheel-angle helpers).
    latest.accelRaw[1] = tv.accelSpecificRaw:dot(tv.currentDir)
    latest.accelRaw[2] = tv.accelSpecificRaw:dot(tv.worldLeft)
    latest.accelRaw[3] = tv.accelSpecificRaw:dot(tv.worldThird)

    latest.angVelRaw[1] = tv.angVelLocalRaw.x
    latest.angVelRaw[2] = tv.angVelLocalRaw.y
    latest.angVelRaw[3] = tv.angVelLocalRaw.z

    -- Legacy curlVel/denom (no M^-1); vs angVelRaw isolates tilted-triangle bias.
    latest.angVelUncorr[1] = tv.curlVel:dot(tv.currentDir) * invDenom
    latest.angVelUncorr[2] = tv.curlVel:dot(tv.worldLeft) * invDenom
    latest.angVelUncorr[3] = tv.curlVel:dot(tv.worldThird) * invDenom

    -- Engine all-node p,q,r (refNode frame) — offline cross-check only.
    local rollAV, pitchAV, yawAV = vs.currVeh:getRollPitchYawAngularVelocity()
    latest.angVelObjRPY[1] = rollAV
    latest.angVelObjRPY[2] = pitchAV
    latest.angVelObjRPY[3] = yawAV

    latest.velRaw[1] = tv.velSensorRaw:dot(tv.currentDir)
    latest.velRaw[2] = tv.velSensorRaw:dot(tv.worldLeft)
    latest.velRaw[3] = tv.velSensorRaw:dot(tv.worldThird)

    -- Triangle axes / triangle-projected vel (legacy chatty body vs published).
    if filters.params.attitudeMode ~= nil and filters.params.attitudeMode ~= 'triangle' then
      writeVec3Table(latest.dirXTri, tv.dirTriX)
      writeVec3Table(latest.dirYTri, tv.dirTriY)
      latest.velTri[1] = tv.velSensorRaw:dot(tv.dirTriX)
      latest.velTri[2] = tv.velSensorRaw:dot(tv.dirTriY)
      latest.velTri[3] = tv.velSensorRaw:dot(tv.dirTriZ)
    else
      writeVec3Table(latest.dirXTri, tv.currentDir)
      writeVec3Table(latest.dirYTri, tv.worldLeft)
      latest.velTri[1] = latest.velRaw[1]
      latest.velTri[2] = latest.velRaw[2]
      latest.velTri[3] = latest.velRaw[3]
    end

    latest.wheelFR_angVelRaw = filters.state.wheelsAngVel.rawFr
    latest.wheelFL_angVelRaw = filters.state.wheelsAngVel.rawFl
    latest.wheelRR_angVelRaw = filters.state.wheelsAngVel.rawRr
    latest.wheelRL_angVelRaw = filters.state.wheelsAngVel.rawRl

    local signSteering = sign(elecVals.steering_input)
    latest.wheelFR_angleLegacy = wheelAngleLegacyRad(wheel_fr.node1, wheel_fr.node2, signSteering)
    latest.wheelFL_angleLegacy = wheelAngleLegacyRad(wheel_fl.node2, wheel_fl.node1, signSteering)
    latest.wheelRR_angleLegacy = wheelAngleLegacyRad(wheel_rr.node1, wheel_rr.node2, signSteering)
    latest.wheelRL_angleLegacy = wheelAngleLegacyRad(wheel_rl.node2, wheel_rl.node1, signSteering)
  end

  -- Policy forward torque estimate + envelope (lib attached by controller_llc).
  if torqueMapEnabled and torquePolicyLib then
    local rear_wheelspeed_ms = 0.5 * (wi.rr.speed + wi.rl.speed)
    local engine_speed_rads = latest.RPM * constants.rpmToAV
    local boost_pressure = latest.turboBoost
    local throttle = latest.throttle
    local est = tonumber(torquePolicyLib.drivetrain_forward_torque(
      engine_speed_rads, boost_pressure, rear_wheelspeed_ms, throttle
    ))
    local t0 = tonumber(torquePolicyLib.drivetrain_forward_torque(
      engine_speed_rads, boost_pressure, rear_wheelspeed_ms, 0
    ))
    local t1 = tonumber(torquePolicyLib.drivetrain_forward_torque(
      engine_speed_rads, boost_pressure, rear_wheelspeed_ms, 1
    ))
    setCustomField(torqueMapFieldName, est)
    latest.torque_min = min(t0, t1)
    latest.torque_max = max(t0, t1)
  else
    latest.torque_min = 0
    latest.torque_max = 0
  end

  -- Add the custom fields
  for fieldName, value in pairs(customFields) do
    latest[fieldName] = value
  end

  -- Now move the index
  beginNewWrite(true)

  -- Bootstrap ring schema from first fully-populated reading.
  if not rb.ringInitialized then
    bootstrapRingFromFirstReading()
  end

  -- Store the latest readings for this State sensor in the extension. 
  -- This is used for sending back on the physics step.
  gtStateManager.cacheLatestReading(sc.id, latest)

end

--[[
Initializes a vehicle state sensor instance with barycentric positioning
on a triangular surface element and configures its operational parameters.

Parameters:
  data : table - Configuration table

Dependencies:
  - Requires global 'wheelRotators' and 'wheelIds' tables for wheel references
  - Relies on 'extensions.xlab_gtState' for state management
  - Assumes 'obj' exists in parent scope with vehicle methods
]]
local function init(data)
  -- Validate critical input parameters
  assert(
    data.nodeIndex1 and data.nodeIndex2 and data.nodeIndex3,
    'Missing node indices in sensor configuration'
  )
  assert(data.u and data.v, 'Missing barycentric coordinates in config')
  -- assert(
  --   (data.u + data.v) <= 1 + 1e-6,
  --   'Invalid barycentric coordinates (u + v must be <= 1, got ' .. data.u .. ', ' .. data.v .. ')'
  -- )

  -- Initialize extension integration
  gtStateManager = extensions.xlab_gtState

  -- Core sensor configuration
  sensorConfig.id = data.sensorId
  sensorConfig.gfxUpdateTime = data.GFXUpdateTime or 0.033 -- Default to ~30Hz

  -- Node indices for triangular mounting surface
  sensorConfig.nodeIndex1 = data.nodeIndex1
  sensorConfig.nodeIndex2 = data.nodeIndex2
  sensorConfig.nodeIndex3 = data.nodeIndex3

  -- Set the vehicle object
  vehicleState.currVeh = obj

  -- Masses of the three nodes
  sensorConfig.m1 = vehicleState.currVeh:getNodeMass(sensorConfig.nodeIndex1)
  sensorConfig.m2 = vehicleState.currVeh:getNodeMass(sensorConfig.nodeIndex2)
  sensorConfig.m3 = vehicleState.currVeh:getNodeMass(sensorConfig.nodeIndex3)
  sensorConfig.inv_m1 = 1.0 / sensorConfig.m1
  sensorConfig.inv_m2 = 1.0 / sensorConfig.m2
  sensorConfig.inv_m3 = 1.0 / sensorConfig.m3

  -- Barycentric coordinates and interpolation weights
  sensorConfig.b1 = data.u
  sensorConfig.b2 = data.v
  sensorConfig.b3 = 1.0 - sensorConfig.b1 - sensorConfig.b2

  -- Non-negative weights for mass interpolation
  sensorConfig.w1 = max(0, sensorConfig.b1)
  sensorConfig.w2 = max(0, sensorConfig.b2)
  sensorConfig.w3 = max(0, sensorConfig.b3)

  -- Spatial configuration
  sensorConfig.signedProjDist = data.signedProjDist
  sensorConfig.triangleSpaceForward = data.triangleSpaceForward
  sensorConfig.triangleSpaceLeft = data.triangleSpaceLeft

  -- Report-point offset (sensor FLU frame): {x=fwd, y=left, z=up} vector from
  -- the attach point to the point where pos/vel/accel must be reported
  -- (typically back up to the CoG when attach_z_offset was used to pick a
  -- stiffer attach triangle). Disabled (nil) when absent or ~zero, so the
  -- default configuration is bit-identical to the previous behaviour.
  sensorConfig.reportOffset = nil
  if data.reportOffset ~= nil then
    local ox = tonumber(data.reportOffset[1]) or 0.0
    local oy = tonumber(data.reportOffset[2]) or 0.0
    local oz = tonumber(data.reportOffset[3]) or 0.0
    if (ox * ox + oy * oy + oz * oz) > 1e-12 then
      sensorConfig.reportOffset = vec3(ox, oy, oz)
      log('I', logTag, string.format(
        'Report-point transport enabled | offset (sensor FLU) = [%.3f, %.3f, %.3f] m',
        ox, oy, oz))
    end
  end

  -- Timing configuration
  ringBuffer.physicsUpdateTime = data.physicsUpdateTime or 0.005 -- Default to 200Hz
  ringBuffer.numPhysicsStepsForGFXSave = max(1, tonumber(data.numPhysicsStepsForGFXSave) or 1)

  -- Optional filter configuration
  filters.params.accelTauS = tonumber(data.accelTauS or data.accel_tau_s) or filters.params.accelTauS
  filters.params.gyroTauS = tonumber(data.gyroTauS or data.gyro_tau_s) or filters.params.gyroTauS
  filters.params.velTauS = tonumber(data.velTauS or data.vel_tau_s) or filters.params.velTauS
  filters.params.wheelAngVelTauS = tonumber(data.wheelAngVelTauS or data.wheel_angvel_tau_s) or filters.params.wheelAngVelTauS
  local attMode = data.attitudeMode or data.attitude_mode
  if type(attMode) == 'string' and attMode ~= '' then
    filters.params.attitudeMode = attMode
  end
  filters.params.attitudeTauS = tonumber(data.attitudeTauS or data.attitude_tau_s) or filters.params.attitudeTauS
  filters.params.debugRaw = (data.debugRaw == true) or (data.debug == true) or (data.debug_raw == true)

  log('I', logTag, string.format(
    'Filter config | accelTauS=%.3f gyroTauS=%.3f velTauS=%.3f wheelAngVelTauS=%.3f attitudeMode=%s attitudeTauS=%.3f debugRaw=%s',
    filters.params.accelTauS,
    filters.params.gyroTauS,
    filters.params.velTauS,
    filters.params.wheelAngVelTauS,
    tostring(filters.params.attitudeMode),
    filters.params.attitudeTauS,
    tostring(filters.params.debugRaw)
  ))

  -- Cache nominal alphas for the common dt path.
  filters.cache.nominalDt = 0 -- default physics update time
  filters.rebuildCache(ringBuffer.physicsUpdateTime)

  -- Reset all filter / attitude states for this sensor.
  filters.resetAll()
  attitude.reset()

  -- Initialize timing state
  ringBuffer.physicsTimer = 0.0

  -- Wheel system integration
  wheel_fr = wheelRotators[wheelIds['FR']]
  wheel_fl = wheelRotators[wheelIds['FL']]
  wheel_rr = wheelRotators[wheelIds['RR']]
  wheel_rl = wheelRotators[wheelIds['RL']]

  -- Engine and gearbox references
  vehicleState.engine = powertrain.getDevice('mainEngine')
  vehicleState.gearbox = powertrain.getDevice('gearbox')

  -- Let's make sure that they are not nil
  if vehicleState.engine == nil then log('E', logTag, 'Engine reference is nil') end
  if vehicleState.gearbox == nil then log('E', logTag, 'Gearbox reference is nil') end

  -- Forward torque estimate enabled; policy lib attached later by controller_llc.
  torquePolicyLib = nil
  torqueMapEnabled = false
  local torqueMapCfg = data.torque_map
  if torqueMapCfg ~= nil then
    torqueMapEnabled = true
    torqueMapFieldName = torqueMapCfg.field_name or 'rear_wheel_torque_est'
    if customFields[torqueMapFieldName] == nil then
      registerCustomField(torqueMapFieldName, 0)
    end
    log(
      'I',
      logTag,
      'torque_map estimate enabled (awaiting LLC torque map) field='
        .. tostring(torqueMapFieldName)
    )
  end

  ringBuffer.readings = {}
  ringBuffer.ringSize = computeRingSize(sensorConfig.gfxUpdateTime, ringBuffer.physicsUpdateTime)
  ringBuffer.writeIdx = 0
  ringBuffer.writeSeq = 0
  ringBuffer.readSeq = 0
  ringBuffer.ringInitialized = false

  -- Just initialize the latestReading to an empty table for now.
  ensureLatestReadingTables()

  -- Debug initialization
  log(
    'I',
    logTag,
    string.format(
      'Initialized sensor %d | Nodes: %d,%d,%d | Update rates: Physics=%.1fkHz GFX=%.1fHz | StepsPerGFX=%d | RingSize=%d',
      sensorConfig.id,
      sensorConfig.nodeIndex1,
      sensorConfig.nodeIndex2,
      sensorConfig.nodeIndex3,
      1 / ringBuffer.physicsUpdateTime / 1000,
      1 / sensorConfig.gfxUpdateTime,
      ringBuffer.numPhysicsStepsForGFXSave,
      ringBuffer.ringSize
    )
  )
end


local function stop()
  torquePolicyLib = nil
  torqueMapEnabled = false
end

--[[
Retrieves comprehensive sensor data for external use, such as graphical updates or external systems.

Returns:
    table: A table containing the following fields:
        - currentPos (vec3): The sensor's current position in world space, calculated by adding
                                the sensor's local position to the vehicle's global position.
        - rawReadings (table): A table of raw sensor readings accumulated since the last graphics update.
]]
local function getSensorData()
  return {
    currentPos = tmpVectors.sensorPos,
    rawReadings = getPendingGFXReadings(),
  }
end

--[[
Retrieves the most recent sensor reading.

Returns:
    table: A table containing the latest sensor data, including time, orientation, velocity,
            acceleration, angular dynamics, position, orientation quaternion, wheel states,
            control inputs, desired inputs, and drive mode status, ..
]]
local function getLatest() return ringBuffer.latestReading end


-- Public interface:
M.update = update
M.init = init
M.stop = stop
M.getSensorData = getSensorData
M.getLatest = getLatest
M.registerCustomField = registerCustomField
M.setCustomField = setCustomField
M.setTorqueMapLib = setTorqueMapLib

return M