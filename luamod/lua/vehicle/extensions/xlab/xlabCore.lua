--[[
    xLab Vehicle Control Module
    Version: 1.2
    Provides direct control for vehicle experiments
    Log Tag: XalVE
]]

local M = {}
local logTag = 'XalVE'

local function table_contains(tbl, element)
  for _, value in pairs(tbl) do
    if value == element then return true end
  end
  return false
end

--[[
    Local function to set ABS state
    @param enabled: boolean - Whether ABS should be enabled
    @return: nil
]]
local function setABS(enabled)
  -- Set ABS behavior
  local mode = enabled and 'realistic' or 'off'
  wheels.setABSBehavior(mode)
  log('I', logTag, 'Attempting to set ABS to ' .. mode)
end

--[[
    Get current ABS setup
    @return: table - ABS configuration including enabled state and behavior
]]
local function getABS() return { hasAbs = electrics.values.hasABS == 1 } end

--[[
    Local function to set ESC state
    @param enabled: boolean - Whether ESC should be enabled
    @return: nil
]]
local function setESC(enabled)
  -- Check if ESC controller exists
  if not esc or not esc.getCurrentConfigData then
    log('E', logTag, 'ESC controller not found')
    return
  end

  -- Get current state
  local escConfig = esc:getCurrentConfigData()
  local currentState = escConfig.escEnabled

  -- Only toggle if needed
  local max_attempts = 5
  local attempts = 0
  while currentState ~= enabled and attempts < max_attempts do
    esc:toggleESCMode()
    log('I', logTag, 'Attempting to set ESC to ' .. tostring(enabled))
    escConfig = esc:getCurrentConfigData()
    currentState = escConfig.escEnabled
    attempts = attempts + 1
  end

  if attempts >= max_attempts then
    log('E', logTag, 'Failed to set ESC to ' .. tostring(enabled))
  end
end

--[[
    Get current ESC configuration
    @return: table - ESC configuration including enabled state and settings
]]
local function getESC()
  if not esc or not esc.getCurrentConfigData then return { enabled = false, hasESC = false } end

  local escConfig = esc:getCurrentConfigData()
  return { enabled = escConfig.escEnabled, hasESC = true }
end

--[[
    Local function to set 4WD mode and/or range box mode
    @param mode: string - 4WD mode to set ('2WD', '4WD')
    @param rangeMode: string - Range box mode to set ('low', 'high')
    @return: nil
]]
local function set4wdMode(mode, rangeMode)
  -- Get 4wd controller
  local ctrl = controller.getController('4wd')
  if not ctrl then
    log('E', logTag, '4WD controller not found')
    return
  end

  -- Set 4WD mode if provided
  if mode then
    local powertrainMode = mode == '2WD' and 'unlocked' or 'locked'
    log('I', logTag, 'Setting 4WD mode to: ' .. mode)
    ctrl.set4WDModeNew(powertrainMode)
  end

  -- Set range box mode if provided
  if rangeMode then
    log('I', logTag, 'Setting range box mode to: ' .. rangeMode)
    ctrl.setRangeModeNew(rangeMode)
  end
end

--[[
    Get current 4WD configuration
    @return: table - 4WD configuration including mode, range box state, and capability
]]
local function get4wdMode()
  local is4wdCapable = controller.getController('4wd') ~= nil
  if not is4wdCapable then return { mode = 'N/A', range = 'N/A', is4wdCapable = false } end
  -- Let's log electrics values
  local mode = (electrics.values.mode4WD == 1) and '4WD' or '2WD'
  local range = (electrics.values.modeRangeBox == 1) and 'low' or 'high'
  return { mode = mode, range = range, is4wdCapable = is4wdCapable }
end

--[[
    Helper function to find differential device by type
    @param diff: string - Differential type ('front', 'rear')
    @return: table|nil - Differential device or nil if not found
]]
local function findDifferentialDevice(diff)
  -- Get all differential devices
  local diffs = powertrain.getDevicesByType('differential')
  if not diffs or #diffs == 0 then
    log('E', logTag, 'No differential devices found')
    return nil
  end

  -- Find the correct differential based on suffix
  local suffix = diff == 'front' and 'F' or 'R'
  for _, d in ipairs(diffs) do
    if string.sub(d.name, -1) == suffix then return d end
  end

  log('E', logTag, 'Could not find ' .. diff .. ' differential')
  return nil
end

--[[
    Local function to lock/unlock differential
    @param diff: string - Differential to control ('front', 'rear')
    @param lock: boolean - Whether to lock the differential
    @return: nil
]]
local function lockDiff(diff, lock)
  -- Find the target differential
  local targetDiff = findDifferentialDevice(diff)
  if not targetDiff then return end

  -- Check available modes
  local modes = targetDiff.availableModes
  if not modes or #modes == 0 then
    log('E', logTag, 'No modes available for differential: ' .. targetDiff.name)
    return
  end

  -- Handle locking/unlocking
  if #modes <= 1 then
    -- If 0 or 1 modes, nothing to change
    log('I', logTag, 'Differential ' .. targetDiff.name .. ' has only one mode, cannot change')
    return
  end

  -- If 2+ modes, handle lock/unlock specifically
  if lock then
    -- Check if locked mode exists
    if table_contains(modes, 'locked') then
      powertrain.setDeviceMode(targetDiff.name, 'locked')
      log('I', logTag, 'Locked ' .. diff .. ' differential: ' .. targetDiff.name)
    else
      log('E', logTag, 'Locked mode not available for differential: ' .. targetDiff.name)
    end
  else
    -- Find first non-locked mode
    local newMode
    for _, mode in ipairs(modes) do
      if mode ~= 'locked' then
        newMode = mode
        break
      end
    end

    powertrain.setDeviceMode(targetDiff.name, newMode)
    log('I', logTag, 'Unlocked ' .. diff .. ' differential: ' .. targetDiff.name)
  end
end

--[[
    Get differential lock state
    @param diff: string - Differential to check ('front', 'rear')
    @return: boolean - Whether differential is locked
]]
local function getDiffLockState(diff)
  local targetDiff = findDifferentialDevice(diff)
  if not targetDiff then
    log('E', logTag, 'Could not find ' .. diff .. ' differential')
    return { locked = false, mode = 'unknown' }
  end
  local currentMode = targetDiff.mode
  return { locked = currentMode == 'locked', mode = currentMode }
end

--[[
Local function to set gearbox mode
@param gearIndex: number - Gear index to shift to
@return: nil
]]
local function setGearboxIndex(gearIndex)
  if not controller or not controller.mainController then
    log('E', logTag, 'Main controller not found')
    return
  end

  controller.mainController.shiftToGearIndex(gearIndex)
  log('I', logTag, 'Shifted to gear index: ' .. gearIndex)
end

--[[
Local function to get gearbox information
@return: table - Gearbox information including current gear index and available gears
]]
local function getGearboxInfo()
  if not controller or not controller.mainController then
    log('E', logTag, 'Main controller not found')
    return { currentGearIndex = -1 }
  end

  -- Assume these electrics values are always available
  local minGearIndex = electrics.values.minGearIndex
  local maxGearIndex = electrics.values.maxGearIndex
  local gearIndex = electrics.values.gearIndex
  local gearModeIndex = electrics.values.gearModeIndex
  local gearName = electrics.values.gear
  local gearbox = powertrain.getDevice('gearbox')
  return {
    minGearIndex = minGearIndex,
    maxGearIndex = maxGearIndex,
    gearIndex = gearIndex,
    gearModeIndex = gearModeIndex,
    gearName = gearName,
    gearRatio = gearbox.gearRatio,
    mode = gearbox.mode and gearbox.mode or '',
  }
end

--[[
    Converts a vector from BeamNG's Left-Back-Up (LBU) coordinate system 
    to the vehicle's Front-Left-Up (FLU) coordinate system
    @param v: vec3 - Input vector in LBU space
    @param vDir: vec3 - Vehicle forward direction vector
    @param vLeft: vec3 - Vehicle left direction vector
    @param vUp: vec3 - Vehicle up direction vector
    @return: vec3 - Converted vector in FLU space
]]
local function convertLBUtoFLU(v, vDir, vLeft, vUp)
  return vec3(
    v:dot(vDir), -- X component (forward)
    v:dot(vLeft), -- Y component (left)
    v:dot(vUp) -- Z component (up)
  )
end

--[[
    Calculates the mass of a wheel by summing its node weights
    @param wheelObj: table - Wheel object from BeamNG API
    @return: number - Total mass of the wheel in kg
]]
local function getWheelMass(wheelObj)
  local mass = 0
  for _, nodeId in ipairs(wheelObj.nodes) do
    local node = v.data.nodes[nodeId]
    if node and node.nodeWeight then mass = mass + node.nodeWeight end
  end
  return mass
end

--[[
    Live node position in the ref-node frame (same as obj:getNodePosition).
    Copied immediately: BeamNG may reuse the returned vec3.
]]
local function nodePos(cid)
  local p = obj:getNodePosition(cid)
  return vec3(p.x, p.y, p.z)
end

--[[
    Official wet COM in the ref-node frame, including wheels and fuel.
    false = include unsprung (wheels).
]]
local function wetCogRel()
  local c = obj:calcCenterOfGravityRel(false)
  return vec3(c.x, c.y, c.z)
end

local function wetNodeMass(node)
  if node and node.cid then return obj:getNodeMass(node.cid) or 0 end
  return 0
end

local function wetTotalMass()
  local totalMass = 0
  for _, node in pairs(v.data.nodes) do
    totalMass = totalMass + wetNodeMass(node)
  end
  return totalMass
end

local function principalAxes()
  local refNodes = v.data.refNodes[0]
  local nodeRef = v.data.nodes[refNodes.ref]
  local nodeBack = v.data.nodes[refNodes.back]
  local nodeUp = v.data.nodes[refNodes.up]
  local refNodePos = nodePos(nodeRef.cid)
  local backNodePos = nodePos(nodeBack.cid)
  local upNodePos = nodePos(nodeUp.cid)
  local vectorForward = (refNodePos - backNodePos):normalized()
  local vectorUp = (upNodePos - refNodePos):normalized()
  local vectorLeft = vectorUp:cross(vectorForward):normalized()
  return vectorForward, vectorLeft, vectorUp
end

local function wheelCenter(wd)
  return (nodePos(wd.node1) + nodePos(wd.node2)) * 0.5
end

-- Catalog I_w prior from this wheel's geometry. Generic densities, not JBeam mass.
local function wheelInertiaPrior(wd)
  local R = wd.radius
  if not R or R < 0.08 then return nil end
  local W = (wd.tireWidth and wd.tireWidth > 0.05) and wd.tireWidth or (0.55 * R)
  local r_rim = (wd.hubRadius and wd.hubRadius > 0.05) and wd.hubRadius or (0.48 * R)
  local r_rot = (wd.brakeDiameter and wd.brakeDiameter > 0.1) and (0.5 * wd.brakeDiameter) or (0.38 * R)
  local m_tire = 2 * math.pi * R * W * 0.022 * 1100
  local m_rim = 280 * r_rim * r_rim
  local m_hw = wd.brakeMass or (50 * R * R)
  return m_tire * (0.80 * R) ^ 2 + 0.50 * m_rim * r_rim * r_rim + 0.50 * m_hw * r_rot * r_rot
end

--[[
    Body triad for steer: x = wheelbase (front-axle mid → rear-axle mid)
    in the body horizontal plane. Not ref→back, not gravity.
]]
local function bodySteerAxes(frontMid, rearMid, bodyUp)
  local forward = frontMid - rearMid
  forward = forward - bodyUp * forward:dot(bodyUp)
  if forward:squaredLength() < 1e-12 then
    forward = frontMid - rearMid
  end
  forward = forward:normalized()
  local left = bodyUp:cross(forward)
  if left:squaredLength() < 1e-12 then
    local _, bodyLeft = principalAxes()
    left = bodyLeft
  else
    left = left:normalized()
  end
  return forward, left, bodyUp
end

--[[
    δ = wheel heading vs the body (body horizontal plane).
    forward = wheelbase, left = bodyUp × forward, same atan2 / node order.
]]
local function wheelHeadingRad(nodeA, nodeB, forward, left, up)
  local axis = nodePos(nodeB) - nodePos(nodeA)
  local roll = axis:cross(up)
  if roll:squaredLength() < 1e-12 then return 0 end
  if roll:dot(forward) < 0 then roll = -roll end
  return math.atan2(roll:dot(left), roll:dot(forward))
end

local function frontRoadwheelSteer()
  local _, _, bodyUp = principalAxes()
  local wRotators = wheels.wheelRotators
  local wIds = wheels.wheelRotatorIDs
  local fr = wRotators[wIds.FR]
  local fl = wRotators[wIds.FL]
  local rr = wRotators[wIds.RR]
  local rl = wRotators[wIds.RL]
  local frontMid = (wheelCenter(fl) + wheelCenter(fr)) * 0.5
  local rearMid = (wheelCenter(rl) + wheelCenter(rr)) * 0.5
  local forward, left, up = bodySteerAxes(frontMid, rearMid, bodyUp)
  local delta_r = wheelHeadingRad(fr.node1, fr.node2, forward, left, up)
  local delta_l = wheelHeadingRad(fl.node2, fl.node1, forward, left, up)
  return {
    delta_l = delta_l,
    delta_r = delta_r,
    delta = 0.5 * (delta_l + delta_r),
    steering_input = electrics.values.steering_input,
  }
end

local function vehicleMotion()
  local vx, vy, vz, speed = 0, 0, 0, 0
  local ok, vel = pcall(function()
    return obj:getVelocity()
  end)
  if ok and vel and vel.x then
    vx, vy, vz = vel.x, vel.y, vel.z
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
  else
    ok, vel = pcall(function()
      return obj:getSmoothRefVelocityXYZ()
    end)
    if ok and vel then
      if type(vel) == 'number' then
        -- some builds return vx,vy,vz as multiple returns; first value only here
        vx = vel
      elseif vel.x then
        vx, vy, vz = vel.x, vel.y, vel.z
      end
      speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    end
  end
  local yaw = obj.getYawAngularVelocity and obj:getYawAngularVelocity() or 0
  local pitch = obj.getPitchAngularVelocity and obj:getPitchAngularVelocity() or 0
  local roll = obj.getRollAngularVelocity and obj:getRollAngularVelocity() or 0
  local omega = math.sqrt(yaw * yaw + pitch * pitch + roll * roll)
  local cogWorld = obj:calcCenterOfGravity(false)
  return {
    speed = speed,
    omega = omega,
    cogZ = cogWorld and cogWorld.z or 0,
    vel = { vx, vy, vz },
  }
end

--[[
    Calculates diagonal inertia component for a given axis (wet, live poses).
    @param axis_id: number - 1(X), 2(Y), or 3(Z)
    @param cog: vec3 - Center of gravity in the ref-node frame
]]
local function getInertiaOnDiag(axis_id, cog, vD, vL, vU)
  local inertia = 0
  local cogFLU = convertLBUtoFLU(cog, vD, vL, vU)
  local axis_mask = { 1, 1, 1 }
  axis_mask[axis_id] = 0
  local cogAdjusted =
    vec3(cogFLU.x * axis_mask[1], cogFLU.y * axis_mask[2], cogFLU.z * axis_mask[3])

  for _, node in pairs(v.data.nodes) do
    local mass = wetNodeMass(node)
    if mass > 0 then
      local posFLU = convertLBUtoFLU(nodePos(node.cid), vD, vL, vU)
      local posAdjusted =
        vec3(posFLU.x * axis_mask[1], posFLU.y * axis_mask[2], posFLU.z * axis_mask[3])
      local delta = posAdjusted - cogAdjusted
      local deltaLength = delta:length()
      inertia = inertia + mass * deltaLength * deltaLength
    end
  end
  return inertia
end

--[[
    Calculates cross inertia component between two axes (wet, live poses).
]]
local function getCrossInertia(axis1, axis2, cog, vD, vL, vU)
  local inertia = 0
  local cogFLU = convertLBUtoFLU(cog, vD, vL, vU)
  for _, node in pairs(v.data.nodes) do
    local mass = wetNodeMass(node)
    if mass > 0 then
      local posFLU = convertLBUtoFLU(nodePos(node.cid), vD, vL, vU)
      local delta = posFLU - cogFLU
      local components = { delta.x, delta.y, delta.z }
      inertia = inertia + mass * components[axis1] * components[axis2]
    end
  end
  return inertia
end

--[[
    Body-frame bicycle / LLC plant, measured live in the ref-node frame.
    Call after the vehicle has settled. No world poses.
]]
local function getVehicleProperties(_props)
  local vectorForward, vectorLeft, vectorUp = principalAxes()
  local cogRel = wetCogRel()
  local totalMass = wetTotalMass()

  local wRotators = wheels.wheelRotators
  local wIds = wheels.wheelRotatorIDs
  local wheelsData = {
    FR = wRotators[wIds.FR],
    FL = wRotators[wIds.FL],
    RR = wRotators[wIds.RR],
    RL = wRotators[wIds.RL],
  }

  local cFL = wheelCenter(wheelsData.FL)
  local cFR = wheelCenter(wheelsData.FR)
  local cRL = wheelCenter(wheelsData.RL)
  local cRR = wheelCenter(wheelsData.RR)
  local frontAxle = (cFL + cFR) * 0.5
  local rearAxle = (cRL + cRR) * 0.5
  -- Bicycle x = wheelbase in the body plane (same triad as δ).
  local steerFwd, steerLeft, steerUp = bodySteerAxes(frontAxle, rearAxle, vectorUp)

  local a = math.abs((frontAxle - cogRel):dot(steerFwd))
  local b = math.abs((rearAxle - cogRel):dot(steerFwd))
  local L = a + b
  -- CoG off the vehicle centerline (front/rear axle mids). +left, +up from axle.
  local axleMid = (frontAxle + rearAxle) * 0.5
  local dCog = cogRel - axleMid
  local cogToCentralAxle = dCog:dot(steerLeft)
  local cogAboveAxle = dCog:dot(steerUp)
  local trackFront = (cFL - cFR):length()
  local trackRear = (cRL - cRR):length()

  local sumDyn, sumNom, sumIw, sumIwSim, nW, nIw, nIwSim = 0, 0, 0, 0, 0, 0, 0
  for _, wd in pairs(wheelsData) do
    if wd.hasTire ~= false then
      sumDyn = sumDyn + (wd.dynamicRadius or wd.radius or 0)
      sumNom = sumNom + (wd.radius or 0)
      nW = nW + 1
      local iPrior = wheelInertiaPrior(wd)
      if iPrior then
        sumIw = sumIw + iPrior
        nIw = nIw + 1
      end
      if wd.inertia then
        sumIwSim = sumIwSim + wd.inertia
        nIwSim = nIwSim + 1
      end
    end
  end
  local wheelRadius = nW > 0 and (sumDyn / nW) or 0
  local wheelRadiusNominal = nW > 0 and (sumNom / nW) or 0
  local wheelInertia = nIw > 0 and (sumIw / nIw) or nil
  local wheelInertiaSim = nIwSim > 0 and (sumIwSim / nIwSim) or nil

  -- h along body z: mean (COM − contact) · bodyUp. Contact is tread node or hub − R.
  local sumH, nH = 0, 0
  for _, wd in pairs(wheelsData) do
    local nid = wd.lastTreadContactNode or wd.treadContactNode or wd.contactNode
    local contact
    if type(nid) == 'number' then
      contact = nodePos(nid)
    else
      contact = wheelCenter(wd) - vectorUp * (wd.dynamicRadius or wd.radius or 0)
    end
    sumH = sumH + (cogRel - contact):dot(vectorUp)
    nH = nH + 1
  end
  local coGHeight = nH > 0 and (sumH / nH) or 0

  log(
    'I',
    logTag,
    string.format(
      'plant mass=%.3f a=%.4f b=%.4f L=a+b=%.4f h=%.4f cogToCentralAxle=%+.4f (lat, +left) cogAboveAxle=%+.4f R=%.4f Rnom=%.4f Iw=%.4f IwSim=%.4f',
      totalMass,
      a,
      b,
      L,
      coGHeight,
      cogToCentralAxle,
      cogAboveAxle,
      wheelRadius,
      wheelRadiusNominal,
      wheelInertia or -1,
      wheelInertiaSim or -1
    )
  )

  return {
    totalMass = totalMass,
    cogToFrontAxle = a,
    cogToRearAxle = b,
    cogToCentralAxle = cogToCentralAxle,
    cogAboveAxle = cogAboveAxle,
    coGHeight = coGHeight,
    distFR = L,
    trackFront = trackFront,
    trackRear = trackRear,
    wheelRadius = wheelRadius,
    wheelRadiusNominal = wheelRadiusNominal,
    wheelInertia = wheelInertia,
    wheelInertiaSim = wheelInertiaSim,
    inertia = {
      xx = getInertiaOnDiag(1, cogRel, vectorForward, vectorLeft, vectorUp),
      yy = getInertiaOnDiag(2, cogRel, vectorForward, vectorLeft, vectorUp),
      zz = getInertiaOnDiag(3, cogRel, vectorForward, vectorLeft, vectorUp),
      xy = getCrossInertia(1, 2, cogRel, vectorForward, vectorLeft, vectorUp),
      xz = getCrossInertia(1, 3, cogRel, vectorForward, vectorLeft, vectorUp),
      yz = getCrossInertia(2, 3, cogRel, vectorForward, vectorLeft, vectorUp),
    },
    cogPosRel = { cogRel.x, cogRel.y, cogRel.z },
    bbox = {
      vehLength = obj:getInitialLength(),
      vehWidth = obj:getInitialWidth(),
      vehHeight = obj:getInitialHeight(),
    },
  }
end

--[[
    Live principal axes + wet CoG in the ref-node frame.
    Do not persist world CoG in a vehicle-config YAML.
]]
local function getVehiclePrincipalAxis()
  local vectorForward, vectorLeft, vectorUp = principalAxes()
  local cogRel = wetCogRel()
  local cogWorld = obj:calcCenterOfGravity(false)
  return {
    cogPosRel = { cogRel.x, cogRel.y, cogRel.z },
    vectorForward = vectorForward:toTable(),
    vectorUp = vectorUp:toTable(),
    vectorLeft = vectorLeft:toTable(),
    debug_cog_world = cogWorld and { cogWorld.x, cogWorld.y, cogWorld.z } or nil,
  }
end

--[[
    Local function to get powertrain properties
    @return: table - Powertrain properties including device types and modes
]]
local function getPowertrainProperties()
  local devices = {}

  for _, device in ipairs(powertrain.getOrderedDevices()) do
    local data = {
      type = device.type,
      mode = device.mode,
    }

    -- Conditional assignments
    if device.parent then data.parentName = device.parent.name end
    if device.gearRatio then data.gearRatio = device.gearRatio end
    if device.gearRatios then data.gearRatios = device.gearRatios end
    if device.diffTorqueSplitA then data.diffTorqueSplit = device.diffTorqueSplitA end
    if device.availableModes then data.availableModes = device.availableModes end

    devices[device.name] = data
  end

  return devices
end

--[[
    Gets information about all vehicle controllers
    @return: table - Dictionary mapping controller names to their types
]]
local function getControllerInfos()
  local controllerTypes = {}
  for controllerName, controllerObj in pairs(controller.getAllControllers()) do
    controllerTypes[controllerName] = controllerObj.typeName
  end
  return controllerTypes
end

local function buildControllerWhitelistLookup(whitelist)
  local lookup = {}
  if type(whitelist) ~= 'table' then return lookup end

  for key, value in pairs(whitelist) do
    if type(key) == 'string' then lookup[key] = true end

    if type(value) == 'string' then
      lookup[value] = true
    elseif type(value) == 'table' then
      if type(value.name) == 'string' then lookup[value.name] = true end
      if type(value.typeName) == 'string' then lookup[value.typeName] = true end
    end
  end

  return lookup
end

--[[
    Local function to disale all safety modules. Typically, ABS, ESC,
    and additional modules from drivingDynamics controller.
    @param whitelist: table|nil - Controllers to preserve, keyed or listed by name/type
    @return: table - Removed controllers indexed by controller name
]]
local function stopSafetyFeatures(whitelist)
  -- Get all the controllers
  local allControllers = controller.getAllControllers()
  local ctrlToRemove = {}
  local whitelistLookup = buildControllerWhitelistLookup(whitelist)
  log('I', logTag, '\n-- Stopping safety features --')
  for ctrlName, ctrlObj in pairs(allControllers) do
    local ctrlType = ctrlObj.typeName
    -- Check if drivingDyanmics is a subset of the controller type
    if string.find(ctrlType, 'drivingDynamics') then
      if whitelistLookup[ctrlName] or whitelistLookup[ctrlType] then
        log('I', logTag, 'Keeping whitelisted controller: ' .. ctrlName .. ' (' .. ctrlType .. ')')
      else
      ctrlToRemove[ctrlName] = ctrlType
      log('I', logTag, 'Attempting to remove controller: ' .. ctrlName)
      if ctrlObj.shutdown then
        ctrlObj:shutdown()
        log('I', logTag, 'Controller removed: ' .. ctrlName)
      else
        if ctrlObj.update then ctrlObj.update = nil end
        if ctrlObj.updateGFX then ctrlObj.updateGFX = nil end
        if ctrlObj.isActive then ctrlObj.isActive = nil end
        log('I', logTag, 'Controller ' .. ctrlName .. ' uppdate, updateGFX set to nil')
      end
      end
    end
  end
  -- Completely unload the controllers
  for n, c in pairs(ctrlToRemove) do
    controller.unloadControllerExternal(n)
  end
  -- Turn off the rest
  setABS(false)
  setESC(false)
  return ctrlToRemove
end

----------------------------------------------------------------------------------------------------
-- Handler functions

--[[
Handler for ABS state requests
@param request: table - Request parameters containing 'enabled' field
@return: nil
]]
function M.handleSetABS(request)
  local ackResponse = 'SetABS'

  if not request then
    log('E', logTag, 'Empty request received for ' .. ackResponse)
    request:sendACK(ackResponse)
    return
  end

  if type(request.enabled) ~= 'boolean' then
    log(
      'E',
      logTag,
      'Invalid enabled type in ' .. ackResponse .. ' request: ' .. type(request.enabled)
    )
    request:sendACK(ackResponse)
    return
  end

  setABS(request.enabled)
  request:sendACK(ackResponse)
end

--[[
Handler for ESC state requests
@param request: table - Request parameters containing 'enabled' field
@return: nil
]]
function M.handleSetESC(request)
  local ackResponse = 'SetESC'

  if not request then
    log('E', logTag, 'Empty request received for ' .. ackResponse)
    request:sendACK(ackResponse)
    return
  end

  if type(request.enabled) ~= 'boolean' then
    log(
      'E',
      logTag,
      'Invalid enabled type in ' .. ackResponse .. ' request: ' .. type(request.enabled)
    )
    request:sendACK(ackResponse)
    return
  end

  setESC(request.enabled)
  request:sendACK(ackResponse)
end

--[[
Handler for 4WD mode requests
@param request: table - Request parameters containing 'mode' and/or 'rangeMode' fields
@return: nil
]]
function M.handleSet4wdMode(request)
  local ackResponse = 'Set4wdMode'

  if not request then
    log('E', logTag, 'Empty request received for ' .. ackResponse)
    request:sendACK(ackResponse)
    return
  end

  if not request.mode and not request.rangeMode then
    log('E', logTag, 'Missing both mode and rangeMode in ' .. ackResponse .. ' request')
    request:sendACK(ackResponse)
    return
  end

  -- Validate mode types if provided
  if request.mode and type(request.mode) ~= 'string' then
    log('E', logTag, 'Invalid mode type in ' .. ackResponse .. ' request: ' .. type(request.mode))
    request:sendACK(ackResponse)
    return
  end

  if request.rangeMode and type(request.rangeMode) ~= 'string' then
    log(
      'E',
      logTag,
      'Invalid rangeMode type in ' .. ackResponse .. ' request: ' .. type(request.rangeMode)
    )
    request:sendACK(ackResponse)
    return
  end

  log(
    'D',
    logTag,
    'Received 4WD mode request: ' .. tostring(request.mode) .. ' ' .. tostring(request.rangeMode)
  )
  -- Set nil to empty provided fields
  if request.mode == '' then request.mode = nil end

  if request.rangeMode == '' then request.rangeMode = nil end

  set4wdMode(request.mode, request.rangeMode)
  request:sendACK(ackResponse)
end

--[[
    Handler for differential lock requests
    @param request: table - Request parameters containing 'diff' and 'lock' fields
    @return: nil
]]
function M.handleLockDiff(request)
  local ackResponse = 'LockDiff'

  if not request then
    log('E', logTag, 'Empty request received for ' .. ackResponse)
    request:sendACK(ackResponse)
    return
  end

  if type(request.diff) ~= 'string' or (request.diff ~= 'front' and request.diff ~= 'rear') then
    log('E', logTag, 'Invalid diff type in ' .. ackResponse .. ' request: ' .. type(request.diff))
    request:sendACK(ackResponse)
    return
  end

  if type(request.lock) ~= 'boolean' then
    log('E', logTag, 'Invalid lock type in ' .. ackResponse .. ' request: ' .. type(request.lock))
    request:sendACK(ackResponse)
    return
  end

  -- Execute the differential lock operation
  lockDiff(request.diff, request.lock)
  request:sendACK(ackResponse)
end

--[[
Handler for gearbox mode requests
@param request: table - Request parameters containing 'gearIndex' field
@return: nil
]]
function M.handleSetGearboxIndex(request)
  local ackResponse = 'SetGearboxIndex'

  if not request then
    log('E', logTag, 'Empty request received for ' .. ackResponse)
    request:sendACK(ackResponse)
    return
  end

  -- Convert gearIndex to number if provided
  local gearIndex = tonumber(request.gearIndex)
  if not gearIndex then
    log(
      'E',
      logTag,
      'Invalid gearIndex type in ' .. ackResponse .. ' request: ' .. type(request.gearIndex)
    )
    request:sendACK(ackResponse)
    return
  end

  setGearboxIndex(gearIndex)
  request:sendACK(ackResponse)
end

--[[
    Handler for ABS state requests
    @param request: table - Request parameters
    @return: nil
]]
function M.handleGetABS(request)
  local data = getABS()
  request:sendResponse({
    type = 'GetABS',
    data = data,
  })
end

--[[
    Handler for ESC state requests
    @param request: table - Request parameters
    @return: nil
]]
function M.handleGetESC(request)
  local data = getESC()
  request:sendResponse({
    type = 'GetESC',
    data = data,
  })
end

--[[
    Handler for 4WD mode requests
    @param request: table - Request parameters
    @return: nil
]]
function M.handleGet4wdMode(request)
  local data = get4wdMode()
  request:sendResponse({
    type = 'Get4wdMode',
    data = data,
  })
end

--[[
    Handler for differential lock state requests
    @param request: table - Request parameters
    @return: nil
]]
function M.handleGetDiffLockState(request)
  -- Do some checks
  if not request or not request.diff then
    log('E', logTag, 'Empty request received for GetDiffLockState')
    return
  end
  if type(request.diff) ~= 'string' or (request.diff ~= 'front' and request.diff ~= 'rear') then
    log('E', logTag, 'Invalid diff type in GetDiffLockState request: ' .. type(request.diff))
    return
  end
  local data = getDiffLockState(request.diff)
  request:sendResponse({
    type = 'GetDiffLockState',
    data = data,
  })
end

--[[
    Handler for gearbox information requests
    @param request: table - Request parameters
    @return: nil
]]
function M.handleGetGearboxInfo(request)
  local data = getGearboxInfo()
  request:sendResponse({
    type = 'GetGearboxInfo',
    data = data,
  })
end

--[[
    Handler for vehicle properties requests
    @param request: table - Request parameters
    @return: nil
]]
function M.handleGetVehicleProperties(request)
  local data = getVehicleProperties(request)
  request:sendResponse({
    type = 'GetVehicleProperties',
    data = data,
  })
end

--[[
    Handler for vehicle principal axis requests
    @param request: table - Request parameters
    @return: nil
]]
function M.handleGetVehiclePrincipalAxis(request)
  local data = getVehiclePrincipalAxis()
  request:sendResponse({
    type = 'GetVehiclePrincipalAxis',
    data = data,
  })
end

function M.handleGetSettleState(request)
  request:sendResponse({
    type = 'GetSettleState',
    data = vehicleMotion(),
  })
end

function M.handleGetFrontRoadwheelSteer(request)
  request:sendResponse({
    type = 'GetFrontRoadwheelSteer',
    data = frontRoadwheelSteer(),
  })
end

--[[
    Handler for powertrain properties requests
    @param request: table - Request parameters
    @return: nil
]]
function M.handleGetPowertrainProperties(request)
  local data = getPowertrainProperties()
  request:sendResponse({
    type = 'GetPowertrainProperties',
    data = data,
  })
end

--[[
    Handler for controller information requests
    @param request: table - Request parameters
    @return: nil
]]
function M.handleGetControllerInfos(request)
  local data = getControllerInfos()
  request:sendResponse({
    type = 'GetControllerInfos',
    data = data,
  })
end

--[[
    Handler relevant engine information like maximum rpm,
    boost pressure, supercharger pressure, and so on.
]]
function M.handleEngineInfos(request)
  -- Query the engine
  local engine = powertrain.getDevice('mainEngine')
  local retData = {
    idleRPM = engine.idleRPM,
    maxRPM = engine.maxRPM,
    fuelVolume = electrics.values.fuelVolume,
    fuelCapacity = electrics.values.fuelCapacity,
    turboBoostMax = electrics.values.turboBoostMax or -1,
    superchargerBoostMax = electrics.values.superchargerBoostMax or -1,
  }
  request:sendResponse({
    type = 'EngineInfos',
    data = retData,
  })
end

--[[
    Handler for stopping safety features
    @param request: table - Request parameters, optionally containing a whitelist table
    @return: nil
]]
function M.handleStopSafetyFeatures(request)
  local data = stopSafetyFeatures(request and request.whitelist)
  request:sendResponse({
    type = 'StopSafetyFeatures',
    data = data,
  })
end

--[[
    Utility function for submitting input events 
    --> Copy from techcore but with the ability to specify filter.

    @param inputs: table - Input table containing key-value pairs
    @param key: string - Key to submit
    @param filter: string
    @return: nil
]]
local submitInput = function(inputs, key)
  local val = inputs[key]
  if val ~= nil then
    -- Need to figure out the value of the filter through FILTER_NAME
    local filter = inputs.filter or 'Direct'
    log('I', logTag, 'Submitting input: ' .. key .. '=' .. val .. ' with filter: ' .. filter)
    local m_filter = ({
      Keyboard = FILTER_KBD,
      Gamepad = FILTER_PAD,
      Direct = FILTER_DIRECT,
      KeyboardDrift = FILTER_KBD2,
      FILTER_AI = FILTER_AI,
    })[filter]
    if m_filter == nil then
      log('E', logTag, 'Invalid filter type: ' .. filter)
      return
    end
    input.event(key, val, m_filter)
  end
end

--[[
    Handler for setting vehicle control inputs
    @param request: table - Request parameters
    @return: nil
]]
function M.handleSetInputs(request)
  local ackResponse = 'Controlled'

  submitInput(request, 'throttle')
  submitInput(request, 'steering')
  submitInput(request, 'brake')
  submitInput(request, 'parkingbrake')
  submitInput(request, 'clutch')

  -- Direct electrics steering_input [-1, 1], same path LLC uses for calibration.
  local steerInput = request['steering_input']
  if steerInput ~= nil then
    local filter = request.filter or 'Direct'
    local m_filter = ({
      Keyboard = FILTER_KBD,
      Gamepad = FILTER_PAD,
      Direct = FILTER_DIRECT,
      KeyboardDrift = FILTER_KBD2,
      FILTER_AI = FILTER_AI,
    })[filter] or FILTER_DIRECT
    input.event('steering', steerInput, m_filter)
    electrics.values.steering_input = steerInput
    log('I', logTag, 'Set steering_input=' .. tostring(steerInput))
  end

  local gear = request['gear']
  if gear ~= nil then drivetrain.shiftToGear(gear) end
  request:sendACK(ackResponse)
end

function M.onExtensionLoaded() log('D', logTag, 'Loaded vehicle xlabCore') end

local function onSocketMessage(request)
  local msgType = 'handle' .. request['type']
  local handler = M[msgType]
  if handler ~= nil then
    handler(request)
  else
    log('E', logTag, 'handler does not exist: ' .. msgType)
  end
end

local function onInit()
  log('I', logTag, 'Extension loaded.')
  setExtensionUnloadMode(M, 'manual') -- this is needed for the extension to survive through level loads
end

M.onInit = onInit
M.onSocketMessage = onSocketMessage
M.stopSafetyFeatures = stopSafetyFeatures

return M
