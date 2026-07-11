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
    Calculates diagonal inertia component for a given axis
    @param axis_id: number - 1(X), 2(Y), or 3(Z)
    @param cog: vec3 - Center of gravity position in the vehicle's LBU space
    @param vD: vec3 - Forward direction vector
    @param vL: vec3 - Left direction vector
    @param vU: vec3 - Up direction vector
    @return: number - Inertia value for the specified axis
]]
local function getInertiaOnDiag(axis_id, cog, vD, vL, vU, worldSpace)
  local inWorldSpace = worldSpace or false
  local inertia = 0
  -- Convert COG to FLU first
  local cogFLU = convertLBUtoFLU(cog, vD, vL, vU)

  -- Create axis mask in FLU space
  local axis_mask = { 1, 1, 1 }
  axis_mask[axis_id] = 0 -- Zero out target axis components

  -- Adjusted COG in FLU space with target axis zeroed
  local cogAdjusted =
    vec3(cogFLU.x * axis_mask[1], cogFLU.y * axis_mask[2], cogFLU.z * axis_mask[3])

  for _, node in pairs(v.data.nodes) do
    if node.nodeWeight then
      -- Convert node position to FLU space
      local nodePos = inWorldSpace and obj:getNodePosition(node.cid) or vec3(node.pos)
      local posFLU = convertLBUtoFLU(nodePos, vD, vL, vU)

      -- Create adjusted position in FLU space
      local posAdjusted =
        vec3(posFLU.x * axis_mask[1], posFLU.y * axis_mask[2], posFLU.z * axis_mask[3])

      -- Calculate squared distance to adjusted COG
      local delta = posAdjusted - cogAdjusted
      local deltaLength = delta:length()
      inertia = inertia + node.nodeWeight * deltaLength * deltaLength
    end
  end
  return inertia
end

--[[
    Calculates cross inertia component between two axes
    @param axis1: number - First axis (1-3)
    @param axis2: number - Second axis (1-3)
    @param cog: vec3 - Center of gravity position
    @param vD: vec3 - Forward direction vector
    @param vL: vec3 - Left direction vector
    @param vU: vec3 - Up direction vector
    @return: number - Cross inertia value
]]
local function getCrossInertia(axis1, axis2, cog, vD, vL, vU, worldSpace)
  local inWorldSpace = worldSpace or false
  local inertia = 0
  local cogFLU = convertLBUtoFLU(cog, vD, vL, vU)

  for _, node in pairs(v.data.nodes) do
    if node.nodeWeight then
      local nodePos = inWorldSpace and obj:getNodePosition(node.cid) or vec3(node.pos)
      local posFLU = convertLBUtoFLU(nodePos, vD, vL, vU)
      local delta = posFLU - cogFLU
      local components = { delta.x, delta.y, delta.z }
      inertia = inertia + node.nodeWeight * components[axis1] * components[axis2]
    end
  end
  return inertia
end

--[[  
    Calculate center of mass from node configuration (static, includes wheels)
    
    COORDINATE FRAME GOTCHA:
    - obj:getNodePosition(cid): World position relative to refNode (refNode is at origin)
    - node.pos: Local/initial configuration position in vehicle frame
    
    @param worldSpace: boolean - If true, use world positions; if false, use local config
    @return: totalMass, cogPosition - Total mass and COG position in selected frame
]]
local function relativeCenterOfMass(worldSpace)
  local totalMass = 0
  local cogPosition = vec3(0, 0, 0)
  local useWorldSpace = worldSpace or true
  
  for _, node in pairs(v.data.nodes) do
    if node.nodeWeight then
      local nodePos = useWorldSpace and obj:getNodePosition(node.cid) or vec3(node.pos)
      cogPosition = cogPosition + nodePos * node.nodeWeight
      totalMass = totalMass + node.nodeWeight
    end
  end
  
  return totalMass, cogPosition / totalMass
end

local function getVehicleProperties(props)
  -- Vehicle dimensions from bounding box
  local vehLength = obj:getInitialLength()
  local vehWidth = obj:getInitialWidth()
  local vehHeight = obj:getInitialHeight()

  -- Determine coordinate frame: worldSpace=true uses world coords, false uses local config
  local useWorldSpace = props and props.worldSpace or true

  -- Dynamic COG from BeamNG physics engine (runtime, global coords)
  local cogDynamicGlobal = obj:calcCenterOfGravity(false)
  -- local refNodeGlobalPos = obj:getPosition()
  
  -- Static COG calculated from node masses
  local totalMass, cogInSelectedFrame = relativeCenterOfMass(useWorldSpace)

  -- Get reference frame nodes
  local refNodes = v.data.refNodes[0]
  local nodeRef = v.data.nodes[refNodes.ref]
  local nodeBack = v.data.nodes[refNodes.back]
  local nodeUp = v.data.nodes[refNodes.up]

  -- Get reference node positions in selected frame
  -- World space: positions relative to refNode (refNode itself is at origin)
  -- Local space: initial configuration positions from jbeam
  local refNodePos, backNodePos, upNodePos
  if useWorldSpace then
    refNodePos = obj:getNodePosition(nodeRef.cid)  -- This is vec3(0,0,0)
    backNodePos = obj:getNodePosition(nodeBack.cid)
    upNodePos = obj:getNodePosition(nodeUp.cid)
  else
    refNodePos = vec3(nodeRef.pos)
    backNodePos = vec3(nodeBack.pos)
    upNodePos = vec3(nodeUp.pos)
  end

  -- Calculate vehicle principal axes from reference nodes
  local vectorForward = (refNodePos - backNodePos):normalized()
  local vectorUp = (upNodePos - refNodePos):normalized()
  local vectorLeft = vectorUp:cross(vectorForward):normalized()

  -- Get wheel rotator data
  local wRotators = wheels.wheelRotators
  local wIds = wheels.wheelRotatorIDs
  local wheelsData = {
    FR = wRotators[wIds.FR],
    FL = wRotators[wIds.FL],
    RR = wRotators[wIds.RR],
    RL = wRotators[wIds.RL],
  }

  -- Get wheel positions in selected coordinate frame
  local wheelPositions = { FR = nil, FL = nil, RR = nil, RL = nil }
  for wheelName, wheelData in pairs(wheelsData) do
    if useWorldSpace then
      wheelPositions[wheelName] = obj:getNodePosition(wheelData.node1)
    else
      wheelPositions[wheelName] = vec3(v.data.nodes[wheelData.node1].pos)
    end
  end

  -- Calculate wheel properties relative to COG
  local wheelInfo = {}
  for wheelName, wheelData in pairs(wheelsData) do
    local wheelPosToCog = wheelPositions[wheelName] - cogInSelectedFrame
    wheelInfo[wheelName:lower()] = {
      mass = getWheelMass(wheelData),
      pos = wheelPosToCog:toTable(),
      inertia = wheelData.inertia,
      radius = wheelData.radius,
      width = wheelData.tireWidth,
    }
  end

  -- Calculate COG to wheel vectors for axle distances
  local cogToFrontRight = wheelPositions.FR - cogInSelectedFrame
  local cogToRearLeft = wheelPositions.RL - cogInSelectedFrame
  local cogPostionGlobal = obj:getPosition() + cogInSelectedFrame -- COG in global coordinates
  local coGHeight = wheelInfo.fr.radius + math.abs(wheelInfo.fr.pos[3])
  -- Return vehicle properties in FLU (Front-Left-Up) frame
  return {
    vehLength = vehLength,
    vehWidth = vehWidth,
    vehHeight = vehHeight,
    coGHeight = coGHeight,
    cogPosDynamic = cogDynamicGlobal:toTable(),      -- Runtime COG in global coords
    cogPosDynamicRel = cogInSelectedFrame:toTable(), -- COG in selected frame
    cogPos = cogPostionGlobal:toTable(),             -- CoG position in the body frame centered on the ground
    distFR = obj:nodeLength(wheelsData.FR.node1, wheelsData.RR.node1),  -- Front-rear wheelbase
    distLR = obj:nodeLength(wheelsData.FR.node1, wheelsData.FL.node1),  -- Left-right track width
    totalMass = totalMass,
    cogToFrontAxle = cogToFrontRight:dot(vectorForward),
    cogToRearAxle = -cogToRearLeft:dot(vectorForward),
    cogToLeftWheelAxle = cogToRearLeft:dot(vectorLeft),
    cogToRightWheelAxle = -cogToFrontRight:dot(vectorLeft),
    vectorForward = vectorForward:toTable(),
    vectorUp = vectorUp:toTable(),
    vectorLeft = vectorLeft:toTable(),
    wheel_fr = wheelInfo.fr,
    wheel_fl = wheelInfo.fl,
    wheel_rr = wheelInfo.rr,
    wheel_rl = wheelInfo.rl,
    inertia = {
      xx = getInertiaOnDiag(1, cogInSelectedFrame, vectorForward, vectorLeft, vectorUp, useWorldSpace),
      yy = getInertiaOnDiag(2, cogInSelectedFrame, vectorForward, vectorLeft, vectorUp, useWorldSpace),
      zz = getInertiaOnDiag(3, cogInSelectedFrame, vectorForward, vectorLeft, vectorUp, useWorldSpace),
      xy = getCrossInertia(1, 2, cogInSelectedFrame, vectorForward, vectorLeft, vectorUp, useWorldSpace),
      xz = getCrossInertia(1, 3, cogInSelectedFrame, vectorForward, vectorLeft, vectorUp, useWorldSpace),
      yz = getCrossInertia(2, 3, cogInSelectedFrame, vectorForward, vectorLeft, vectorUp, useWorldSpace),
    },
  }
end

--[[
    Get vehicle principal axes and center of gravity in global coordinates
    Uses current world positions to compute runtime orientation and COG
    @return: table - COG positions, principal axes, and reference positions
]]
local function getVehiclePrincipalAxis()
  -- Get reference frame nodes
  local refNodes = v.data.refNodes[0]
  local nodeRef = v.data.nodes[refNodes.ref]
  local nodeBack = v.data.nodes[refNodes.back]
  local nodeUp = v.data.nodes[refNodes.up]

  -- Get world positions (relative to refNode, which is at origin in this frame)
  local refNodeWorldPos = obj:getNodePosition(nodeRef.cid)   -- vec3(0,0,0)
  local backNodeWorldPos = obj:getNodePosition(nodeBack.cid)
  local upNodeWorldPos = obj:getNodePosition(nodeUp.cid)

  -- Calculate principal axes from current world positions
  local vectorForward = (refNodeWorldPos - backNodeWorldPos):normalized()
  local vectorUp = (upNodeWorldPos - refNodeWorldPos):normalized()
  local vectorLeft = vectorUp:cross(vectorForward):normalized()

  -- Get COG in world space relative to refNode
  local _, cogWorldRelativeToRef = relativeCenterOfMass(true)
  local refNodeGlobalPos = obj:getPosition()
  
  -- Convert COG to global coordinates
  local cogGlobal = refNodeGlobalPos + (cogWorldRelativeToRef - refNodeWorldPos)

  return {
    cogPosStatic = cogGlobal:toTable(),                    -- COG in global coordinates
    cogPosRel = (cogGlobal - refNodeGlobalPos):toTable(), -- COG relative to refNode global pos
    vectorForward = vectorForward:toTable(),               -- Forward direction
    vectorUp = vectorUp:toTable(),                         -- Up direction
    vectorLeft = vectorLeft:toTable(),                     -- Left direction
    -- currPos = refNodeGlobalPos:toTable(),                  -- RefNode global position
    -- forwardVec = obj:getDirectionVector():normalized():toTable(),   -- Vehicle forward from BeamNG
    -- upVec = obj:getDirectionVectorUp():normalized():toTable(),      -- Vehicle up from BeamNG
    -- posRef = refNodeWorldPos:toTable(),                    -- RefNode world pos (0,0,0)
    -- cogRel = cogWorldRelativeToRef:toTable(),              -- COG in world frame rel to refNode
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
