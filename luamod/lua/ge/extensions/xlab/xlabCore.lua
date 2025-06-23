-- Extra functionalities to beamng tech
--

local logTag = 'XlabGE'
local M = {}

local GtStates = {}
local Controllers = {}

-- Basic Hello Xlab
M.handleHelloXlab = function(request)
  log('I', logTag, 'Hello Xlab')
  request:sendACK('HelloXlab')
end

-- Groundtruth full state
M.handleGetGtStateId = function(request)
  local sensorId = GtStates[request['name']]
  local resp = { type = 'getGtStateId', data = sensorId }
  request:sendResponse(resp)
end

M.handleOpenGtState = function(request)
  log('I', logTag, 'Opening GtState...')
  local args = {}
  args.GFXUpdateTime = request['GFXUpdateTime']
  args.physicsUpdateTime = request['physicsUpdateTime']
  args.numPhysicsStepsForGFXSave = request['numPhysicsStepsForGFXSave']
  args.pos = vec3(request['pos'][1], request['pos'][2], request['pos'][3])
  args.dir = vec3(request['dir'][1], request['dir'][2], request['dir'][3])
  args.left = vec3(request['left'][1], request['left'][2], request['left'][3])
  args.isVisualised = request['isVisualised']
  args.isUsingGravity = request['isUsingGravity']
  args.isSnappingDesired = request['isSnappingDesired']
  args.isForceInsideTriangle = request['isForceInsideTriangle']
  args.isDirWorldSpace = request['isDirWorldSpace']
  args.isAllowWheelNodes = request['isAllowWheelNodes']

  local name = request['name']
  local vid = scenetree.findObject(request['vid']):getID()

  GtStates[name] = extensions.xlab_sensors.createGtState(vid, args)
  log('I', logTag, 'Opened GtState sensor')

  request:sendACK('OpenedGtState')
end

M.handleCloseGtState = function(request)
  local name = request['name']
  local vid = request['vid']
  local sensorId = GtStates[name]
  if sensorId ~= nil then
    GtStates[name] = nil -- remove from ge lua
    extensions.xlab_sensors.removeGtState(vid, sensorId) -- remove from vlua.
    log('I', logTag, 'Closed Groundtruth State sensor')
  end

  request:sendACK('ClosedGtState')
end

M.handlePollGtStateGE = function(request)
  local name = request['name']
  local sensorId = GtStates[name]
  if sensorId ~= nil then
    local readings = extensions.xlab_sensors.getGtStateReadings(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollGtStateGE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end
  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = { type = 'PollGtStateGE', data = {} }
  log('I', logTag, 'WARNING: GtState sensor not found')
  request:sendResponse(resp)
end

M.handleOpenController = function(request)
  local name = request.name
  log('I', logTag, 'Opening LowLevelController...')

  -- lookup the real vehicle ID
  local rawVid = request.vid
  local obj = scenetree.findObject(rawVid)
  if not obj then
    log('E', logTag, 'Vehicle object not found: ' .. tostring(rawVid))
    request:sendACK('OpenedController', false)
    return
  end
  local vid = obj:getID()

  -- build our data table, filtering out unwanted keys
  local skipKeys = {
    name = true,
    vid = true,
    type = true,
    ack = true,
    _id = true,
    handled = true,
  }
  local data = { controllerId = name }
  for k, v in pairs(request) do
    if not skipKeys[k] then
      local vt = type(v)
      if vt == 'string' or vt == 'number' or vt == 'boolean' or vt == 'table' then data[k] = v end
    end
  end

  -- TODO: This may need some more generality in case controller names 
  if data.controllerType == 'nn' or data.controllerType == 'nn_v1' then
    assert(
      not Engine.Sandbox.Lua.isEnabled(),
      'This controller can only run when the Lua security sandbox is disabled. '
        .. "You will have to restart BeamNG with the '-disable-sandbox' argument."
    )
    local mod_libpath = 'lua/vehicle/controller/xlab/lib/libnn.so'
    local fs_libpath = 'tmp/libnn.so'
    if jit and jit.os then
      if jit.os == "Windows" then
        mod_libpath = 'lua/vehicle/controller/xlab/lib/libnn.dll'
        fs_libpath = 'tmp/libnn.dll'
      end
    end
    copyfile(mod_libpath, fs_libpath)

    be:sendToMailbox('libnnPath', FS:virtual2Native(fs_libpath))
    log('I', logTag, 'Using ' .. fs_libpath)
  end

  -- handle gtStateName → gtStateSensorId
  if data.gtStateName then
    local sid = GtStates[data.gtStateName]
    if sid then
      data.gtStateSensorId = sid
      log('I', logTag, 'Using gtState sensor: ' .. data.gtStateName .. ' (ID:' .. sid .. ')')
    else
      log('W', logTag, 'No valid gtState name, using limited vehicle state')
    end
    data.gtStateName = nil
  end

  -- send into the vehicle
  local payload = lpack.encode(data)
  local cmd = string.format('extensions.xlab_controller.create(%q)', payload)
  be:queueObjectLua(vid, cmd)

  -- track it
  Controllers[name] = {
    vid = vid,
    gtStateSensorId = data.gtStateSensorId,
  }

  log('I', logTag, string.format('Opened %s:%s', name, data.controllerType or ''))
  request:sendACK('OpenedController')
end

M.handleCloseController = function(request)
  local name = request['name']
  local vid = request['vid']

  if Controllers[name] then
    local controllerData = Controllers[name]
    Controllers[name] = nil
    be:queueObjectLua(vid, 'extensions.xlab_controller.remove("' .. name .. '")')
    log('I', logTag, 'Closed LowLevelController: ' .. name)
  end

  request:sendACK('ClosedController')
end

-- Update gtState sensor linked to controller
M.handleSetControllerGtState = function(request)
  local controllerName = request['controllerName']
  local gtStateName = request['gtStateName']

  if not Controllers[controllerName] then
    log('E', logTag, 'Controller not found: ' .. controllerName)
    request:sendBNGValueError('Controller not found: ' .. controllerName)
    return
  end

  if not GtStates[gtStateName] then
    log('E', logTag, 'GtState sensor not found: ' .. gtStateName)
    request:sendBNGValueError('GtState sensor not found: ' .. gtStateName)
    return
  end

  local vid = Controllers[controllerName].vid
  local gtStateSensorId = GtStates[gtStateName]

  -- Update controller data
  Controllers[controllerName].gtStateSensorId = gtStateSensorId
  Controllers[controllerName].gtStateName = gtStateName

  -- Tell the controller to use this gtState sensor
  be:queueObjectLua(vid, 'extensions.xlab_controller.setGtStateSensor(' .. gtStateSensorId .. ')')

  log(
    'I',
    logTag,
    'Updated controller ' .. controllerName .. ' to use gtState sensor ' .. gtStateName
  )
  request:sendACK('ControllerGtStateUpdated')
end

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
  log('D', logTag, 'Lua version: ' .. _VERSION)
  if headless_mode then
    log('I', logTag, 'headless_mode set')
  end

  setExtensionUnloadMode(M, 'manual') -- this is needed for the extension to survive through level loads
end

M.onInit = onInit
M.onSocketMessage = onSocketMessage

return M
