-- Extra functionalities to beamng tech
--

local logTag = 'XlabGE'
local M = {}

local GtStates = {}
local Controllers = {}

if not extensions.scenario_scenariosLoader then
  extensions.load('scenario/scenariosLoader')
end

-- Basic Hello Xlab
M.handleHelloXlab = function(request)
  log('I', logTag, 'Hello Xlab')
  request:sendACK('HelloXlab')
end

-- Inspect deployed experiment assets before Python loads them through the
-- standard Track Builder API. Return exact file bytes for SHA-256 verification
-- and metadata; do not trust source-tree checksums for a deployed mod.
M.handleGetEnvironmentAssets = function(request)
  local manifest = jsonReadFile('/art/xlab_environments/manifest.json')
  if not manifest then
    request:sendBNGValueError('xlab_environments manifest missing; install xlab_environments.zip')
    return false
  end
  local contents = {}
  local assetRoot = '/trackEditor/xlab_environments/'
  for _, assetPath in ipairs(request.paths or {}) do
    if type(assetPath) ~= 'string' or
      assetPath:sub(1, #assetRoot) ~= assetRoot or
      assetPath:find('..', 1, true) then
      request:sendBNGValueError('Invalid environment asset path')
      return false
    end
    local content = readFile(assetPath)
    if not content then
      request:sendBNGValueError('Environment asset missing: ' .. assetPath)
      return false
    end
    contents[assetPath] = content
  end
  -- BeamNG 0.38.3's LoadTrackBuilderTrack handler expects this extension loaded.
  if not extensions.util_trackBuilder_splineTrack then
    extensions.load('util/trackBuilder/splineTrack')
  end
  request:sendResponse({type = 'EnvironmentAssets', manifest = manifest, contents = contents})
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
  args.isVisualised = request['isVisualised']
  args.accel_tau_s = request['accel_tau_s']
  args.gyro_tau_s = request['gyro_tau_s']
  args.vel_tau_s = request['vel_tau_s']
  args.wheel_angvel_tau_s = request['wheel_angvel_tau_s']
  args.debug_raw = request['debug_raw']
  args.torque_map = request['torque_map']
  args.contact = request['contact']
  args.surface = request['surface']

  local name = request['name']
  local vid = scenetree.findObject(request['vid']):getID()

  GtStates[name] = extensions.xlab_sensors.createGtState(vid, args)
  log('I', logTag, 'Opened GtState sensor')
  log('I', logTag, '[handle] numPhysicsStepsForGFXSave: ' .. tostring(args.numPhysicsStepsForGFXSave))

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

-- Stage immutable native bytes once per content identity. Reusing an existing
-- matching file avoids overwriting DLLs that another vehicle VM still holds.
local function stageNativeLibrary(base, manifest)
  local windows = jit and jit.os == 'Windows'
  local extension = windows and '.dll' or '.so'
  assert(manifest.target == (windows and 'windows' or 'linux'), 'Native library target mismatch')
  local library, hash = manifest.library, manifest.library_sha256
  assert(type(library) == 'string' and library:match('^[%w_-]+%.[%w]+$')
    and library:sub(-#extension) == extension, 'Invalid packaged native library name')
  assert(type(hash) == 'string' and #hash == 64 and hash:match('^%x+$'),
    'Invalid packaged native library hash')
  local contents = assert(readFile(base .. library), 'Missing packaged native library: ' .. library)
  local staged = 'tmp/xlab_' .. hash .. extension
  local cached = readFile(staged)
  if cached ~= nil then
    assert(cached == contents, 'Cached native library differs from packaged bytes: ' .. staged)
  else
    copyfile(base .. library, staged)
    assert(readFile(staged) == contents, 'Failed to stage native library: ' .. library)
  end
  local path = FS:virtual2Native(staged)
  assert(type(path) == 'string' and path ~= '', 'Native library path is unavailable: ' .. staged)
  return path
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

  local torqueMapStem = data.calibration and data.calibration.torque_map

  if torqueMapStem then
    assert(
      not Engine.Sandbox.Lua.isEnabled(),
      'Native torque map libs require Lua sandbox disabled. '
        .. "Restart BeamNG with '-disable-sandbox'."
    )
    assert(type(torqueMapStem) == 'string' and torqueMapStem:match('^[%w_-]+$'),
      'calibration.torque_map must name a packaged library')
    local base = 'lua/vehicle/controller/xlab/lib/'
    local manifest = assert(jsonReadFile(base .. torqueMapStem .. '.manifest.json'),
      'Missing packaged torque map manifest: ' .. torqueMapStem)
    local extension = jit and jit.os == 'Windows' and '.dll' or '.so'
    assert(manifest.library == torqueMapStem .. extension, 'Torque map manifest library mismatch')
    data.torqueMapPath = stageNativeLibrary(base, manifest)
  elseif data.controllerType == 'llc' then
    log('W', logTag, 'LLC opened without calibration.torque_map; torque control disabled')
  end

  -- An optional packaged export is staged before entering the vehicle Lua VM.
  -- Use a content-addressed native filename: Windows may keep an old DLL loaded.
  if data.environment then
    local env = data.environment
    assert(type(env) == 'table', 'environment must be a table')
    local export = env.vehicle_export
    assert(type(export) == 'string' and export:match('^[%w_-]+$'),
      'environment.vehicle_export must name a packaged vehicle export')
    assert(not Engine.Sandbox.Lua.isEnabled(),
      "Native vehicle exports require '-disable-sandbox'; restart BeamNG")
    local base = 'lua/vehicle/controller/xlab/lib/vehicles/' .. export .. '/'
    local manifest = assert(jsonReadFile(base .. 'manifest.json'),
      'Missing packaged vehicle export: ' .. export)
    local windows = jit and jit.os == 'Windows'
    local platform = windows and 'windows' or 'linux'
    local library = windows and 'roll_filter.dll' or 'libroll_filter.so'
    assert(manifest.target == platform and manifest.precision == 'float32'
      and manifest.environment_abi_version == 1 and manifest.library == library,
      'Incompatible packaged vehicle export')
    env.vehicle_export = nil
    env.library_path = stageNativeLibrary(base, manifest)
    env.ffi_header_path = base .. 'roll_filter_ffi.h'
    log('I', logTag, 'Staged vehicle environment export: ' .. export)
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


M.handleGetCurrentLevel = function(request)
  -- Get the currently loaded level name
  local levelIdentifier = getCurrentLevelIdentifier()
  
  if not levelIdentifier or levelIdentifier == '' then
    log('W', logTag, 'No level currently loaded')
    request:sendResponse({
      type = 'GetCurrentLevel',
      level = nil,
      loaded = false
    })
    return
  end
  
  log('D', logTag, 'Current level: ' .. levelIdentifier)
  request:sendResponse({
    type = 'GetCurrentLevel',
    level = levelIdentifier,
    loaded = true
  })
end

M.handleStartLevel = function(request)
  local levelName = request['levelName']
  if not levelName then
    log('E', logTag, 'No level name provided in StartLevel request')
    return
  end

  local levelInfo = extensions.core_levels.getLevelByName(levelName)
  if not levelInfo then
    log('E', logTag, 'Level not found: ' .. levelName)
    return
  end

  extensions.core_levels.startLevel(levelInfo.fullfilename)
  log('I', logTag, 'Started level: ' .. levelName)
  request:sendACK('StartedLevel')
end

M.handleGetAdvancedLevelInfo = function(request)

  local levelName = request['levelName']
  if not levelName then
    log('E', logTag, 'No level name provided in GetAdvancedLevelInfo request')
    return false
  end

  -- Get level info
  local levelInfo = extensions.core_levels.getLevelByName(levelName)
  if not levelInfo then
    log('E', logTag, 'Level not found: ' .. levelName)
    return false
  end

  -- Get scenarios for the level
  local allScenarios = extensions.scenario_scenariosLoader.getList()
  local levelScenarios = {}
  for _, scenario in ipairs(allScenarios) do
    if scenario.levelName == levelName then
      table.insert(levelScenarios, {
        name = scenario.name,
        humanName = scenario.humanName or scenario.name,
        description = scenario.description,
        sourceFile = scenario.sourceFile  -- Path to load the scenario
      })
    end
  end

  -- Enrich spawn points with pos/rot (from levels.lua onGetRawPoiListForLevel)
  local enrichedSpawnPoints = {}
  for _, spawnPoint in ipairs(levelInfo.spawnPoints or {}) do
    if spawnPoint.objectname then
      local obj = scenetree.findObject(spawnPoint.objectname)
      if obj then
        table.insert(enrichedSpawnPoints, {
          objectname = spawnPoint.objectname,
          translationId = spawnPoint.translationId,
          previews = spawnPoint.previews,
          pos = obj:getPosition(),  -- vec3 {x, y, z}
          rot = obj:getRotation()   -- quat {x, y, z, w}
        })
      else
        log('W', 'handleGetAdvancedLevelInfo', 'Spawn point object not found: ' .. spawnPoint.objectname)
      end
    end
  end

  -- Prepare response
  local resp = {
    type = 'GetAdvancedLevelInfo',
    levelName = levelName,
    levelInfo = levelInfo,  -- Full level metadata
    scenarios = levelScenarios,  -- List of scenarios with names and paths
    spawnPoints = enrichedSpawnPoints,  -- Spawn points with pos/rot
  }
  request:sendResponse(resp)
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
