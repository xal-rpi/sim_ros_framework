-- Opt-in garage physics and game-side JSONL recording; Python owns plans and transfer.
-- Static node ballast [kg] and held world-vertical forces [N] use separate sessions.
-- Stop/reset/unload restore owned masses before reporting recording/cleanup errors.
-- Public calls: inspect -> validateConfig/start -> status -> stop -> readRecording.
-- See docs/support_garage.md for frames, configuration, timing and recovery.
local M = {}
local corners = {'FL', 'FR', 'RL', 'RR'}
-- Allow float32 JSON decoding noise without accepting materially wrong totals.
local WEIGHT_TOLERANCE = 1e-6
local state
local lastSession
local function finite(x)
  return type(x) == 'number' and x == x and math.abs(x) < math.huge
end

local function copy(x)
  if type(x) ~= 'table' then
    return x
  end
  local y = {}
  for key, value in pairs(x) do
    y[key] = copy(value)
  end
  return y
end

local function array(p) return {p.x, p.y, p.z} end

local function vector(t) return t and finite(t[1]) and finite(t[2]) and finite(t[3]) end

local function loadValid(ct, age)
  return ct and ct.contactValid and vector(ct.force) and vector(ct.normal) and
    ct.normal[3] > 0.999 and finite(ct.age) and ct.age >= 0 and ct.age <= age
end

local function emit(row) assert(state.file:write(jsonEncode(row), '\n')) end

local function nodeWorld(id) return obj:getPosition() + obj:getNodePosition(id) end

local function chassisFrame(ids)
  local o = nodeWorld(ids.ref)
  local f = (o - nodeWorld(ids.back)):normalized()
  local l = (nodeWorld(ids.up) - o):cross(f):normalized()
  local u = f:cross(l):normalized()
  assert(f:length() > 0.9 and l:length() > 0.9, 'degenerate chassis reference triangle')
  return o, f, l, u
end

local function localPoint(p, o, f, l, u)
  local d = p - o
  return {d:dot(f), d:dot(l), d:dot(u)}
end

local function worldPoint(p, o, f, l, u) return o + f * p[1] + l * p[2] + u * p[3] end

local function restoreMasses(s)
  local errors = {}
  for id, mass in pairs(s.baselineMass) do
    local ok, err = pcall(obj.setNodeMass, obj, id, mass)
    if not ok or err == false then
      errors[#errors + 1] = tostring(id) .. ': ' .. tostring(err)
    end
  end
  return errors
end
-- expectedOutput is the caller's session identity. Validate before any mutation.
local function checkSession(expectedOutput)
  if expectedOutput == nil then
    return
  end
  assert(type(expectedOutput) == 'string' and expectedOutput ~= '', 'Invalid expected session output')
  local output = state and state.config.output or lastSession and lastSession.output
  assert(output == expectedOutput, 'Garage session identity changed; refusing operation')
end

-- Restoration is attempted for every node even if logging or another restore fails.
-- Status remains queryable after failure; callers must not treat it as complete.
function M.stop(reason, expectedOutput)
  checkSession(expectedOutput)
  if not state then
    return
  end
  local session = state
  local restoreErrors = restoreMasses(session)
  local recordingErrors = {}
  local function recordingOperation(label, operation)
    local ok, err = pcall(operation)
    if not ok then
      recordingErrors[#recordingErrors + 1] = label .. ': ' .. tostring(err)
    end
  end
  recordingOperation('stop record', function()
    emit({kind = 'stop', time = obj:getSimTime(), reason = reason or 'manual', restoreErrors = restoreErrors})
  end)
  recordingOperation('flush', function() assert(session.file:flush()) end)
  recordingOperation('close', function() assert(session.file:close()) end)
  local failed = #restoreErrors > 0 or #recordingErrors > 0
  lastSession = {
    active = false, reason = failed and 'cleanup error' or (reason or 'manual'),
    stopReason = reason or 'manual', output = session.config.output,
    restoreErrors = restoreErrors, recordingErrors = recordingErrors,
  }
  state = nil
  assert(not failed, 'Garage cleanup failed; inspect status for restoration and recording errors')
end

-- Structural nodes are reported in a fixed reference-triangle chassis FLU frame [m].
function M.listNodes()
  local nodes = {}
  local ids = v.data.refNodes[0]
  local o, f, l, u = chassisFrame(ids)
  for _, n in pairs(v.data.nodes) do
    if n.wheelID == nil then
      nodes[#nodes + 1] = {cid = n.cid, name = n.name, chassisFLU = localPoint(nodeWorld(n.cid), o, f, l, u),
        mass = obj:getNodeMass(n.cid), group = n.group}
    end
  end
  return {referenceNodes = copy(ids), nodes = nodes}
end

local function makeSchedule(cfg)
  local s = {{name = 'reference', mode = 'reference', level = 0, duration = 8, weights = {}, from = 0, to = 0}}
  for level = 1, cfg.levels do
    for _, placement in ipairs(cfg.testPlacements) do
      local direction = placement.forceDirection or cfg.forceDirection
      local amount = (cfg.mode == 'static_ballast' and cfg.ballastKg or cfg.forceN) * level / cfg.levels
      local prefix = placement.name .. '_' .. level
      local function add(suffix, duration, weights, from, to)
        s[#s + 1] = {name = prefix .. suffix, mode = placement.name, level = level, duration = duration,
          weights = weights, from = from, to = to, forceDirection = direction}
      end
      add('_ramp', 3, placement.weights, 0, amount)
      add('_hold', cfg.holdSeconds, placement.weights, amount, amount)
      if cfg.mode == 'static_ballast' then
        add('_remove', 3, placement.weights, amount, 0)
        add('_baseline', 7, {}, 0, 0)
      else add('_release', 7, {}, 0, 0) end
    end
  end
  return s
end
-- Validate the complete command before engine queries, files or physics changes.
local function keys(values, allowed, label)
  assert(type(values) == 'table', label .. ' must be a table')
  for key in pairs(values) do
    assert(allowed[key], 'Unknown ' .. label .. '.' .. tostring(key))
  end
end

local function arrayValues(values, label)
  assert(type(values) == 'table' and #values > 0, label .. ' must be a nonempty list')
  local count = 0
  for key in pairs(values) do
    count = count + 1
    assert(finite(key) and key == math.floor(key) and key >= 1 and key <= #values,
      label .. ' must be a dense list')
  end
  assert(count == #values, label .. ' must be a dense list')
end

local function nodeId(id)
  return finite(id) and id >= 0 and id == math.floor(id)
end

local defaults = {
  mode = 'static_ballast', levels = 1, ballastKg = 10, forceN = 200,
  holdSeconds = 8, maxSpeed = 0.5, maxRate = 0.5, massReadbackGrace = 0.5,
  forceDirection = 'down', minLoadFraction = 0.1, maxLoadAge = 0.05,
}
local configKeys = {
  output = true, gtController = true, cogHeightEstimate = true, nodes = true,
  testPlacements = true, referenceNodes = true,
}
for key in pairs(defaults) do
  configKeys[key] = true
end

-- Return a private, resolved command. Units/ranges are documented in the garage guide.
function M.validateConfig(inputCfg)
  keys(inputCfg, configKeys, 'garage config')
  local cfg = copy(inputCfg)
  for key, value in pairs(defaults) do
    if cfg[key] == nil then
      cfg[key] = value
    end
  end
  assert(cfg.mode == 'static_ballast' or cfg.mode == 'baseline_release', 'Unknown garage mode')
  assert(type(cfg.output) == 'string' and cfg.output ~= '', 'Explicit output JSONL path required')
  assert(type(cfg.gtController) == 'string' and cfg.gtController ~= '', 'Explicit gtController required')
  assert(finite(cfg.cogHeightEstimate) and cfg.cogHeightEstimate > 0,
    'Supply approximate cogHeightEstimate in metres')
  assert(cfg.levels == 1 or cfg.levels == 2 or cfg.levels == 3, 'levels must be 1, 2 or 3')
  assert(finite(cfg.ballastKg) and cfg.ballastKg > 0 and cfg.ballastKg <= 200,
    'ballastKg must be in (0,200] kg total')
  assert(finite(cfg.forceN) and cfg.forceN > 0 and cfg.forceN <= 2000,
    'forceN must be in (0,2000] N total')
  assert(cfg.forceDirection == 'up' or cfg.forceDirection == 'down', 'forceDirection must be up or down')
  assert(cfg.mode == 'baseline_release' or cfg.forceDirection == 'down',
    'forceDirection applies to baseline_release only')
  assert(finite(cfg.minLoadFraction) and cfg.minLoadFraction > 0 and cfg.minLoadFraction < 1,
    'minLoadFraction must be in (0,1)')
  -- The engine decoder can round a decimal limit through float32.
  assert(finite(cfg.maxLoadAge) and cfg.maxLoadAge > 0 and cfg.maxLoadAge <= 0.050001,
    'maxLoadAge must be in (0,0.05] seconds')
  cfg.maxLoadAge = math.min(cfg.maxLoadAge, 0.05)
  assert(finite(cfg.holdSeconds) and cfg.holdSeconds >= 3, 'holdSeconds must be at least 3 s')
  assert(finite(cfg.massReadbackGrace) and cfg.massReadbackGrace >= 0.1 and cfg.massReadbackGrace <= 2,
    'massReadbackGrace must be 0.1 to 2 seconds')
  assert(finite(cfg.maxSpeed) and cfg.maxSpeed > 0 and finite(cfg.maxRate) and cfg.maxRate > 0,
    'Positive finite motion limits required')
  if cfg.referenceNodes ~= nil then
    assert(type(cfg.referenceNodes) == 'table', 'referenceNodes must be a table')
    for _, key in ipairs({'ref', 'back', 'up'}) do
      assert(nodeId(cfg.referenceNodes[key]), 'referenceNodes requires integer ref/back/up IDs')
    end
  end
  assert(type(cfg.nodes) == 'table' and next(cfg.nodes), 'Explicit structural node groups required')
  local seen = {}
  for location, entries in pairs(cfg.nodes) do
    assert(type(location) == 'string' and location ~= '', 'Nonempty location names required')
    arrayValues(entries, 'nodes.' .. location)
    local sum = 0
    for _, entry in ipairs(entries) do
      local id, weight = entry, 1 / #entries
      if type(entry) == 'table' then
        keys(entry, {cid = true, weight = true}, 'node')
        id, weight = entry.cid, entry.weight
      end
      assert(nodeId(id) and not seen[id], 'Invalid/duplicate node ID')
      assert(finite(weight) and weight > 0, 'Positive finite node weight required')
      seen[id], sum = true, sum + weight
    end
    assert(math.abs(sum - 1) <= WEIGHT_TOLERANCE, 'Node weights must sum to one per location')
  end
  arrayValues(cfg.testPlacements, 'testPlacements')
  local names = {}
  for _, placement in ipairs(cfg.testPlacements) do
    keys(placement, {name = true, weights = true, forceDirection = true}, 'test placement')
    assert(type(placement.name) == 'string' and placement.name ~= '' and not names[placement.name],
      'Unique nonempty test placement names required')
    names[placement.name] = true
    assert(type(placement.weights) == 'table' and next(placement.weights), 'Placement weights required')
    local sum = 0
    for location, weight in pairs(placement.weights) do
      assert(cfg.nodes[location] and finite(weight) and weight >= 0, 'Invalid placement weight/location')
      sum = sum + weight
    end
    assert(math.abs(sum - 1) <= WEIGHT_TOLERANCE, 'Placement weights must sum to one')
    local direction = placement.forceDirection or cfg.forceDirection
    assert(direction == 'up' or direction == 'down', 'forceDirection must be up or down')
    assert(cfg.mode == 'baseline_release' or direction == 'down', 'Upward force requires baseline_release')
  end
  return cfg
end

local function start(inputCfg)
  assert(not state, 'Garage session already active')
  local cfg = M.validateConfig(inputCfg)
  local ctrl = controller.getController(cfg.gtController)
  assert(ctrl and ctrl.getLatest, 'select an active gtState controller')
  local gt = ctrl.getLatest()
  assert(gt and gt.contact, 'GT contact data unavailable; enable contact sampling and wait for GT')
  local ids = cfg.referenceNodes or v.data.refNodes[0]
  local o, f, l, u = chassisFrame(ids)
  local byId = {}
  for _, node in pairs(v.data.nodes) do
    byId[node.cid] = node
  end
  local targets, baseline, expected = {}, {}, {}
  local locations = {}
  assert(type(cfg.nodes) == 'table', 'explicit sprung chassis node groups required')
  for location, _ in pairs(cfg.nodes) do
    locations[#locations + 1] = location
  end
  table.sort(locations)
  assert(#locations > 0, 'no ballast locations')
  for _, location in ipairs(locations) do
    local supplied = cfg.nodes[location]
    assert(type(supplied) == 'table' and #supplied > 0, 'node IDs required for ' .. location)
    targets[location] = {}
    for _, entry in ipairs(supplied) do
      local id = type(entry) == 'table' and entry.cid or entry
      local weight = type(entry) == 'table' and entry.weight or 1 / #supplied
      local n = byId[id]
      assert(n and n.wheelID == nil, 'Missing/wheel node ' .. tostring(id))
      local mass = obj:getNodeMass(id)
      assert(finite(mass) and mass > 0, 'invalid live node mass')
      baseline[id] = mass
      expected[id] = mass
      targets[location][#targets[location] + 1] = {cid = id, name = n.name, weight = weight,
        placementChassisFLU = localPoint(nodeWorld(id), o, f, l, u)}
    end
  end
  -- Use existing, always-published GT channels; debugRaw may be disabled.
  assert(vector(gt.pos) and vector(gt.dirX) and vector(gt.dirY), 'GT pose unavailable')
  assert(vector(gt.accel) and vector(gt.angVel) and vector(gt.vel), 'GT motion unavailable')
  local go = vec3(unpack(gt.pos))
  local gf = vec3(unpack(gt.dirX))
  local gl = vec3(unpack(gt.dirY))
  local gu = gf:cross(gl)
  local markerLocal, markerBody, referenceLoads = {}, {}, {}
  for _, c in ipairs(corners) do
    local ct = gt.contact[c]
    assert(ct and ct.hub and ct.point and ct.contactValid, 'reference requires four valid contacts and hub positions')
    assert(ct.normal and ct.normal[3] > 0.999, 'garage tests require flat ground')
    assert(loadValid(ct, cfg.maxLoadAge) and ct.force[3] > 0, 'reference requires fresh positive load at ' .. c)
    referenceLoads[c] = ct.force[3]
    -- Fixed body markers at axle x/y and estimated baseline CoG height above patch.
    -- No engine CoG enters this construction or the garage fit.
    local marker = vec3(ct.hub[1], ct.hub[2], ct.point[3] + cfg.cogHeightEstimate)
    markerLocal[c] = localPoint(marker, o, f, l, u)
    markerBody[c] = localPoint(marker, go, gf, gl, gu)
  end
  local old = io.open(cfg.output, 'r')
  if old then
    old:close()
    error('output exists; choose a new filename')
  end
  local schedule = makeSchedule(cfg)
  local duration = 0
  for _, phase in ipairs(schedule) do
    duration = duration + phase.duration
  end
  assert(type(enablePhysicsStepHook) == 'function', 'native extension physics hook API unavailable')
  local file = assert(io.open(cfg.output, 'w'))
  state = {file = file, ctrl = ctrl, targets = targets, baselineMass = baseline, expectedMass = expected,
    referenceNodes = copy(ids), markerLocal = markerLocal, locations = locations,
    heightDirection = {f.z, l.z, u.z}, schedule = schedule, phase = 1,
    elapsed = 0, started = obj:getSimTime(), seq = gt._seq, config = cfg, appliedBallast = {}, massChangedAt = {},
    referenceLoads = referenceLoads, appliedForceWorld = {0, 0, 0}}
  enablePhysicsStepHook() -- main.lua defaults extensionsHook to nop, even with GT active.
  emit({kind = 'metadata', schemaVersion = 4, source = 'supportGarage-v2.4', startTime = state.started,
    config = cfg, targets = targets, baselineNodeMasses = baseline, referenceNodes = copy(ids),
    markerLocal = markerLocal, markerBody = markerBody, referenceLoads = referenceLoads, schedule = schedule, duration = duration,
    motionReference = 'GT fixed nominal-CoG body point after GT capture; not live ballast-dependent CoG',
    motionFields = 'gt.accel specific force, gt.vel and gt.angVel in published body FLU; gt.pos world, quat xyzw body-to-world',
    placementFrame = 'fixed chassis FLU: reference node origin, ref-back forward, orthogonalized up',
    timing = 'GT sampled before extension hook; mass/force commands affect subsequent native integration',
    garageInputs = 'corner loads and fixed chassis marker heights; CoG height is user estimate',
    diagnostics = 'existing GT motion copied without differentiation or second lever transport; markerBody uses GT frame',
    massReadback = 'appliedBallast is commanded node mass; native getter delayed; steady checks after grace interval'})
  return {output = cfg.output, duration = duration, mode = cfg.mode, targets = copy(targets)}
end
-- Start one validated session; failures restore masses and retain cleanup status.
function M.start(cfg)
  assert(not state, 'garage session already active')
  local ok, result = pcall(start, cfg)
  if not ok then
    local cleanupOk, cleanupError = pcall(M.stop, 'start error: ' .. tostring(result))
    error(tostring(result) .. (cleanupOk and '' or '; ' .. tostring(cleanupError)))
  end
  return result
end
-- Inventory and contact-enabled GT instances; this does not actuate the vehicle.
function M.inspect()
  local result = M.listNodes()
  result.source = 'supportGarage-v2.4'
  result.gtControllers = {}
  for _, ctrl in ipairs(controller.getControllersByType('xlab/gtState')) do
    local gt = ctrl.getLatest and ctrl.getLatest()
    result.gtControllers[#result.gtControllers + 1] = {name = ctrl.name, contactEnabled = gt and gt.contact ~= nil or false}
  end
  result.status = M.status()
  return result
end
-- Read bounded byte chunks only from the matching, stopped game-side recording.
function M.readRecording(offset, limit, expectedOutput)
  checkSession(expectedOutput)
  assert(not state, 'finish or stop the session before exporting')
  assert(lastSession and lastSession.output, 'no completed garage recording')
  if offset == nil then offset = 0 end
  if limit == nil then limit = 131072 end
  assert(finite(offset) and offset >= 0 and offset == math.floor(offset), 'invalid byte offset')
  assert(finite(limit) and limit > 0 and limit <= 262144 and limit == math.floor(limit), 'invalid chunk size')
  local file = assert(io.open(lastSession.output, 'rb'))
  local ok, data = pcall(function()
    assert(file:seek('set', offset))
    local bytes, err = file:read(limit)
    assert(bytes ~= nil or err == nil, err)
    return bytes or ''
  end)
  local closed, closeError = pcall(function() assert(file:close()) end)
  assert(ok, data)
  assert(closed, closeError)
  return {data = data, nextOffset = offset + #data, eof = #data < limit, output = lastSession.output}
end
-- Snapshot active progress or the last terminal status, including cleanup errors.
function M.status(expectedOutput)
  checkSession(expectedOutput)
  if not state then
    return copy(lastSession or {active = false})
  end
  return {active = true, mode = state.config.mode, phase = state.schedule[state.phase].name,
    elapsed = state.elapsed, output = state.config.output}
end

local function step(dt)
  local now = obj:getSimTime()
  if not finite(dt) or dt <= 0 or now < state.started then
    M.stop('invalid time/reset')
    return
  end
  if controller.getController(state.config.gtController) ~= state.ctrl then
    M.stop('GT removed/replaced')
    return
  end
  local gt = state.ctrl.getLatest()
  if not gt or not gt.contact or not finite(gt.time) or now - gt.time > 0.05 then
    M.stop('GT stale')
    return
  end
  for _, c in ipairs(corners) do
    local ct = gt.contact[c]
    if not loadValid(ct, state.config.maxLoadAge) then
      M.stop('contact invalid/stale: ' .. c)
      return
    end
    if ct.force[3] < state.config.minLoadFraction * state.referenceLoads[c] then
      M.stop('corner load below support margin: ' .. c)
      return
    end
  end
  local p, q, r = obj:getRollPitchYawAngularVelocity()
  if obj:getVelocity():length() > state.config.maxSpeed or math.sqrt(p * p + q * q + r * r) > state.config.maxRate then
    M.stop('motion limit')
    return
  end
  -- Native setNodeMass readback is delayed; inspect only steady commands.
  -- Ramps change every step, so a getter cannot validate their current target.
  for id, mass in pairs(state.expectedMass) do
    local changed = state.massChangedAt[id] or state.started
    if now - changed >= state.config.massReadbackGrace then
      local actual = obj:getNodeMass(id)
      if not finite(actual) or math.abs(actual - mass) > 1e-4 then
        emit({kind = 'massMismatch', time = now, cid = id, expectedKg = mass, reportedKg = actual,
          secondsSinceCommand = now - changed, readbackToleranceKg = 1e-4})
        M.stop('mass readback mismatch after steady command')
        return
      end
    end
  end
  while state.elapsed >= state.schedule[state.phase].duration do
    state.elapsed = state.elapsed - state.schedule[state.phase].duration
    state.phase = state.phase + 1
    if state.phase > #state.schedule then
      M.stop('complete')
      return
    end
    emit({kind = 'phase', time = now, phase = state.schedule[state.phase].name})
    assert(state.file:flush())
  end
  local phase = state.schedule[state.phase]
  local amount = phase.from + (phase.to - phase.from) * state.elapsed / phase.duration
  local o, f, l, u = chassisFrame(state.referenceNodes)
  local garage = {cornerLoads = {}, cornerHeights = {}, loadValid = {}, loadAge = {}, referenceWorld = array(o),
    heightSensitivity = f.z * state.heightDirection[1] + l.z * state.heightDirection[2] + u.z * state.heightDirection[3]}
  for _, c in ipairs(corners) do
    local ct = gt.contact[c]
    garage.cornerHeights[c] = worldPoint(state.markerLocal[c], o, f, l, u).z
    garage.loadValid[c] = loadValid(ct, state.config.maxLoadAge) or false
    if ct then
      garage.loadAge[c] = ct.age end
      if garage.loadValid[c] then garage.cornerLoads[c] = ct.force[3]
    end
  end
  -- Snapshot belongs to masses applied on the previous hook, not commands below.
  if gt._seq ~= state.seq then
    local reportedMasses = {}
    for id, _ in pairs(state.baselineMass) do
      reportedMasses[tostring(id)] = obj:getNodeMass(id)
    end
    emit({kind = 'sample', time = now, dt = dt, phase = phase.name, level = phase.level,
      phaseElapsed = state.elapsed, mode = state.config.mode, garage = garage,
      appliedBallast = {massKg = state.appliedBallast.massKg or 0,
        placementChassisFLU = state.appliedBallast.placementChassisFLU},
      reportedNodeMasses = reportedMasses,
      appliedForceWorld = copy(state.appliedForceWorld),
      gt = {time = gt.time, angVelRaw = gt.angVelRaw, velRaw = gt.velRaw,
        dirX = gt.dirX, dirY = gt.dirY, quat = gt.quat,
        pos = gt.pos, vel = gt.vel, accel = gt.accel, angVel = gt.angVel}})
    state.seq = gt._seq
  end
  local ballast = {massKg = 0, placementChassisFLU = {0, 0, 0}, nodes = {}}
  local forceNodes = {}
  local forceZ = 0
  for _, location in ipairs(state.locations) do
    local total = amount * (phase.weights[location] or 0)
    for _, node in ipairs(state.targets[location]) do
      local share = total * node.weight
      if state.config.mode == 'static_ballast' then
        local mass = state.baselineMass[node.cid] + share
        if mass ~= state.expectedMass[node.cid] then
          obj:setNodeMass(node.cid, mass)
          state.massChangedAt[node.cid] = now
        end
        state.expectedMass[node.cid] = mass
        ballast.massKg = ballast.massKg + share
        for axis = 1, 3 do
          ballast.placementChassisFLU[axis] = ballast.placementChassisFLU[axis] + share * node.placementChassisFLU[axis]
        end
        ballast.nodes[#ballast.nodes + 1] = {cid = node.cid, addedKg = share, world = array(nodeWorld(node.cid))}
      elseif share ~= 0 then
        local fz = (phase.forceDirection == 'up' and 1 or - 1) * share
        obj:applyForceVector(node.cid, vec3(0, 0, fz))
        forceZ = forceZ + fz
        forceNodes[#forceNodes + 1] = {cid = node.cid, force = {0, 0, fz}, world = array(nodeWorld(node.cid))}
      end
    end
  end
  if ballast.massKg > 0 then
    for axis = 1, 3 do
      ballast.placementChassisFLU[axis] = ballast.placementChassisFLU[axis] / ballast.massKg
    end
  end
  state.appliedBallast = ballast
  state.appliedForceWorld = {0, 0, forceZ}
  -- Log exact transition commands, including release edges, plus GT-rate ramps.
  if state.commandPhase ~= state.phase or state.commandSeq ~= gt._seq then
    emit({kind = 'command', time = now, phase = phase.name, phaseElapsed = state.elapsed,
      ballast = {massKg = ballast.massKg, placementChassisFLU = ballast.placementChassisFLU},
      forceNodes = forceNodes})
    state.commandPhase = state.phase
    state.commandSeq = gt._seq
  end
  state.elapsed = state.elapsed + dt
end

function M.onPhysicsStep(dt)
  if not state then
    return
  end
  local ok, err = pcall(step, dt)
  if not ok then
    local cleanupOk, cleanupError = pcall(M.stop, 'error: ' .. tostring(err))
    error(tostring(err) .. (cleanupOk and '' or '; ' .. tostring(cleanupError)))
  end
end

function M.onReset()
  -- Native vehicle reset already restores node masses; avoid reapplying old fuel masses.
  if state then
    state.baselineMass = {}
    M.stop('vehicle reset; native masses restored')
  end
end

function M.onExtensionUnloaded()
  M.stop('extension unload')
end
return M
