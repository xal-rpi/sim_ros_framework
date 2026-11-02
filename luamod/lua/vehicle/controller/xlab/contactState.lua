-- Corner contact sample for gtState. Not a controller.
-- gtState requires this only when sensors.gtstate.contact.enable is true,
-- then publishes one sample on its physics step.
--
-- v2: force/moment are the latest completed single-physics-step aggregate.
-- Held values have explicit age/status; missing contact never means proven lift.
-- No direct wheel-core getter reads: downforce is cached graphics-rate RAW data.
-- Hub/shock motion is read each GT sample. Call physicsStep BEFORE GT rate gating,
-- and updateGFX from gtState's graphics callback (after wheels.updateGFX).

local abs, max = math.abs, math.max
local M = {}

local CORNERS = { 'FL', 'FR', 'RL', 'RR' }
local FOUR = { FL = true, FR = true, RL = true, RR = true }
local logTag = 'contactState'

local function newBin()
  return {
    n = 0, nsum = 0,
    fx = 0, fy = 0, fz = 0,
    mx = 0, my = 0, mz = 0,
    px = 0, py = 0, pz = 0,
    nx = 0, ny = 0, nz = 0,
    peakFn = -1, matA = -1, matB = -1,
    depth = 0, slip = 0,
    firstTime = nil, lastTime = nil, step = nil,
    nodeIds = {}, seenNodes = {}, duplicatePackets = 0,
  }
end

local function copyBin(src)
  return {
    n = src.n, nsum = src.nsum,
    fx = src.fx, fy = src.fy, fz = src.fz,
    mx = src.mx, my = src.my, mz = src.mz,
    px = src.px, py = src.py, pz = src.pz,
    nx = src.nx, ny = src.ny, nz = src.nz,
    peakFn = src.peakFn, matA = src.matA, matB = src.matB,
    depth = src.depth, slip = src.slip,
    firstTime = src.firstTime, lastTime = src.lastTime, step = src.step,
    nodeIds = src.nodeIds, duplicatePackets = src.duplicatePackets,
  }
end

local function zeroBin(b)
  b.n = 0
  b.nsum = 0
  b.fx = 0
  b.fy = 0
  b.fz = 0
  b.mx = 0
  b.my = 0
  b.mz = 0
  b.px = 0
  b.py = 0
  b.pz = 0
  b.nx = 0
  b.ny = 0
  b.nz = 0
  b.peakFn = -1
  b.matA = -1
  b.matB = -1
  b.depth = 0
  b.slip = 0
  b.firstTime, b.lastTime, b.step = nil, nil, nil
  b.nodeIds, b.seenNodes, b.duplicatePackets = {}, {}, 0
end

local function addContact(bin, pos, normal, normalForce, depth, slipVel, mat1, mat2)
  local px, py, pz = pos.x, pos.y, pos.z
  local nx, ny, nz = normal.x, normal.y, normal.z
  local fx = normalForce * nx
  local fy = normalForce * ny
  local fz = normalForce * nz
  bin.n = bin.n + 1
  bin.nsum = bin.nsum + normalForce
  bin.fx = bin.fx + fx
  bin.fy = bin.fy + fy
  bin.fz = bin.fz + fz
  bin.mx = bin.mx + (py * fz - pz * fy)
  bin.my = bin.my + (pz * fx - px * fz)
  bin.mz = bin.mz + (px * fy - py * fx)
  bin.px = bin.px + px * normalForce
  bin.py = bin.py + py * normalForce
  bin.pz = bin.pz + pz * normalForce
  bin.nx = bin.nx + nx * normalForce
  bin.ny = bin.ny + ny * normalForce
  bin.nz = bin.nz + nz * normalForce
  if normalForce > bin.peakFn then
    bin.peakFn = normalForce
    bin.matA = mat1
    bin.matB = mat2
  end
  if depth > bin.depth then
    bin.depth = depth
  end
  local slip = abs(slipVel or 0)
  if slip > bin.slip then
    bin.slip = slip
  end
end

local function chooseContact(fresh, previous, grounded)
  if fresh and fresh.n > 0 then
    return 'fresh', fresh
  end
  if grounded ~= false and previous and previous.n > 0 then
    return 'hold', previous
  end
  return 'clear', fresh or newBin()
end

local function weightedNormal(bin)
  if bin.n == 0 or bin.nsum == 0 then
    return nil, nil, nil
  end
  local mag = math.sqrt(bin.nx * bin.nx + bin.ny * bin.ny + bin.nz * bin.nz)
  if mag < 1e-12 then
    return nil, nil, nil
  end
  return bin.nx / mag, bin.ny / mag, bin.nz / mag
end

local function pickShock(beams, corner)
  local shock, damper
  local shockCount, damperCount = 0, 0
  for _, beam in pairs(beams) do
    local name = beam.name
    if name == 'shock_' .. corner then
      shock = beam
      shockCount = shockCount + 1
    elseif name == 'damper_' .. corner then
      damper = beam
      damperCount = damperCount + 1
    end
  end
  if shockCount > 0 then
    return shock, shockCount, 'shock_' .. corner
  end
  if damperCount > 0 then
    return damper, damperCount, 'damper_' .. corner
  end
  return nil, 0, nil
end

local function selfTest()
  local failures = {}
  local function check(cond, msg)
    if not cond then failures[#failures + 1] = msg end
  end
  local function near(a, b, msg)
    check(type(a) == 'number' and abs(a - b) < 1e-6, msg .. ' got ' .. tostring(a))
  end

  local bin = newBin()
  addContact(bin, { x = 1, y = 0, z = 0 }, { x = 0, y = 0, z = 1 }, 100, 0.01, 0.2, 4, 5)
  addContact(bin, { x = 3, y = 0, z = 0 }, { x = 0, y = 0, z = 1 }, 300, 0.02, 1.5, 7, 8)
  near(bin.nsum, 400, 'nsum')
  near(bin.fz, 400, 'fz')
  near(bin.my, -1000, 'my')
  near(bin.px / bin.nsum, 2.5, 'weighted x')
  check(bin.matA == 7 and bin.matB == 8, 'material')

  local fresh = newBin()
  addContact(fresh, { x = 0, y = 0, z = 0 }, { x = 0, y = 0, z = 1 }, 50, 0, 0, 1, 1)
  local empty = newBin()
  local mode, chosen = chooseContact(fresh, nil, true)
  check(mode == 'fresh' and chosen.fz == 50, 'fresh')
  mode, chosen = chooseContact(empty, fresh, true)
  check(mode == 'hold' and chosen.fz == 50, 'hold')
  mode, chosen = chooseContact(empty, fresh, false)
  check(mode == 'clear' and chosen.n == 0, 'airborne')
  mode, chosen = chooseContact(empty, nil, true)
  check(mode == 'clear' and chosen.n == 0, 'no packet yet')

  local beams = {
    { name = 'damper_FL', cid = 1 },
    { name = 'shock_FL', cid = 2 },
    { name = 'damper_FR', cid = 3 },
  }
  local picked, _, pickedName = pickShock(beams, 'FL')
  check(picked and picked.cid == 2 and pickedName == 'shock_FL', 'shock preferred')
  picked = pickShock(beams, 'RL')
  check(picked == nil, 'missing beam')

  if #failures == 0 then
    return true, 'pass'
  end
  return false, table.concat(failures, '; ')
end

local function write3(dst, x, y, z)
  if x == nil then
    return nil
  end
  dst = dst or { 0, 0, 0 }
  dst[1] = x
  dst[2] = y
  dst[3] = z
  return dst
end

local function assignId(map, id, key, label)
  if id == nil then
    return
  end
  local prev = map[id]
  if prev ~= nil and prev ~= key then
    log('W', logTag, string.format('%s %s mapped to %s and %s; keeping %s', label, tostring(id), prev, key, prev))
    return
  end
  map[id] = key
end

local function bindWheels(state)
  local list = wheels and wheels.wheels or {}
  for _, wd in pairs(list) do
    if type(wd) == 'table' and FOUR[wd.name] then
      if state.wheelsByCorner[wd.name] == nil then
        state.wheelsByCorner[wd.name] = wd
      else
        log('W', logTag, 'duplicate wheel name ' .. wd.name)
      end
      assignId(state.cornerByWheelId, wd.wheelID, wd.name, 'wheelID')
      assignId(state.cornerByCid, wd.cid, wd.name, 'cid')
    end
  end
end

local function bindShocks(state)
  local beams = (v and v.data and v.data.beams) or {}
  for _, corner in ipairs(CORNERS) do
    local beam, count, name = pickShock(beams, corner)
    local entry = { name = name, cid = nil, refM = nil, count = count }
    if beam then
      entry.cid = beam.cid
      if beam.cid ~= nil then
        local ok, ref = pcall(obj.getBeamRefLength, obj, beam.cid)
        if ok then
          entry.refM = ref
        end
      end
      if count > 1 then
        log('W', logTag, string.format('%s matched %d beams; using one', name, count))
      end
    end
    state.shocks[corner] = entry
  end
end

local function sampleHub(wd)
  if not wd or wd.node1 == nil or wd.node2 == nil then
    return nil, nil, nil, nil, nil, nil
  end
  local ok, x, y, z, vx, vy, vz = pcall(function()
    local p1 = obj:getNodePosition(wd.node1)
    local p2 = obj:getNodePosition(wd.node2)
    local origin = obj:getPosition()
    local v1 = obj:getNodeVelocityVector(wd.node1)
    local v2 = obj:getNodeVelocityVector(wd.node2)
    return origin.x + (p1.x + p2.x) * 0.5,
      origin.y + (p1.y + p2.y) * 0.5,
      origin.z + (p1.z + p2.z) * 0.5,
      (v1.x + v2.x) * 0.5,
      (v1.y + v2.y) * 0.5,
      (v1.z + v2.z) * 0.5
  end)
  if not ok then
    return nil, nil, nil, nil, nil, nil
  end
  return x, y, z, vx, vy, vz
end

local function sampleShock(cid)
  if cid == nil then return nil, nil, nil, nil end
  -- Independent protected calls: an unavailable force getter must not discard motion.
  local okL, length = pcall(obj.getBeamLength, obj, cid)
  local okV, vel = pcall(obj.getBeamVelocity, obj, cid)
  local okS, stress = pcall(obj.getBeamStress, obj, cid)
  local okD, stressDamp = pcall(function() return obj:getBeamStressDamp(cid) end)
  return okL and length or nil, okV and vel or nil,
    okS and stress or nil, okD and stressDamp or nil
end

-- Snapshot ONLY fields written by stock wheels.updateWheelsGFX. The native
-- getter's reset semantics are unknown; do not add a second reader.
local function cachedWheel(wd, now)
  if not wd then return nil end
  local mat1, mat2 = wd.contactMaterialID1, wd.contactMaterialID2
  local known = type(mat1) == 'number' or type(mat2) == 'number'
  local grounded = known and ((type(mat1) == 'number' and mat1 >= 0)
    or (type(mat2) == 'number' and mat2 >= 0)) or nil
  if known and not grounded then grounded = false end
  return { raw = wd.downForceRaw, time = now, grounded = grounded }
end

local function cornerTemplate(shock)
  return {
    hub = { 0, 0, 0 },
    hubVel = { 0, 0, 0 },
    shockLength = nil,
    shockVel = nil,
    shockStress = nil,
    shockRef = shock and shock.refM or nil,
    shockName = shock and shock.name or nil,
    nContacts = 0,
    normalSum = 0,
    force = { 0, 0, 0 },
    moment = { 0, 0, 0 },
    point = nil,
    normal = nil,
    matA = -1,
    matB = -1,
    depth = 0,
    slipVel = 0,
    age = nil,
    downforce = nil,
  }
end

local function publishCorner(dst, bin, age, down, hub, shock)
  local hx, hy, hz, hvx, hvy, hvz = hub[1], hub[2], hub[3], hub[4], hub[5], hub[6]
  dst.hub = write3(dst.hub, hx, hy, hz)
  dst.hubVel = write3(dst.hubVel, hvx, hvy, hvz)
  local length, vel, stress, stressDamp = sampleShock(shock and shock.cid or nil)
  dst.shockLength = length
  dst.shockVel = vel
  dst.shockStress = stress -- Preserve existing spring/elastic-stress semantics.
  dst.shockStressDamp = stressDamp -- Spring plus damping, NOT a pure damper in general.
  dst.shockCid = shock and shock.cid or nil
  dst.shockRef = shock and shock.refM or nil
  dst.shockName = shock and shock.name or nil
  dst.nContacts = bin.n
  dst.normalSum = bin.nsum
  dst.force = write3(dst.force, bin.fx, bin.fy, bin.fz)
  dst.moment = write3(dst.moment, bin.mx, bin.my, bin.mz)
  dst.depth = bin.depth
  dst.slipVel = bin.slip
  dst.downforce = down
  if bin.n > 0 and bin.nsum ~= 0 then
    dst.point = write3(dst.point, bin.px / bin.nsum, bin.py / bin.nsum, bin.pz / bin.nsum)
    local nx, ny, nz = weightedNormal(bin)
    dst.normal = write3(dst.normal, nx, ny, nz)
    dst.matA = bin.matA
    dst.matB = bin.matB
    dst.age = age
  else
    dst.point = nil
    dst.normal = nil
    dst.matA = -1
    dst.matB = -1
    dst.age = nil
  end
end

local function create(config)
  config = config or {}
  local maxAge = tonumber(config.max_contact_age) or 0.1
  assert(maxAge > 0, 'contact max_contact_age must be positive')
  local ok, detail = selfTest()
  if ok then
    log('I', logTag, 'self-test pass')
  else
    log('E', logTag, 'self-test fail: ' .. detail)
  end

  local state = {
    enabled = true,
    live = {},
    held = {},
    heldTime = {},
    wheelsByCorner = {},
    cornerByWheelId = {},
    cornerByCid = {},
    shocks = {},
    step = 0, stepDt = nil, wheelCache = {},
    intervalSteps = 0, intervalPackets = {}, intervalContactSteps = {},
  }
  for _, corner in ipairs(CORNERS) do
    state.live[corner] = newBin()
    state.intervalPackets[corner], state.intervalContactSteps[corner] = 0, 0
  end
  bindWheels(state)
  bindShocks(state)

  local api = {}

  function api.nodeCollision(p)
    if not state.enabled or type(p) ~= 'table' or type(p.normalForce) ~= 'number' or not p.pos or not p.normal then
      return
    end
    local node = v.data.nodes and v.data.nodes[p.id1] or nil
    if not node or node.wheelID == nil then
      return
    end
    local key = state.cornerByWheelId[node.wheelID] or state.cornerByCid[node.wheelID]
    local bin = key and state.live[key] or nil
    if not bin then
      return
    end
    local now = obj:getSimTime()
    bin.firstTime = bin.firstTime or now
    bin.lastTime, bin.step = now, state.step
    if bin.seenNodes[p.id1] then
      bin.duplicatePackets = bin.duplicatePackets + 1
    else
      bin.seenNodes[p.id1] = true
      bin.nodeIds[#bin.nodeIds + 1] = p.id1
    end
    addContact(bin, p.pos, p.normal, p.normalForce, p.depth or 0, p.slipVel or 0, p.materialID1 or -1, p.materialID2 or -1)
  end

  -- Called EVERY physics update, including updates skipped by the GT sampler.
  -- Callback timestamps are kept separately so scheduling can be verified in logs.
  function api.physicsStep(now, dt)
    if not state.enabled then return end
    state.step = state.step + 1
    state.stepDt = dt
    state.intervalSteps = state.intervalSteps + 1
    for _, corner in ipairs(CORNERS) do
      local live = state.live[corner]
      if live.n > 0 then
        state.held[corner] = copyBin(live)
        state.heldTime[corner] = live.lastTime or now
        state.intervalPackets[corner] = state.intervalPackets[corner] + live.n
        state.intervalContactSteps[corner] = state.intervalContactSteps[corner] + 1
        zeroBin(live)
      end
    end
  end

  function api.updateGFX(now)
    if not state.enabled then return end
    for _, corner in ipairs(CORNERS) do
      state.wheelCache[corner] = cachedWheel(state.wheelsByCorner[corner], now)
    end
  end

  function api.sample(dst, now)
    if not state.enabled or type(dst) ~= 'table' then return end
    now = now or obj:getSimTime()
    for _, corner in ipairs(CORNERS) do
      local wd = state.wheelsByCorner[corner]
      local cache = state.wheelCache[corner]
      local cacheAge = cache and max(0, now - cache.time) or nil
      local cacheValid = cacheAge ~= nil and cacheAge <= maxAge
      local grounded = cacheValid and cache.grounded
      if not cacheValid then grounded = nil end
      local previous = state.held[corner]
      local age = previous and max(0, now - state.heldTime[corner]) or nil
      local status = 'unavailable'
      local chosen = newBin()
      if previous and previous.nsum > 0 and age <= maxAge then
        -- Fresh collision evidence takes priority over an older graphics snapshot.
        if grounded == false and cache.time >= state.heldTime[corner] then
          status = 'off_ground'
        else
          chosen = previous
          status = state.intervalContactSteps[corner] > 0 and 'fresh' or 'held'
        end
      elseif previous then
        status = 'stale'
      elseif grounded == false then
        status = 'off_ground'
      end
      dst[corner] = dst[corner] or cornerTemplate(state.shocks[corner])
      local hx, hy, hz, hvx, hvy, hvz = sampleHub(wd)
      publishCorner(dst[corner], chosen, age, cache and cache.raw or nil,
        { hx, hy, hz, hvx, hvy, hvz }, state.shocks[corner])
      local out = dst[corner]
      out.contactStatus = status
      out.contactValid = chosen.n > 0 and chosen.nsum > 0
      out.lastContactAge = age -- Can remain available when the selected force is stale.
      out.contactTime = previous and previous.lastTime or nil
      out.contactFirstTime = previous and previous.firstTime or nil
      out.contactStep = previous and previous.step or nil
      out.physicsStep = state.step
      out.physicsDt = state.stepDt
      out.nodeIds = chosen.nodeIds
      out.duplicatePackets = chosen.duplicatePackets
      out.physicsStepsInterval = state.intervalSteps
      out.contactStepsInterval = state.intervalContactSteps[corner]
      out.contactPacketsInterval = state.intervalPackets[corner]
      out.downforceSource = 'wheel_graphics_raw'
      out.downforceAge = cacheAge
      out.downforceValid = cacheValid and type(cache.raw) == 'number'
      out.groundedGraphics = cache and cache.grounded or nil
      if cache and cache.grounded == false then out.groundedGraphics = false end
      out.schemaVersion = 2
      out.loggerVersion = 'contactState-v2-step-cache'
      state.intervalPackets[corner], state.intervalContactSteps[corner] = 0, 0
    end
    state.intervalSteps = 0
  end

  function api.readingTemplate()
    local contact = {}
    for _, corner in ipairs(CORNERS) do
      contact[corner] = cornerTemplate(state.shocks[corner])
    end
    return contact
  end

  function api.disable()
    state.enabled = false
  end

  local names = {}
  for _, corner in ipairs(CORNERS) do
    local shock = state.shocks[corner]
    names[#names + 1] = string.format('%s=%s', corner, tostring(shock and shock.name))
  end
  log('I', logTag, 'contact sample enabled ' .. table.concat(names, ' '))
  return api
end

M.create = create

return M
