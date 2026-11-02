-- Surface geometry for gtState, independent of wheel contact.
--
-- Each location uses four downward static rays: one for surface height and
-- three surrounding heights for a local plane normal. Locations are the CoG
-- and four wheel hub midpoints (not tread/contact nodes). At most 20 rays are
-- needed per geometry update, which defaults to every GT sample.
-- Positions/normals use the same world frame as gtState.pos and gtState.quat.
--
-- Geometry has a configurable cadence; intervening GT records copy the cache.
-- Wheel material IDs are copied on every GT sample from BeamNG's wheel cache.
-- No collision callbacks or additional timestamps are required. Query vectors,
-- geometry tables, and output field names are allocated once at creation.
local M = {}
-- Local aliases avoid repeated global/table lookups, especially in query().
local sqrt, max, min = math.sqrt, math.max, math.min
local format = string.format
local NAMES = {'surfaceCoG', 'surfaceFL', 'surfaceFR', 'surfaceRL', 'surfaceRR'}
local RUBBER_MATERIAL_ID = 4 -- BeamNG 0.38.3 input.lua uses this convention.
local QUERY_MARGIN = 0.5    -- metres above the query point / nearby surface.

-- friction: startup lookup indexed by collision material ID. Entries are valid
-- only when all ground models with that collision type agree on coefficients.
-- This is the fallback when terrain_friction is disabled; live mode uses GE replies.
-- fl/fr/rl/rr: existing BeamNG wheel tables; no wheel-core getter is invoked.
function M.create(config, friction, fl, fr, rl, rr, sensorId)
  local period = max(0, tonumber(config.update_time) or 0)
  local radius = max(0.01, tonumber(config.normal_radius) or 0.15)
  local range = max(1, tonumber(config.max_distance) or 100)
  -- Equilateral triangle around the query XY. Precompute its dimensions once.
  local halfWidth = 0.8660254037844386 * radius
  local halfRadius, triangleHeight = 0.5 * radius, 1.5 * radius
  local triangleWidth, maxHeightSpread = 2 * halfWidth, 2 * radius
  local timer, initialized = 0, false
  -- Optional lightweight profiling. HighPerfTimer returns elapsed milliseconds
  -- (BeamNG vehicle/ve_utils.lua). Use separate timers for call cost and wall
  -- throughput. No per-record timing channels or per-sample allocations.
  local profileEnabled = config.profile == true
  local reportInterval = max(0.1, tonumber(config.profile_interval) or 5)
  local callTimer = profileEnabled and HighPerfTimer() or nil
  local wallTimer = profileEnabled and HighPerfTimer() or nil
  local profileStarted = false
  local sampleCount, geometryCount, rayCount = 0, 0, 0
  local simulatedSeconds, totalCostMs, peakCostMs = 0, 0, 0
  local origin, down = vec3(), vec3(0, 0, -1)
  local points = {vec3(), vec3(), vec3(), vec3(), vec3()}
  local wheelList = {fl, fr, rl, rr}
  -- Optional GE lookup at graphics cadence; physics sampling only copies results.
  local terrainFrictionEnabled = config.terrain_friction == true
  local sequence, pendingSequence, pendingSeconds = 0, nil, 0
  local liveFriction = {{}, {}, {}, {}}
  local cache, keys = {}, {}
  local fields = {'queryPos', 'normal', 'height', 'heightValid', 'normalValid',
    'contactMaterialID1', 'contactMaterialID2', 'staticFriction', 'slidingFriction', 'frictionValid'}
  for i = 1, 5 do
    keys[i] = {}
    for _, field in ipairs(fields) do keys[i][field] = NAMES[i] .. '_' .. field end
    cache[i] = {queryPos = {0, 0, 0}, height = 0, heightValid = false,
      normal = {0, 0, 0}, normalValid = false}
  end

  -- castRayStatic returns distance along the unit downward direction. Convert
  -- that distance to world Z. A full-range result means no usable hit; never
  -- substitute flat ground. This query excludes vehicles/moving supports.
  local function heightAt(x, y, z)
    if profileEnabled then rayCount = rayCount + 1 end
    origin:set(x, y, z)
    local distance = obj:castRayStatic(origin, down, range)
    if not distance or distance ~= distance or distance < 0 or distance >= range then return nil end
    return z - distance
  end

  -- Refresh the height at any cached location (one ray).
  local function queryHeight(dst, point)
    dst.queryPos[1], dst.queryPos[2], dst.queryPos[3] = point.x, point.y, point.z
    dst.heightValid = false
    dst.height = 0
    -- Small upward margin tolerates a hub slightly below the surface.
    local height = heightAt(point.x, point.y, point.z + QUERY_MARGIN)
    if not height then return end
    dst.height, dst.heightValid = height, true
  end

  -- Estimate the local normal at any location. Height/normal validity are independent:
  -- the center can hit even if surrounding rays miss or cross an edge.
  local function queryNormal(dst, point)
    dst.normalValid = false
    dst.normal[1], dst.normal[2], dst.normal[3] = 0, 0, 0
    if not dst.heightValid then return end
    local height = dst.height
    -- Anchor normal rays near the found surface, independent of airborne height.
    local top = height + QUERY_MARGIN + maxHeightSpread
    -- Triangle vertices in world XY:
    --   p1 = (x,             y + radius,     z1)
    --   p2 = (x + halfWidth,  y - halfRadius, z2)
    --   p3 = (x - halfWidth,  y - halfRadius, z3)
    local z1 = heightAt(point.x, point.y + radius, top)
    local z2 = heightAt(point.x + halfWidth, point.y - halfRadius, top)
    local z3 = heightAt(point.x - halfWidth, point.y - halfRadius, top)
    if not z1 or not z2 or not z3 then return end
    -- Reject abrupt height discontinuities rather than fitting across a cliff/deck.
    -- This is a conservative heuristic; it also rejects sufficiently steep
    -- continuous surfaces. It does not prove the samples share one mesh.
    if max(z1, z2, z3, height) - min(z1, z2, z3, height) > maxHeightSpread then return end
    -- a = p2 - p3, b = p1 - p3. Their cross product points upward because
    -- a.x and b.y are positive. Normalize it to obtain the plane unit normal.
    local ax, az = triangleWidth, z2 - z3
    local bx, by, bz = halfWidth, triangleHeight, z1 - z3
    local nx, ny, nz = -az * by, az * bx - ax * bz, ax * by
    local length = sqrt(nx * nx + ny * ny + nz * nz)
    dst.normal[1], dst.normal[2], dst.normal[3] = nx / length, ny / length, nz / length
    dst.normalValid = true
  end

  -- Called once for the initial GT record. gtState deep-copies that template
  -- into its ring slots, so each slot owns its vectors (no cache aliasing).
  -- Flat field names let the existing pickle logger expand vectors normally.
  local function initializeReading(reading)
    for i = 1, 5 do
      local key = keys[i]
      reading[key.queryPos] = {0, 0, 0}
      reading[key.height] = 0
      reading[key.heightValid] = false
      reading[key.normal] = {0, 0, 0}
      reading[key.normalValid] = false
      if i > 1 then
        reading[key.contactMaterialID1] = -1
        reading[key.contactMaterialID2] = -1
        reading[key.staticFriction] = 0
        reading[key.slidingFriction] = 0
        reading[key.frictionValid] = false
      end
    end
  end

  -- Copy existing graphics-rate material data; this adds no geometry queries.
  -- A rubber/non-rubber pair identifies the surface side. Unknown or ambiguous
  -- coefficients stay zero with frictionValid=false; IDs are always preserved.
  -- In live mode, select the latest accepted GE result instead of this fallback.
  local function copyWheelMaterial(reading, key, wheel, corner)
    local id1 = wheel.contactMaterialID1 or -1
    local id2 = wheel.contactMaterialID2 or -1
    reading[key.contactMaterialID1], reading[key.contactMaterialID2] = id1, id2
    local surfaceID = -1
    if id1 == RUBBER_MATERIAL_ID then
      surfaceID = id2
    elseif id2 == RUBBER_MATERIAL_ID then
      surfaceID = id1
    end
    local coefficients
    if terrainFrictionEnabled then
      local live = liveFriction[corner]
      -- Do not copy coefficients from a response for a different contact pair.
      if live.id1 == id1 and live.id2 == id2 then coefficients = live end
    else
      coefficients = surfaceID >= 0 and surfaceID ~= RUBBER_MATERIAL_ID and friction[surfaceID] or nil
    end
    local valid = coefficients and coefficients.valid == true or false
    reading[key.frictionValid] = valid
    reading[key.staticFriction] = valid and coefficients.static or 0
    reading[key.slidingFriction] = valid and coefficients.sliding or 0
  end

  -- Called at GT cadence. cog/ref are world positions; hub arguments are node
  -- positions relative to ref, with world-oriented components. Add ref once.
  local function sample(reading, dt, cog, ref, hubFl, hubFr, hubRl, hubRr)
    if profileEnabled then
      if not profileStarted then
        wallTimer:reset() -- exclude sensor initialization / waiting for first sample
        profileStarted = true
      end
      callTimer:reset()
    end
    timer = timer + dt
    if not initialized or timer >= period then
      -- Preserve fractional elapsed time. period=0 requests every GT sample.
      timer = period > 0 and timer % period or 0
      initialized = true
      if profileEnabled then geometryCount = geometryCount + 1 end
      points[1]:set(cog)
      points[2]:setAdd2(ref, hubFl)
      points[3]:setAdd2(ref, hubFr)
      points[4]:setAdd2(ref, hubRl)
      points[5]:setAdd2(ref, hubRr)
      for i = 1, 5 do
        queryHeight(cache[i], points[i])
        queryNormal(cache[i], points[i])
      end
    end
    -- Copy values rather than assigning cache tables into ring-buffer slots:
    -- later geometry refreshes must not mutate previously recorded GT samples.
    for i = 1, 5 do
      local key, src = keys[i], cache[i]
      local pos = reading[key.queryPos]
      for j = 1, 3 do pos[j] = src.queryPos[j] end
      reading[key.height] = src.height
      reading[key.heightValid] = src.heightValid
      local normal = reading[key.normal]
      for j = 1, 3 do normal[j] = src.normal[j] end
      reading[key.normalValid] = src.normalValid
      if i > 1 then
        copyWheelMaterial(reading, key, wheelList[i - 1], i - 1)
      end
    end
    if profileEnabled then
      local costMs = callTimer:stop()
      sampleCount = sampleCount + 1
      simulatedSeconds = simulatedSeconds + dt
      totalCostMs = totalCostMs + costMs
      peakCostMs = max(peakCostMs, costMs)
      if simulatedSeconds >= reportInterval then
        local wallMs = wallTimer:stopAndReset()
        -- Budget is sampler wall time per simulated second, not whole-game CPU.
        -- sim/real is observed throughput; pauses, slow motion, other game work,
        -- and external pacing affect it, so it cannot attribute lag to this sampler.
        log('I', 'GtState.surface', format(
          'samples=%d geometry=%d rays=%d | sampler avg=%.3f ms max=%.3f ms | budget=%.2f%% | sim/real=%.3fx',
          sampleCount, geometryCount, rayCount, totalCostMs / sampleCount, peakCostMs,
          100 * totalCostMs / (1000 * simulatedSeconds),
          wallMs > 0 and 1000 * simulatedSeconds / wallMs or 0))
        sampleCount, geometryCount, rayCount = 0, 0, 0
        simulatedSeconds, totalCostMs, peakCostMs = 0, 0, 0
      end
    end
  end
  -- Queue one batch using the latest geometry's own positions and heights.
  -- These match each other even when geometry has a reduced sampling cadence.
  -- A watchdog handles lost replies; sequence numbers reject late old replies.
  -- Its elapsed time is internal only, never added to GT records.
  local function updateGFX(dt)
    if not terrainFrictionEnabled or not initialized then return end
    if pendingSequence then
      pendingSeconds = pendingSeconds + dt
      if pendingSeconds < 1 then return end
      for i = 1, 4 do liveFriction[i].valid = false end
      pendingSequence = nil
    end
    local requestWheels = {}
    for i = 1, 4 do
      local geometry, wheel = cache[i + 1], wheelList[i]
      requestWheels[i] = {pos = geometry.queryPos, height = geometry.height,
        heightValid = geometry.heightValid, id1 = wheel.contactMaterialID1 or -1,
        id2 = wheel.contactMaterialID2 or -1}
    end
    sequence = sequence + 1
    pendingSequence, pendingSeconds = sequence, 0
    obj:queueGameEngineLua(format('xlab_sensors.queryTerrainFriction(%q)',
      lpack.encode({sensorId = sensorId, sequence = sequence, wheels = requestWheels})))
  end

  local function acceptFriction(result)
    if result.sequence ~= pendingSequence then return end
    for i = 1, 4 do liveFriction[i] = result.wheels[i] end
    pendingSequence, pendingSeconds = nil, 0
  end

  return {initializeReading = initializeReading, sample = sample,
    updateGFX = updateGFX, acceptFriction = acceptFriction}
end

return M
