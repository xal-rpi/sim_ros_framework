-- Offline road-geometry acquisition and stationary native-scene surveys.
-- No vehicle/physics callbacks. Capture the evaluated mesher payload while a
-- track is built, before Track Builder discards its tables. The Python probe
-- can inject this module into GE without installing/restarting a mod.
local M = {}
local captures = {}
local mission
local meshBuilderPath = 'util/trackBuilder/segmentToProceduralMesh'
local function xyz(v) return {v.x, v.y, v.z} end

local function nonempty(value)
  return type(value) == 'string' and value ~= '' and value or nil
end

-- Terrain material indices are local to a TerrainBlock. Resolve only the
-- actual ray-hit terrain, never the terrain underneath a mesh or bridge.
-- Cache per batch: these names are export metadata, not per-frame telemetry.
local function terrainSurface(terrain, point, cache)
  local index = terrain:getMaterialIdxWs(point)
  if not index or index < 0 then return nil end
  local key = terrain:getID() .. ':' .. index
  if cache[key] then return cache[key] end
  local material = terrain:getMaterial(index)
  if not material then return nil end
  local result = {kind = 'terrain', terrain_material_index = index,
    material_name = nonempty(material:getInternalName()),
    ground_model_name = nonempty(material:getGroundmodelName()),
    resolution_source = 'ray_hit_terrain_material'}
  cache[key] = result
  return result
end

-- Capture the effective runtime Material field after Track Builder has applied
-- JSON overrides AND defaults. Do not infer a model from a texture/material name.
local function meshSurface(name)
  local material = name and scenetree.findObject(name)
  local groundtype
  if material then groundtype = nonempty(material:getField('groundtype', 0)) end
  return {kind = 'managed_trackbuilder', material_name = name,
    ground_model_name = groundtype, resolution_source = 'evaluated_mesh_material'}
end

local function syncMission()
  local current = getMissionFilename()
  if current ~= mission then captures = {}; mission = current end
  return current
end

-- Inventory is observational: it does not load extensions or create a vehicle.
function M.probe()
  syncMission()
  local classes = {}
  for _, class in ipairs({'DecalRoad', 'MeshRoad', 'ProceduralMesh', 'BeamNGVehicle', 'TerrainBlock', 'TSStatic'}) do
    local objects = {}
    for _, name in ipairs(scenetree.findClassObjects(class)) do
      local obj = scenetree.findObject(name)
      if obj then
        local entry = {name = name, id = obj:getID()}
        -- Native C++ methods may be absent on a class; probe defensively.
        local ok, value = pcall(function() return obj:getEdgeCount() end)
        if ok then entry.edgeCount = value end
        objects[#objects + 1] = entry
      end
    end
    classes[class] = objects
  end
  return {mission = mission, classes = classes, engineRayAvailable = Engine.castRay ~= nil,
    decalVertexReadbackAvailable = type(Sim.getVerticesOfDecalRoad) == 'function',
    legacyDecalVertexGetterAvailable = type(Sim.getDecalRoadVertices) == 'function'}
end

-- The evaluated navigation graph connects authored road fragments and junctions.
-- It is route guidance, not pavement geometry. Export both directions explicitly
-- and keep the native one-way metadata rather than joining nearby decal ends.
function M.navigation()
  local current = syncMission()
  local path = assert(map.getGraphpath(), 'Navigation graph is unavailable')
  local positions, rawNodes = path.positions or {}, map.getMap().nodes or {}
  local keys = {}
  for key in pairs(positions) do keys[#keys + 1] = key end
  table.sort(keys)
  local indices, nodes, edges = {}, {}, {}
  for i, key in ipairs(keys) do
    indices[key] = i - 1
    nodes[i] = {key = key, xyz = xyz(positions[key]), radius = path.radius[key],
      normal = rawNodes[key] and rawNodes[key].normal and xyz(rawNodes[key].normal) or nil}
  end
  for _, a in ipairs(keys) do
    for b, edge in pairs(path.graph[a] or {}) do
      assert(indices[b], 'Navigation edge has an unknown endpoint')
      if not edge.oneWay or edge.inNode == a then
        edges[#edges + 1] = {from = indices[a], to = indices[b],
          oneWay = edge.oneWay or false, drivability = edge.drivability,
          nativeCost = edge.len, nativeInNode = edge.inNode, nativeOutNode = edge.outNode}
      end
    end
  end
  table.sort(edges, function(a,b) return a.from < b.from or (a.from == b.from and a.to < b.to) end)
  return {mission = current, nodes = nodes, edges = edges,
    convention = 'Directed native navigation edges; radius is navigation metadata, not pavement half-width'}
end

-- Overview footprint: all terrain extents and native road stations. Distant
-- background scenery is deliberately not used to set the sampling rectangle.
-- Report the definition rather than claiming every static object is included.
function M.footprint()
  local lo, hi = {math.huge, math.huge}, {-math.huge, -math.huge}
  local function extend(x, y)
    lo[1], lo[2] = math.min(lo[1], x), math.min(lo[2], y)
    hi[1], hi[2] = math.max(hi[1], x), math.max(hi[2], y)
  end
  for _, name in ipairs(scenetree.findClassObjects('TerrainBlock')) do
    local box = scenetree.findObject(name):getWorldBox()
    extend(box.minExtents.x, box.minExtents.y)
    extend(box.maxExtents.x, box.maxExtents.y)
  end
  for _, name in ipairs(scenetree.findClassObjects('DecalRoad')) do
    for _, row in ipairs(scenetree.findObject(name):getEdgesTable()) do
      for _, p in ipairs(row) do extend(p.x, p.y) end
    end
  end
  assert(lo[1] < hi[1] and lo[2] < hi[2], 'No terrain or road footprint; specify a bounded survey instead')
  return {minXY = lo, maxXY = hi, mission = syncMission(),
    source = 'Union of TerrainBlock world boxes and all DecalRoad evaluated edges; not distant background scenery'}
end

-- Surface observations are intentionally separate from road guides. A decal
-- can be paint, dirt decoration or an invisible AI path, not a collision mesh.
-- Each vertical column selects ONLY the first static hit in the given world-Z
-- band. It neither enumerates stacked levels nor certifies a drivable surface.
-- Batch from Python to bound each GE call; nothing runs on update callbacks.
function M.survey(points, zMin, zMax, relative)
  assert(zMax > zMin, 'Survey requires an explicit increasing height band')
  local hits = {}
  local down, range = vec3(0, 0, -1), zMax - zMin
  local terrains = scenetree.findClassObjects('TerrainBlock')
  -- The core terrain helper selects the first terrain. Avoid that ambiguity
  -- when a level contains several TerrainBlocks.
  local terrain = #terrains == 1 and scenetree.findObject(terrains[1]) or nil
  local objectFields, materialCache = {}, {}
  for _, p in ipairs(points) do
    -- Corridor queries use a per-point reference height. Ordinary grid surveys
    -- use absolute world-Z bounds. Both explicitly retain their height band.
    local offset = relative and p[3] or 0
    local a, b = vec3(p[1], p[2], zMax + offset), vec3(p[1], p[2], zMin + offset)
    local distance = castRayStatic(a, down, range)
    local entry = {valid = distance and distance >= 0 and distance < range or false}
    if entry.valid then entry.height = a.z - distance end
    local ok, hit = pcall(Engine.castRay, a, b, true, false)
    if ok and hit then
      entry.enginePoint = hit.pt and xyz(hit.pt) or nil
      entry.engineNormal = hit.norm and xyz(hit.norm) or nil
      if hit.obj then
        entry.engineObject = tostring(hit.obj)
        local objectOK, id = pcall(function() return hit.obj:getID() end)
        if objectOK then entry.engineObjectId = id end
        local classOK, class = pcall(function() return hit.obj:getClassName() end)
        if classOK then entry.engineObjectClass = class end
        -- These properties belong to Engine.castRay's hit. Python attaches
        -- them only if this hit also agrees with the static height query.
        entry.surfaceProperties = {kind = classOK and class == 'TerrainBlock' and 'terrain' or 'mesh',
          resolution_source = 'ray_hit_object_only'}
        if classOK and class == 'TerrainBlock' and hit.pt then
          -- Engine's hit.obj is a generic SimObject wrapper; resolve the same
          -- ID through scenetree to obtain TerrainBlock-specific bindings.
          local typedTerrain = objectOK and scenetree.findObjectById(id)
          local materialOK, properties = pcall(terrainSurface, typedTerrain, hit.pt, materialCache)
          if materialOK and properties then entry.surfaceProperties = properties end
        end
        if objectOK then
          local fields = objectFields[id]
          if not fields then
            fields = {persistentId = hit.obj:getField('persistentId', ''), name = hit.obj:getName()}
            if classOK and class == 'TSStatic' then
              fields.shapeName = hit.obj:getField('shapeName', '')
            end
            objectFields[id] = fields
          end
          entry.engineObjectPersistentId = fields.persistentId
          entry.engineObjectShape = fields.shapeName
          entry.engineObjectName = fields.name
        end
      end
    end
    if terrain then
      local height = terrain:getHeight(a)
      if height then
        entry.terrainHeight = height
        entry.terrainNormal = xyz(terrain:getNormal(a, true, true))
        entry.terrainObjectId = terrain:getID()
      end
    end
    hits[#hits + 1] = entry
  end
  return hits
end

function M.capture(path)
  local current = syncMission()
  assert(scenetree.MissionGroup, 'Load a fresh scene before capturing geometry')
  assert(type(path) == 'string' and path:sub(1, 31) == '/trackEditor/xlab_environments/'
    and not path:find('..', 1, true), 'Only managed environment assets are supported')
  assert(not captures[path], 'Asset already captured in this scene; use a fresh scene')
  local track = assert(jsonReadFile(path), 'Track JSON missing')
  assert(current:match('/levels/([^/]+)/') == track.level, 'Asset requires its declared base level')
  assert(not track.intersections or next(track.intersections) == nil,
    'Intersection mesh capture is not implemented; refusing an incomplete export')
  -- Do not layer another copy onto an existing instance loaded by the stock API.
  local basename = path:match('([^/]+)%.json$')
  for _, name in ipairs(scenetree.findClassObjects('ProceduralMesh')) do
    assert(not name:find(basename, 1, true), 'Asset mesh already exists; load a fresh scene')
  end
  extensions.load('util/trackBuilder/splineTrack')
  local tb = assert(extensions.util_trackBuilder_splineTrack)
  local mesher = require(meshBuilderPath)
  local original = mesher.compileMeshInfo
  local records = {}
  local function captureSegment(segment)
    local meshes = original(segment)
    local points = {}
    for pointIndex, p in ipairs(segment.points) do
      if p.quality[segment.quality] then
        points[#points + 1] = {
          index = pointIndex, xyz = {p.position.x, p.position.y, p.position.z + p.zOffset},
          width = p.width, bank = p.bank, pitch = p.pitch,
          lateral = xyz(p.orientation.nx), forward = xyz(p.orientation.ny),
          up = xyz(p.orientation.nz)}
      end
    end
    -- Keep only the final rebuild for each piece. LOD arrays share payloads;
    -- copying all three LODs would duplicate the physical geometry.
    local key = segment.subTrackIndex .. ':' .. segment.index
    records[key] = {piece = segment.index, subtrack = segment.subTrackIndex,
      quality = segment.quality, points = points, meshes = meshes,
      centerMesh = segment.centerMesh, segment = segment}
    return meshes
  end
  mesher.compileMeshInfo = captureSegment
  local ok, err = xpcall(function() tb.load(track, true, nil, nil, true, false) end, debug.traceback)
  mesher.compileMeshInfo = original -- restore even when generation throws
  if not ok then tb.unloadAll(); error(err) end
  local result = {path = path, mission = current, segments = {},
    declaredConnected = track.connected, scope = 'managed_trackbuilder_meshes',
    completeScene = false, meshBuilderPath = meshBuilderPath}
  for _, record in pairs(records) do
    local obj = record.segment.mesh
    assert(obj, 'Evaluated segment has no scene mesh')
    record.objectId = obj:getID()
    record.objectName = 'xlabGeometry_' .. basename .. '_' .. record.subtrack .. '_' .. record.piece
    obj:setName(record.objectName) -- rename only this asset's owned mesh
    record.segment = nil
    -- Plain arrays at the transfer boundary; keep source vertex/normal indices.
    local packed = {}
    for submeshIndex, mesh in ipairs(record.meshes) do
      local vertices, faces, normals, normalCorners = {}, {}, {}, {}
      for _, v in ipairs(mesh.verts) do vertices[#vertices + 1] = xyz(v) end
      for _, n in ipairs(mesh.normals) do normals[#normals + 1] = xyz(n) end
      assert(#mesh.faces % 3 == 0, 'Unexpected face-corner count')
      for i = 1, #mesh.faces, 3 do
        faces[#faces + 1] = {mesh.faces[i].v, mesh.faces[i+1].v, mesh.faces[i+2].v}
        normalCorners[#normalCorners + 1] = {mesh.faces[i].n, mesh.faces[i+1].n, mesh.faces[i+2].n}
      end
      local surfaceOK, properties = pcall(meshSurface, mesh.material)
      packed[#packed + 1] = {index = submeshIndex, tag = mesh.tag, material = mesh.material,
        surfaceProperties = surfaceOK and properties or nil,
        vertices = vertices, triangles = faces, normals = normals, normalCorners = normalCorners}
    end
    record.meshes = packed
    result.segments[#result.segments + 1] = record
  end
  table.sort(result.segments, function(a,b)
    return a.subtrack < b.subtrack or (a.subtrack == b.subtrack and a.piece < b.piece)
  end)
  result.evaluatedClosed = tb.getPieceInfo().trackClosed
  tb.unloadAll() -- retain scene geometry, as the official loader does
  captures[path] = result
  return {path = path, mission = current, segmentCount = #result.segments,
    evaluatedClosed = result.evaluatedClosed, declaredConnected = result.declaredConnected}
end

function M.segment(path, index)
  syncMission()
  return assert(captures[path], 'No capture for this asset/scene').segments[index]
end

-- Batched stationary collision checks. A short vertical ray is explicit about
-- the selected height band. It does not promise multi-hit/layer enumeration.
function M.rays(points, above, below)
  local hits = {}
  for _, p in ipairs(points) do
    local a, b = vec3(p[1], p[2], p[3] + above), vec3(p[1], p[2], p[3] - below)
    local distance = castRayStatic(a, vec3(0,0,-1), above + below)
    local entry = {valid = distance and distance >= 0 and distance < above + below or false}
    if entry.valid then entry.height = a.z - distance end
    local ok, hit = pcall(Engine.castRay, a, b, true, false)
    if ok and hit then
      entry.enginePoint = hit.pt and xyz(hit.pt) or nil
      entry.engineNormal = hit.norm and xyz(hit.norm) or nil
      -- Avoid serializing engine userdata; available hit fields are probed
      -- separately rather than assuming object/material identity exists.
    end
    hits[#hits + 1] = entry
  end
  return hits
end

return M
