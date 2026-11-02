-- Resolve live terrain friction in GE Lua. Contact IDs alone identify collision
-- categories, so they cannot distinguish ASPHALT from ASPHALT_PREPPED, etc.
-- Called once per vehicle graphics request; no physics-rate engine queries.
local M = {}
local abs = math.abs
local upper = string.upper
local RUBBER_ID = 4
local HEIGHT_TOLERANCE = 0.02 -- metres; conservative mesh/bridge rejection

local function resolveWheel(terrain, wheel, models)
  local invalid = {valid = false, static = 0, sliding = 0,
    id1 = wheel.id1, id2 = wheel.id2}
  if not terrain or not wheel.heightValid then return invalid end
  local surfaceID = wheel.id1 == RUBBER_ID and wheel.id2 or
    (wheel.id2 == RUBBER_ID and wheel.id1 or -1)
  if surfaceID < 0 or surfaceID == RUBBER_ID then return invalid end

  local point = vec3(wheel.pos[1], wheel.pos[2], wheel.pos[3])
  local terrainHeight = terrain:getHeight(point)
  -- Compare against the surface hit already measured by vehicle Lua. A bridge
  -- or mesh above terrain must not inherit the terrain's friction. Coplanar
  -- mesh overlays remain a limitation of this terrain-only lookup.
  if not terrainHeight or terrainHeight ~= terrainHeight or
    abs(terrainHeight - wheel.height) > HEIGHT_TOLERANCE then return invalid end
  local index = terrain:getMaterialIdxWs(point)
  if not index or index < 0 then return invalid end
  local material = terrain:getMaterial(index)
  if not material then return invalid end
  local name = material:getGroundmodelName()
  if not name or name == '' then return invalid end
  name = upper(name)

  -- Read the engine's LIVE model, not core_environment's startup Lua registry:
  -- mods can change engine values through be:setGroundModel() during a run.
  -- Deduplicate only within this request, so later mod changes remain visible.
  local model = models[name]
  if model == nil then
    local groundModel = be:getGroundModel(name)
    local data = groundModel and groundModel.data
    model = data and {
      static = tonumber(data.staticFrictionCoefficient),
      sliding = tonumber(data.slidingFrictionCoefficient),
      collisiontype = tonumber(data.collisiontype),
    } or false
    models[name] = model
  end
  if not model or model.collisiontype ~= surfaceID or not model.static or
    not model.sliding or model.static ~= model.static or
    model.sliding ~= model.sliding then return invalid end
  return {valid = true, static = model.static, sliding = model.sliding,
    id1 = wheel.id1, id2 = wheel.id2}
end

function M.query(request)
  local terrain = core_terrain.getTerrain()
  local models, results = {}, {}
  for i = 1, 4 do
    -- A failed binding/material lookup invalidates this wheel but still sends
    -- a reply, allowing the next graphics request instead of leaving it pending.
    local wheel = request.wheels[i]
    local ok, result = pcall(resolveWheel, terrain, wheel, models)
    results[i] = ok and result or {valid = false, static = 0, sliding = 0,
      id1 = wheel.id1, id2 = wheel.id2}
  end
  return {sensorId = request.sensorId, sequence = request.sequence, wheels = results}
end

return M
