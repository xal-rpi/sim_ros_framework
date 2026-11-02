# Road geometry export: source inspection and implementation plan

Status: source investigation and proposed design, 2026-10-16. A first managed-track
capture/selection/viewer prototype has since been implemented and tested; see
[working commands, actual schema and results](../tools/road_geometry/README.md).
Native-scene road guides and bounded collision point surveys have also been
tested on Derby; their distinct `0.2-native-survey` schema does not claim an
exact mesh, selected route, or usable pavement corridor.
The broader map/route capabilities below remain the target design. No game/mod
files or active scenes were changed during the initial investigation. See
[simulator operations](simulator_operations.md) for the autonomous test workflow.

## Findings from the installed simulator

Inspected BeamNG.tech 0.38.3.0's actual `lua/` tree, the installed BeamNGpy API,
our environment loader, and the downstream `drive_trajopt`/`xal_vehicle_models`
surface code. Paths below are relative to the simulator installation; line
numbers describe this inspected version, not a stable upstream API contract.

| Source | What it supplies | Limitation for this task |
|---|---|---|
| `vehicle/controller/tech/roadsSensor.lua:320–377` | Local navgraph road selection from vehicle position/direction; subsequent spline/edge estimates | Waits 5000 physics steps for the map; refreshes nearest-road selection only above 2 m/s, otherwise uses previous keys. Not a physical mesh survey. |
| `ge/extensions/tech/sensors.lua:getRoadGraph` | GE navgraph connectivity, XYZ node positions, node radii and normals | Does not need driving. Radius is navigation metadata, not measured pavement half-width. |
| `ge/extensions/tech/techCore.lua:1083–1120`, `GetRoadNetwork` | DecalRoad fields and evaluated ordered left/middle/right edge stations via `getEdgesTable()` | DecalRoads only, not Track Builder procedural meshes or all drivable space. Authored road strips can overlap. `drivableOnly` filters `drivability == -1`, not a geometric safety test. |
| Runtime native bindings, tested on Derby | `Sim.getVerticesOfDecalRoad(id)` gives flattened left/middle/right stations matching `getEdgesTable()` on a tested road | The differently named `Sim.getDecalRoadVertices()` used by the tech handler is absent in this build. The working getter supplies no triangle connectivity. |
| `ge/extensions/core/terrain.lua` and native TerrainBlock bindings | Direct height and normal observations; runtime collision hit object/class/persistent ID/shape also verified on Derby | Terrain excludes overlying static meshes. First collision hit includes buildings/obstacles and does not enumerate all layers. No verified exact static mesh readback. |
| `ge/extensions/editor/meshEditor.lua`, shared road/mesh editor APIs | MeshRoad control nodes, widths, depths and transforms; road edge APIs appear in editor helpers | Verify evaluated MeshRoad edge readback on a runtime instance before promising coverage. Control nodes alone are not the evaluated deck. |
| `ge/extensions/util/trackBuilder/splineTrack.lua:getSegmentInfo` | Evaluated segment points, width, bank, height offset, frame and source piece/subtrack indices | Only while builder state exists. Our stock loader clears it after each asset load. |
| `ge/extensions/util/trackBuilder/segmentToProceduralMesh.lua:16–45,76–85` | Actual mesher input: vertices, face corners, normals, material and submesh tags | Mesh payload is transient and includes undersides, caps, borders, walls and ceilings as applicable. Not every triangle is driving surface. |
| `ge/extensions/util/trackBuilder/basicCenters.lua:286–369` | World-space vertices generated with the evaluated cross-section/frame/height; triangle corner indices | Face vertex indices are zero-based even though Lua vertex tables are one-based. Shading normal indices are separate from vertex indices. |
| `ge/extensions/editor/rayCastTest.lua:72–77` | `Engine.castRay` examples return `hit.pt` and `hit.norm`; `castRayStatic` gives distance | Runtime verification needed for hit object identity, material fields, filters and multiple-hit behavior. A first hit alone cannot identify an arbitrary stacked road. |
| `ge/extensions/tech/openDriveExporter.lua:474–493` | A fitted road network derived from the navgraph | Not an exact physical mesh exporter. It fits cubics and writes an empty lateral profile; unsuitable as the ground truth for cambered tracks. |

The Road Sensor also synthesizes lane configuration when absent
(`getEdgeLaneConfig`). Its spline and edge estimates must not be promoted to
measured pavement boundaries. Its driving dependency is an implementation
choice of this local sensor, not a fundamental requirement of map extraction.

### Important Track Builder lifecycle

`techCore.handleLoadTrackBuilderTrack` calls `tb.load`, renames meshes, then
`tb.unloadAll`. In this version, `unloadAll` clears `subTracks`, `subPieces` and
intersections while leaving the created scene meshes in place. The mesher also
sets `segment.meshes=nil` after `createMesh`.

Consequently calling `getSegmentInfo` after the existing loader completes is
too late. Enumerating `ProceduralMesh` objects later does not recover the
builder's source topology. A general engine mesh getter has not been verified
from the inspected Lua code. That remains a runtime capability probe, not an
assumption about what the native engine supports.

For our three managed assets, prefer capturing the exact payload supplied to
`createMesh`, together with evaluated point/frame/source data, during loading.
An xlab-owned loader can scope a capture around the existing mesher's public
compile function, restoring the original function even on failure. It should
capture the final realized revision/quality for each segment, not concatenate
every rebuild or duplicate LOD. Export must not reimplement JSON curves or mesh
cross-sections in Python. A second compile from evaluated segments is another
possible probe, but compile mutates submesh bookkeeping; it is not a read-only
getter and cannot be called blindly after materialization.

Intersections use `multiTrackMerger.lua` and call `createMesh` independently of
the segment compiler. They need their own capture path; until supported, report
them as missing geometry. The current three imported tracks have empty
intersection definitions. Generic scene procedural meshes and imported static
assets remain a separate capability, not automatically covered by this hook.

Track ownership must be `(asset hash/path, subtrack, piece, submesh)`, with runtime
scene IDs as extra provenance. Do not identify ownership by scanning/renaming
every object containing `procMesh`: the stock handler's global scan is not a
reliable ownership mechanism for multiple tracks. Preserve each asset/instance
independently even when it shares XY, material or endpoints with another.

## Existing recordings for validation

Read-only inspection of the usual retained pickle/metadata files establishes:

| Run | Metadata-selected asset | GT samples | Duration | Surface height below CoG |
|---|---|---:|---:|---:|
| `utv_canam_x3_r322_easylift/run_032` | `flat_circuit.json` | 16,800 | 83.995 s | 4.8000–4.9157 m |
| `utv_canam_x3_r322_easylift/run_033` | `rolling_hills_circuit.json` | 15,420 | 77.095 s | 1.8000–5.8465 m |

Metadata hashes are respectively
`76cda25b2bc0190f57011f26591e09b05d9ee114d6dfc78b0c263ee41b2f6cbb` and
`de0038f76d33da9ef0cca4139f2982ea27c0993b5a5522e65f1b7f9220cfe968`.
All five locations have valid height/normal flags in both runs, and normal
length error is below 6e-15. Valid flags do not establish correct surface
selection or agreement with a model at an edge.

These heights corroborate the saved design elevation plus the default regular
cross-section, including its recessed middle. Vehicle CoG XYZ is not pavement
XYZ. Compare exports to `surface*_queryPos` XY and `surface*_height`/normal, not
to `pos_z` directly. Driving logs only sample the driven footprint; they cannot
prove coverage of an entire track or provide its physical boundaries.

## Recommended design

Extract once from a loaded scene, then do selection, validation and fitting
offline in Python. Keep geometry acquisition independent of a vehicle. A driven
trace, parked vehicle pose or clicked `(x,y,z,heading)` may seed route selection,
but must not define the whole physical map or be required to scan it.

Use three acquisition paths with a common small array schema:

1. **Managed Track Builder assets:** evaluated mesh payload plus ordered generator
   references. Best first implementation for these tracks. Export enabled before
   scene loading so transient data is retained. No default physics-rate work.
2. **Native roads:** enumerate DecalRoads, probe MeshRoad evaluated geometry, retain
   graph connections as navigation information. Preserve individual strips and
   any duplicate/overlapping candidates; do not silently fuse them into a road.
3. **Areas without accessible geometry:** explicitly bounded ray survey seeded
   by a selected surface/height band. It is sampled evidence, not exact mesh or
   guaranteed pavement. Missing coverage and uncertain edges remain unknown.

A first-hit global downward heightmap would choose bridge decks over lower
roads. Prefer source-owned triangles; otherwise require a Z-aware selection,
short local rays and a reported ambiguity/miss. Repeated hits are only an option
after verifying native behavior; do not promise a complete collision mesh from
rays. Overhangs and different road levels must remain separate, not collapse to
one `Z(X,Y)` sheet.

### Frame and measurement conventions

- Output uses our existing local ENU convention, identical to `gtState.pos` and
  `surface*_queryPos`/normals: project east X, project north Y, up Z, right-handed,
  metres. This is a project-local map frame; geographic datum/origin is unknown.
- Record `T_enu_from_native` explicitly; identity is the initial mapping matching
  current GT logging. Do not apply a vehicle FLU conversion to world geometry.
- An alignment operation is `p_out = R p_enu + t`, with a proper rotation
  (`det R=+1`); normals use `R n` only. Preserve the original ENU export.
- Route forward follows explicit travel order. Positive lateral is left. For a
  chosen tangent `t` and reference normal `n`, left is `normalize(n cross t)`.
- Preserve raw mesh winding/shading normals. For selected driving patches,
  declare how normals are oriented; ambiguous/inverted/vertical patches are
  flagged, not silently made drivable by flipping normals toward world up.
- Separate geometric face normals, interpolated shading normals and ray-fit
  normals. They are different measurements with different approximation errors.

### Proposed files and arrays (draft schema v0.1)

`metadata.json`, optional `geometry.npz`, and `routes/<name>.npz` suffice. JSON
contains string tables and provenance; NPZ uses numeric arrays only, read with
`allow_pickle=False`. No exporter class hierarchy or Lua surface engine.

Metadata records frame/units, schema, level/scenario/asset identities and hashes,
simulator version and inspected Lua source hashes, export settings/quality,
array descriptions, source kinds, component/boundary/route tables, uncertainties,
missing capabilities and validation results. Unknown values are null in JSON
or represented by a numeric validity mask, never invented values. Scene IDs
alone are not stable source identities across reloads.

Geometry arrays, when available:

| Array | Shape | Meaning |
|---|---|---|
| `vertices_xyz` | `(V,3)` float64 | Physical mesh vertices, world ENU metres |
| `triangles` | `(F,3)` int64 | Zero-based vertex connectivity |
| `triangle_source_id` | `(F,)` int64 | JSON source table reference |
| `triangle_patch_id` | `(F,)` int64 | Candidate surface patch, independently of route |
| `triangle_role`, `triangle_role_valid` | `(F,)` integers/bool | Known top/bottom/border/wall/cap or unknown; classification method recorded |
| `boundary_xyz`, `boundary_offsets` | `(B,3)`, `(K+1,)` | Ordered polylines; closure, source/patch and physical/usable/unknown role in JSON |

Retain original submesh/material IDs. Shared physical vertices may be duplicated
in the renderer input; any adjacency established by tolerance is recorded as a
derived association. Do not silently weld pieces, erase seams or bridge gaps.
Object groups are not automatically connected components. Exact shared-index
components, declared piece connections and tolerance-based matches are distinct.
Mesh boundary edges are not automatically pavement edges: caps, undersides,
walls and strips may hide or create topological boundaries. Holes/exclusions are
only labeled when source geometry/selection establishes them.

Route arrays (design proposal; the implemented common contract is documented
in [the exporter README](road_geometry.md#common-route-schema-05-road-routes)):

| Array | Shape | Meaning |
|---|---|---|
| `station_id` | `(N,)` int64 | Stable identity within the immutable source export; travel order is row order |
| `center_xyz` | `(N,3)` float64 | Selected route points on a specified driving patch |
| `source_ref` | `(N,4)` int64 | Asset-instance/source-table, subtrack, piece, evaluated point; -1 plus validity when unknown |
| `center_triangle_id`, `center_barycentric` | `(N,)`, `(N,3)` | Optional trace back to mesh projection, with validity |
| `section_offsets` | `(N+1,)` int64 | Ragged cross-section sample ranges |
| `section_xyz` | `(Q,3)` float64 | Ordered right-to-left surface samples |
| `section_lateral_xy_m` | `(Q,)` float64 | Horizontal projection on declared horizontal unit left axis, not cross-section arc length |
| `section_normal`, `section_normal_valid` | `(Q,3)`, `(Q,)` | Normal and its availability/type |
| `section_triangle_id` | `(Q,)` int64 | Triangle provenance, if available |
| `physical_limits_xy_m`, `physical_limits_valid` | `(N,2)` | Right/left physical pavement limits where known |
| `optimization_limits_xy_m`, `optimization_limits_valid` | `(N,2)` | Separately chosen right/left optimization limits |

Route JSON declares direction, open/closed/unknown topology, selected patch
sequence, section construction, boundary associations and connectivity. Keep
station identity separate from distance. Optional cumulative chord distances
are named `center_distance_xy_m`/`center_distance_xyz_m` or
`reference_distance_xy_m`/`reference_distance_xyz_m`; none is downstream fitted
surface progress, and endpoint gaps are not counted as fabricated connecting
segments. Closure requires explicit source intent plus measured continuity;
`connected=true` or nearby endpoints alone does not certify it.

Keep the generator design reference (which can be above the recessed road)
separate from the surface-projected `center_xyz`. Construct initial sections by
intersecting selected patches with planes perpendicular to a declared evaluated
route tangent, through the route reference. Choose the connected surface branch
associated with the selected route; preserve other branches as ambiguities or
separate sections. Never span a missing interval with an invented straight
surface. Actual surface samples capture camber/profile and are not limited to
left/center/right points.

Connectivity is a small JSON list of explicit oriented section/station endpoint
connections, with source, geometric residual and validity. Navgraph edges remain
navigation connections until their physical crossing/level is verified.
Material names may be exported. Grip, safety margins, speed restrictions and
obstacle semantics remain unknown unless their source is actually available;
navigation drivability and inferred lane counts are not substitutes.

### Reproducible route recipes

Keep the export immutable. A JSON recipe references its hash and an ordered
operation list: select route/component/source/spatial ROI (including Z), slice
explicit station IDs, split, concatenate explicitly compatible endpoints,
reverse, choose a seam only on a declared closed route, rigid transform, or
resample with declared spacing and interpolation. New samples retain parent
station interval/weight and source-triangle references; do not relabel them as
original evaluated stations.

Reversal reverses stations and each section's right-to-left ordering, negates
lateral coordinates, swaps/negates right and left limits, reverses connections,
and keeps surface normal orientation. Concatenation checks frame, orientation,
surface ownership and position/tangent residuals; it rejects incompatible joins.
Resampling can use a documented polyline parameter initially, then reproject on
the selected original patch. No silent smoothing, snapping, closure or gap fill.

## Staged implementation and tests

1. **Capability probe:** inventory scene road/mesh objects and API availability,
   confirm ray fields and filters, capture one segment in a fresh paused test
   scene, and verify indexed geometry against actual collision ray hits. Do not
   engage a vehicle. Confirm native quality/coordinate/frame conventions before
   freezing the schema. Classify failures as unsupported or incomplete.
2. **Track Builder export:** implement an opt-in GE capture in xlab, source-owned
   per-asset cache/chunked transfer, JSON/NPZ writer and raw inspection. Clear cache
   with scene lifecycle; fail rather than return geometry from another scene.
   No base-game edits, new maps, GT message changes or per-vehicle sampling needed.
3. **Routes/corridors:** expose candidate source paths without assuming a global
   centerline. Add mesh-backed center projection/sections, explicit route/ROI/Z
   selection, limit roles and recipes. Unknown or ambiguous data stays flagged.
4. **Validation and inspection:** finite/index/degenerate/duplicate checks,
   station spacing and discontinuities, normal/winding checks, self-intersections,
   section ordering/holes/branching, limits, endpoint and tangent closure residuals.
   A standalone interactive 3D HTML view shows components, source IDs, routes,
   boundaries, station/section selection and highlighted defects; elevation,
   signed bank, width and spacing plots share selected stations. Save JSON
   diagnostics and export/recipe hashes. Report errors, not just a pass/fail flag.
5. **Coverage tests and example exports:** Flat Circuit + run_032, Rolling Hills
   + run_033, and the banked donut. Compare exported triangle heights/normals at
   logged GT query positions; account for ray normal radius, triangulation, edge
   neighborhoods and source selection. Use independent random collision probes
   as well, so a driven path is not the only coverage test. Add a stock road with
   junction/marking overlaps and an existing overpass to test that co-located XY
   layers and disconnected components are not fused. Check multiple managed
   assets retain distinct ownership without changing their saved placement.
6. **Downstream fit check:** Python selects a section and uses the existing
   plan-view-height or Darboux/Darboux-height surface engine. Validate fitted
   positions, normals, boundaries, seam and conditioning against withheld mesh
   samples. Raw export stays untouched. No dynamics/optimizer constraints in Lua.

Start with a concrete capability/mesh example on Flat Circuit before expanding
to every map type. Set numerical acceptance tolerances from collision probes,
mesh resolution and convergence tests; do not reuse a centimetre fit tolerance
as proof of complete topology. A second export should be deterministic for the
same scene inputs/settings, and reversal twice/recipe replay should preserve
geometry and provenance. Include negative tests for gaps, duplicate endpoints,
wrong layer, unsupported intersections, missing edges, and incomplete geometry.

Command roles: `probe`, `export`, `select --recipe`, `validate`, `inspect`.
Use CLI options/recipes with existing level presets, not extra experiment YAMLs
for every route, crop, vehicle or export setting. The prototype currently offers
`probe`, `capture`, `inspect`, `validate`, `compare-log`, `select` and `camera`;
coverage is limited to the managed Track Builder path described in its README.
