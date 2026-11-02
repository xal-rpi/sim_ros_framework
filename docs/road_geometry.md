# Road geometry contracts and ownership

The [tool README](../tools/road_geometry/README.md) contains the working commands
and reading map. This document describes measurements and the downstream boundary.

## Acquisition and editing

`acquisition.py` owns BeamNG connections, explicit scene replacement, Lua injection
and batch requests. `luamod/lua/ge/extensions/xlab/roadGeometry.lua` observes the
native evaluated builder mesh, navigation and collision APIs. Python packs those
observations; it does not reconstruct builder geometry from the asset JSON.

`capture --fresh` loads a vehicle-free tech_ground scene for one managed asset.
`survey --fresh --level PRESET` loads a vehicle-free base level. Without `--fresh`,
survey observes the current scene. `prepare` and `refine-overview` measure context
in the loaded level. `extract-path` verifies the mission and, for network intent,
the saved navigation geometry/directions before measuring. Mission equality does
not prove that collision objects, mods or scene variants match: complete scene
hashing is unavailable and recorded as such.

Native static heights and Engine ray-hit normals/object attributes remain distinct.
The attributes are attached only when the hits agree within 1 mm. Missing heights
remain NaN/false; the authored reference never fills a missing road observation.
The recorded vertical ray band chooses a first hit, not every stacked layer.
Overview grids and nearby context clouds are visualization only. They neither
identify pavement boundaries nor establish connectivity or drivability.

The scene template owns rendering and map selection. `route_editor.js` owns
waypoints, undo/redo, preview and local save/load; `route_planner.js` is the pure
planning implementation also called by the offline Node rebuild. The scripts are
embedded into one standalone page and intentionally share its DOM and scene
rendering functions. Route inspection uses its separate template/viewer because
it selects already measured rows, rather than authoring new acquisition intent.

A saved track recipe is validated by `track_recipe.validate_recipe` for both
local-server saves and extraction. It must include valid reference stations,
source intent, explicit topology, a supported free/network mode, matching source
hashes and complete extraction settings. Network references additionally retain
directed edge identities and optional evaluated-guide associations. Python
independently replays recorded reference operations before acquisition. A save is
a usable recipe, not an incomplete draft. Area bookmarks require finite ENU bounds
and a focus; route selections validate hashes, rows, direction and seam choice.

## Extraction settings

All seven keys below are required in a track recipe's `settings`. Unknown keys,
booleans, nonfinite values and invalid ranges are rejected. The browser supplies
these values when it saves a recipe.

| Setting | Units and meaning |
|---|---|
| `left_extent_m`, `right_extent_m` | Positive horizontal extraction extent from the reference, on each side |
| `extra_margin_m` | Nonnegative additional sampling extent on each side; excluded from requested limits |
| `station_spacing_m` | Positive reference resampling target in metres; actual stations are stored explicitly |
| `lateral_spacing_m` | Positive maximum spacing in horizontal cross-sections, metres |
| `ray_above_m`, `ray_below_m` | Positive vertical search distances above/below reference Z, metres |

Cross-sections are horizontal perpendiculars to the reference tangent. Undefined
tangents raise. Corridors are limited to 250,000 query columns. Fold diagnostics
identify a sampling strip that crosses itself; they do not repair the reference
or certify a fitted surface. A finite reference can still produce missing road
hits, especially with a narrow search band or at overpasses.

Bounded short-node transitions edit the reference only, under a recorded policy.
They cannot silently bridge separate guides, move protected waypoints or repair
large gaps. Rebuilding writes a new recipe and content-addressed guide association
snapshot; prior recipes and their snapshots remain reproducible.

## Common route schema (`0.5-road-routes`)

`capture`, `extract-path` and `select-route` all finish through `route_schema.py`.
Managed assets, measured native corridors (including Derby), and selected routes
share one metadata version, route descriptor and core array vocabulary. The
whole-map native survey remains a distinct acquisition format: it is not a route.
The JSON envelope stays `0.1-road-route-json` for the existing trajgen importer;
its `route_schema_version` identifies the common numeric contract.

`metadata.json.routes` always lists the available routes, including a single
measured corridor. Each descriptor has `name`, `file`, `sha256`, `stations`,
`topology`, `direction` (`array_row_order`), `start_station_id`, `end_station_id`,
`start_xyz`, `end_xyz`, `seam_station_id` (null unless closed), `end_meaning`,
`cross_sections`, `lateral`, `limits`, `station_convention`, and `acquisition`.
Unclassified acquisitions may retain `topology: unknown` for inspection; JSON
can preserve this sentinel. Downstream import requires an explicit open/closed
route selection.
Acquisition records retain source construction, reference role, original direction
and original endpoint/closure evidence. Close endpoints do not imply closure.

Metadata records local ENU, metres, right handed axes, unknown geographic datum,
`T_enu_from_native`, source identity/hashes, `sources`, `surface_objects`,
`surfaces`, normal conventions and unavailable information. Current native-world
coordinates use the same identity transform as GT. Runtime object indices and
builder piece IDs remain separate provenance domains; unknown values stay -1.

All NPZ arrays load with `np.load(path, allow_pickle=False)`. Use the descriptor's
`file` instead of assuming a specific filename: managed routes live under
`routes/subtrack_N.npz`; measured/selected single routes use `route.npz`.
Float arrays are float64; indices/offsets are integers; validity arrays are bool.
XYZ, lateral coordinates, limits and distances use metres; axes/normals are unitless.

| Core route arrays | Shapes / interpretation |
|---|---|
| `station_id`, `source_station_id`, `source_row` | `(N,)` each; source identities and immediate source row. Array row order defines travel order; station IDs need not increase after reversal/seam selection |
| `center_xyz`, `center_valid` | `(N,3)`, `(N,)`; observed surface center; missing Z remains NaN |
| `reference_xyz`, `reference_valid` | `(N,3)`, `(N,)`; separate design/authored/navigation reference, identified in acquisition; never substituted for missing surface |
| `center_distance_xy_m`, `center_distance_xyz_m` | `(N,)` each; cumulative horizontal/3D chords on observed centers, starting at zero |
| `reference_distance_xy_m`, `reference_distance_xyz_m` | `(N,)` each; cumulative horizontal/3D chords on stored reference rows, separately recomputed; not original authored-polyline progress |
| `horizontal_left_axis`, `horizontal_left_valid` | `(N,3)`, `(N,)`; horizontal unit left axis. Collision acquisition retains its query axis; managed acquisition uses world up cross native forward, falling back to a duplicate-skipping reference chord if unavailable |
| `section_xyz`, `section_offsets`, `section_valid` | `(Q,3)`, `(N+1,)`, `(Q,)`; ragged source samples ordered right to left; missing hits stay invalid |
| `section_lateral_xy_m`, `section_lateral_valid` | `(Q,)` each; dot of sample minus measured center XY with that row's horizontal left axis. Left positive; horizontal projection, not banked profile length. XY can remain known when Z is missing |
| `section_reference_xyz` | `(Q,3)`; collision query references; NaN when no separate query was performed |
| `section_normal`, `section_normal_valid` | `(Q,3)`, `(Q,)`; matching native Engine hit normals in ENU, unflipped; NaN/false when unavailable. Mesh face normals remain separate |
| `sampled_limits_xy_m`, `sampled_limits_valid` | `(N,2)` each, `[right,left]`; valid observed sample envelope, not guaranteed continuous coverage or a physical edge |
| `requested_limits_xy_m`, `requested_limits_valid` | `(N,2)` each; chosen extraction extents excluding extra margin; unknown on managed asset captures |
| `physical_limits_xy_m`, `physical_limits_valid` | `(N,2)` each; known usable pavement edges; currently NaN/false |
| `optimization_limits_xy_m`, `optimization_limits_valid` | `(N,2)` each; explicit optimization-domain choice; currently NaN/false |
| `center_surface_index`, `section_surface_index` | `(N,)`, `(Q,)`; indices into metadata `surfaces`, -1 unknown |
| `section_object_index` | `(Q,)`; index into `surface_objects`, -1 unknown |
| `source_ref` | `(N,3)`; native 1-based `(subtrack,piece,evaluated_point)` when available, otherwise -1; column meaning also recorded in acquisition |
| `section_vertex_id` | `(Q,)`; exact `geometry.npz` vertex reference, -1 when no mesh correspondence |

Optional `source_*` arrays retain acquisition-only quantities without giving them
a shared geometric meaning: `source_section_lateral_m` is the original banked
builder projection or horizontal query offset; `source_profile_limits_m`,
`source_profile_right_xyz`, `source_profile_left_xyz` describe the generated strip,
not certified usable edges. `source_forward_axis`/`source_up_axis` are generator
axes, not surface normals. `source_reference_distance_xyz_m` retains the original
authored-polyline progress. Reversal reorders this provenance but does not change
its source orientation. Common lateral coordinates, limits and distances do update.

No distance here is fitted surface progress. Invalid coordinates propagate NaN
through subsequent chord sums; gaps are never bridged. Unknown limits carry both
NaN values and false validity, rather than a fabricated width.

Optional `geometry.npz` remains a physical mesh, independent of routes:
`vertices_xyz (V,3)`, `triangles (F,3)` (zero-based, original winding),
`face_normal (F,3)`, `triangle_source_id (F,)` into `sources`, and
`triangle_surface_index (F,)` into `surfaces`. Source submesh/material/piece
references and raw duplicate vertices are preserved. `source_asset.json` is the
exact deployed builder definition; `complete_scene` is false.


## Surface identities

`surfaces` records native object/material/ground-model identity, not friction
coefficients. Unknown fields are null; unresolved indices are -1. Runtime object
IDs and terrain material indices are local snapshot identifiers. Terrain beneath
a bridge cannot label that bridge's surface. Managed source-profile attribution
also cannot label an occluding object. Material strings are never interpreted as
certified physical properties.

## Publication, migration and validation

New acquisitions finish normalization before publication. Saved legacy bundles
are converted only by `normalize-routes SOURCE --output DESTINATION`. It stages a
copy, normalizes arrays, validates, regenerates optional `road.json` and the viewer,
then renames the completed directory into place. An existing destination, symlinks
or an overlapping source/destination are rejected. Source metadata, arrays,
recipes and collection manifests are never rewritten. Collection owners must
explicitly point a manifest at a new bundle after reviewing it.

Supported input route schemas are `0.1-probe`, `0.3-track-samples`,
`0.4-selected-route` and the canonical `0.5-road-routes`. Older inputs have explicit
interpretations for conversion; arbitrary schemas cannot fall through to a
legacy reader. Recorded previous hash pairs preserve selection recipes whose row
identities and measured geometry survive normalization. They do not authorize
recipes from unrelated acquisitions.

Canonical validation rejects malformed shapes/mask/index types, invalid offsets,
invalid valid-coordinate/normal values, unordered lateral samples, invalid limits,
unresolved surface-table references and declared closed seams with a gap over
1 mm. Verified meshes additionally require finite positions, integer in-range
connectivity, nondegenerate triangles and unit face normals. `validate` writes a
validation report; `export-json` checks without changing the acquisition's report.
These are observation checks. They do not certify the optimizer's fitted chart.

## drive_trajopt / trajgen boundary

`drive_trajopt.trajgen.sites.import_beamng.import_beamng_route` consumes one
materialized `road.json` with schema `0.1-road-route-json`, local_ENU and metres.
The envelope stays unchanged; `route_schema_version` identifies the numeric
bundle contract. Select travel direction, seam and interval upstream first.
The adapter imports center XYZ, section XYZ/offsets, optional world normals,
explicit topology and source station identities. It rejects missing selected
coordinates. Unavailable numeric values serialize as JSON null, never JSON NaN.

The current trajgen `TrackDefinition` requires at least four stations and two
samples per section. Acquisition allows smaller observations for inspection;
exporting one does not claim it is ready for fitting. Normals require explicit
boolean validity masks. Unknown limits remain unknown. Extraction extents and
sample envelopes do not choose an optimization corridor.

Trajgen owns `prepare_track`/route preparation: representation choice, fitting,
chart coordinates, explicit duplicate-seam policy, domain/coverage and fit/fold
acceptance. `drive_trajopt` owns optimization and later compilation/caching and
execution artifacts. This tool never fits a surface, compiles a controller,
sends a vehicle command or smooths/resamples measured XYZ. Source station IDs,
measured chord distances and fitted chart progress remain distinct.
