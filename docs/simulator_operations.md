# Autonomous BeamNG operations

Launch, deployment, scene loading and recording are separate operations. Use the
resolved configuration and live readiness signals to identify what is running.

## Locate the installation and active user folder

Use installation/user-folder configuration or inspect the existing simulator
setup; do not hardcode a username or assume WSL. Relevant settings:

| Setting | Meaning |
|---|---|
| `BEAMNG_HOME` (used in examples here) | Installation directory containing `lua/` and `Bin64/` |
| `BEAMNG_USER` (used in examples here) | Active game user directory containing logs, settings, `mods/` |
| `BEAMNG_MOD_DIR` | Actual output/install directory consumed by both mod build scripts |
| ROS launch `host`, `port` | Simulator tech TCP endpoint, commonly localhost:25252 here |

The first two are example shell variables, not newly implemented discovery
features. If working from Linux against Windows, use the mounted paths for file
access and Windows paths for Windows process invocation. Native Linux uses its
native installation/user folder and the Linux mod target. Check the active
profile, not an old version/cache directory.

The sources inspected for the 2026-10-16 road investigation were the user's
BeamNG.tech 0.38.3.0 installation. The launch configurations connect to a running
game (`launch: false`); starting ROS alone does not start the executable.

## Game modes and physics pacing

| Launch mode | Arguments | Rendering |
|---|---|---|
| Windowed | normal launch | UI and GPU rendering |
| Headless | `-headless` | No game window/CEF UI; GPU rendering remains |
| No GPU | `-headless -gfx null` | Rendering disabled; camera-derived sensors unavailable |

See [BeamNG's mode documentation](https://documentation.beamng.com/beamng_tech/headless_mode/).
Use the same installation, active user folder and mods for comparisons. Check
custom-track collision, sensor freshness and graphics/control callbacks in the
chosen mode. Removing rendering alone does not establish repeatable control.

The supplied profiles connect to an existing game (`beamng.open_args.launch: false`).
BeamNGpy `headless`/`nogpu` are constructor launch options under `beamng`; connecting
to an existing process does not change its mode. A mode change needs a new process.
Keep game-process ownership separate from scenario restart and ROS-node ownership.

`defaults/sim.yaml` selects physics speed factor -1: each graphics frame advances
1/fps vehicle seconds using the game's existing frame limit. Select
`compose.sim: defaults/sim_fixed_step.yaml` to also set that frame limit to 60 Hz.
For wall-clock pacing, override the same setup call:

```yaml
overrides:
  beamng:
    setup_funcs:
      beamng.settings.set_deterministic:
        speed_factor: 0
```

The API name does not determine pacing: `speed_factor: 0` selects wall time.
`steps_per_second: 0` only skips setting the frame limit; it does not select wall
time. Frame groups leave the native physics rate unchanged, normally 2000 Hz.
LLC `controllerRate` is independent. See [physics pacing](https://documentation.beamng.com/beamng_tech/deterministic_mode/).

For repeatable batches, record actual launch arguments, user/mod identities,
physics speed factor, foreground/background frame limits, model/reference identity,
and measured vehicle-time/wall-time ratio. Do not infer pacing from a YAML label.

## Startup handoff for automation

Read this section before starting or replacing the game. The existing BeamNGpy
launcher successfully started Windows BeamNG.tech 0.38.3.0 from WSL in headless,
no-GPU and windowed modes. No separate process-launch framework is needed.

1. Resolve the executable, active user profile, mods and requested mode. The WSL
   probe used a mounted installation path, relative binary
   `Bin64/BeamNG.tech.x64.exe`, and a Windows user-folder path (parent of `current/`).
2. Identify any existing primary game and its profile. `open(launch=True)` reuses
   a reachable API; it does not apply a new mode to that process. Changing mode
   requires closing the identified game gracefully and waiting for its exit.
3. Launch with BeamNGpy `headless=True` or `nogpu=True` and the native-library
   flags, then wait for the API handshake. Load `xlab/xlabCore` through TechCom
   after mods mount; normal ROS connection already requests and verifies it.
   Passing a mod extension to BeamNGpy's fresh-process launch emits an early
   `-lua extensions.load(...)` callback and can report `extension unavailable`
   before `xlab.zip` mounts. Treat successful post-start extension verification
   as required. Verify actual process arguments/profile, not requested mode alone.
4. Use normal ROS bringup / `drive-run` for scenario loading and recording. Wait
   for reference, measurements, accepted horizon and logger readiness before
   engagement. Check custom-track collision and measurement validity in a new mode.
5. Track game ownership separately. ROS shutdown currently disconnects and leaves
   the game open; automated game shutdown must target only a game owned by the run.

Both non-windowed modes completed the three-lap banked-oval scenario with GTState,
surface sampling, torque estimates and 100 Hz LLC execution. Graphics callbacks
continued in both modes. This establishes compatibility, not repeatability or a
tracking improvement.

On the tested BeamNG.tech 0.38.3.0 / BeamNGpy 1.35.1 setup, full stop/load in
no-GPU mode reproducibly crashes on the second load. A stock scenario in an empty
profile reproduces the same native double-free without our Lua or mods. Disabling
shader precompilation and reusing scenario files do not prevent it. Headless GPU
rendering passed three stock full reloads and three normal ROS/LLC startups.
Use headless GPU rendering for full-reload batches on this installation, or a fresh
game per no-GPU batch. Two stock same-scene restarts passed in no-GPU mode; production
controller reset and map/vehicle changes are not qualified by that check.

Local launch/recording evidence: `drive_trajopt/experiments/headless_focus_20261031/README.md`.
Reload reproduction, debugger reports and lifecycle checks:
`drive_trajopt/experiments/nogpu_reload_20261031/README.md` in the companion repo.

## Build and deploy mods

From the repository root, after pointing `BEAMNG_MOD_DIR` at the active `mods/`:

```bash
python3 environments/build.py
bash luamod/build.bash --win
```

The repository script is the portable build command. `--win` targets the Windows
simulator even when the build runs on Linux. Use `--linux` for native Linux;
Windows native libraries require the MinGW cross compiler in this Bash build.
All runtime Lua/JSON sources are packaged regardless of Git staging.

The builds are independent: `xlab.zip` contains integration Lua/native libraries;
`xlab_environments.zip` contains Track Builder assets and their generated hash
manifest. Both builders use explicit `--output` first, then `BEAMNG_MOD_DIR`, then their
package-local `dist/` directory with a warning. Both preserve the previous ZIP if
a build fails. The Bash builder takes `--output=/path/xlab.zip`; the Python
builder accepts `--output /path/xlab_environments.zip`.
An output in local `dist/` is not installed in the game. Copy/deploy deliberately
when the build destination is not the active mods folder.

Check the deployed ZIP entries and contents/checksums, not just its filename.
Do not install both a stale unpacked mod and a new ZIP over the same virtual
paths. Python/config changes need the appropriate ROS package build/source;
an environment-only update does not itself require a new integration mod.

## Restart and observe the simulator

For the Windows simulator, the user supplied this invocation. In PowerShell:

```powershell
& "$env:BEAMNG_HOME\Bin64\BeamNG.tech.x64.exe" -tcom -colorStdOutLog -disable-sandbox -nosteam -console
```

Use the actual installation variable/path in that Windows shell. The
`-disable-sandbox` option is used by this integration's native torque-map DLLs;
it is not necessary for reading road geometry alone. `-nosteam` is optional.
Do not use the Windows binary for a native Linux simulator. Discover the
installed Linux launch executable/options rather than guessing a Windows path.

Identify the specific simulator process before terminating anything. Prefer a
graceful close of that instance, wait for exit, then start one replacement.
Do not kill unrelated BeamNG, ROS or controller processes. When changing
integration Lua/assets, a full game restart is the most reproducible way to
avoid an old loaded extension or resource cache. Python-only changes normally
require restarting the corresponding ROS process, not the game.

Wait for tech TCP readiness and a successful BeamNGpy handshake, not just a
fixed sleep. Use bounded polls and inspect logs if it fails. The active user
folder's `beamng.log` is the primary persistent GE/vehicle Lua log; keep the
console/stdout too when launching with `-colorStdOutLog -console`. Record the
start point/offset so stale errors from an earlier session are not misattributed.
Look for fatal TechCom handling errors, Lua tracebacks, missing handlers/mods,
failed collision loads and acknowledgements. A base level's "Level loaded" line
does not prove the custom asset or sensors initialized successfully.

For a read-only GE inspection without ROS, an already running simulator can be
connected with BeamNGpy (`open(launch=False)`) and queried with
`control.queue_lua_command(chunk, response=True)`. Disconnect when finished;
do not send quit, load a level or move a vehicle as part of a capability query.
If the ROS manager owns the connection, prefer its request proxy to avoid
competing clients. Connection arguments must match the running simulator.
Network/process/filesystem actions still follow the execution environment's
approval requirements; task authorization does not override sandbox access.

## Load test scenes without multiplying YAMLs

Example of the established framework launch after building/sourcing ROS:

```bash
ros2 launch bng_bringup simulator.launch.py \
  config:=tech_ground_oval.yaml \
  level:=tech_ground_with_flat_circuit spawn:=on_track
```

Use `level:=tech_ground_with_rolling_hills_circuit` for Rolling Hills, or omit
`level` for the existing banked donut run. This run selects the easy-lift Can-Am;
`single_vehicle.yaml` defaults to Covet Turbo Rally unless `vehicle:=` overrides it. `level`
selects an environment YAML preset, not a newly registered Freeroam map. Reuse
run/preset/CLI options instead of creating a run YAML for every combination.

CREATE mode checks for a loaded level, stops it if present, then loads the new
scenario, vehicles and environment assets. A level-query or stop failure propagates
instead of being treated as an empty scene. It also starts the vehicle runtime,
settling, sensors and LLC; it is not
a pure geometry-only loader. A geometry export tool should instead support GE
scene acquisition without any vehicle/LLC. Do not engage MPC, issue drive/AI
commands, enable cruise, or create trajectory command publishers for extraction.
Do not drive a car merely to populate a Road Sensor. A fresh test scene may be
needed to enable capture before Track Builder discards its evaluated tables.

Check the printed manifest for resolved vehicle, actual base level, selected
spawn position/yaw and UDP ports. Position overrides are resolved before initial
creation; later explicit-pose restoration uses that same configuration. `cling`
false preserves elevated spawn intent, but the prefab/load/start/replacement
sequence still needs collision-aware testing. Spawn Z is a vehicle reference
position, not CoG or road height. Do not infer road elevation from spawn success.

Wait for `sim_ready` and expected asset load/sensor acknowledgements before
recording. Multiple ROS launchers or loggers against the same ports are a
failure mode; record process IDs and stop only processes started by the test.

## Programmatic requests and logging

The generic service is `execute_request` (`bng_msgs/srv/ExecuteRequest`). Inspect
`core/request_handler.py` before using it: it routes known requests, manager
methods and `beamng.*` API calls. `utils/services_utils.py:send_request` offers a
bounded client. For repeated calls, the existing `GarageServiceClient` is an
example of one ROS context/client kept across polls; garage actions themselves
are unrelated to geometry export. See the [support garage guide](support_garage.md)
for experiment configuration, output identity, restoration and recovery.

Use `get_sim_config` for deployed assets, hashes, selected scene and resolved
settings. Read-only BeamNG method calls can go through the manager's connection,
for example `beamng.scenario.get_road_network` with explicit edge/filter options.
There is no ROS road-export service yet. The [standalone geometry prototype](../tools/road_geometry/README.md)
connects through BeamNGpy, injects the local capture module into GE and uses a
fresh vehicle-free scene; it does not need a new mod install or ROS manager.
Its `inspect` and `select` commands are offline; `camera` moves only the free
camera. Do not run its `capture --fresh` against a scene that must be preserved.

For native maps, `survey --level derby --fresh --bounds Xmin Ymin Zmin Xmax
Ymax Zmax --output NEW_FOLDER` resolves the existing ROS base-level preset and
loads a paused zero-vehicle test. Omit `--fresh` to observe the already loaded
scene without changing its pause/camera/vehicles. Custom objects/environment
additions require loading through ROS first; the pure loader rejects such
presets rather than omitting additions. `survey` saves native decal guides and
bounded collision points, not invented pavement triangles. The Derby examples
and WSL viewer instructions are in the tool README. No production launch adds
an automatic export or per-frame geometry task.

For selection context, `prepare EXPORT_FOLDER --spacing 3 --height-band ZMIN
ZMAX` adds a coarse full terrain/road-footprint point cloud and native directed
navigation graph to an existing survey. `derby/inspect.html` is the retained main
Derby entry point. Its waypoint editor saves a `0.3-track-recipe`; `extract-path
EXPORT_FOLDER recipe.json --output EXPORT_FOLDER/tracks/NAME` measures lateral
surface sections without any vehicle/controller. Saved inspection pages work
offline; acquisition still requires the intended scene loaded. These commands
inject the current local Lua probe; no integration-mod rebuild is necessary.
Do not treat overview colours, navgraph radii, or chosen corridor extents as
physical drivable boundaries. See the tool README for schema and test results.

The recommended interactive entry is `serve EXPORT_FOLDER --http-port 8766`:
open `http://localhost:8766/inspect.html` in the browser. Its file API saves named
tracks/areas under that export's `selections/` and keeps overwritten versions in
`.history/`. Static HTML/generic HTTP hosting can only download to the browser's
chosen directory. The retained Derby editor has a 1 m overview under `overviews/`
and an active `overview_current.json`; the original 3 m preview is preserved.
`refine-overview EXPORT_FOLDER --spacing 1` captures another density explicitly.

Road-following mode now associates native graph links with evaluated guide curves.
Do not call graph chords accurate road centerlines: the imported user test showed
up to 1.85 m XY disagreement with evaluated guides. Legacy recipes retain their
original interpretation; selecting evaluated curves is an explicit conversion.
`navigation_geometry.json` keeps association provenance, endpoint connectors and
unsupported fallback links. `0.4-track-recipe` records its hash. Original map,
road fields and native navigation snapshots are unchanged.

The 0.3.0 environment package additionally registers `banked_oval_b15`
(`Oval_15.json`), selected with `level:=tech_ground_with_banked_oval` using the
existing run. `tech_ground_with_oval` selects the original donut asset, corrected
into a closed circle in package 0.3.1.

Native route authoring now uses branch-aware local guide projections and an
explicit bounded same-guide reference transition option. `rebuild-reference
SOURCE_SURVEY OLD_RECIPE --output NEW_RECIPE` replans with the browser's functions
(Node.js required), records waypoint reprojections and transition intervals,
and keeps old guide association snapshots reproducible. `extract-path` validates
the operation with independent Python replay and queries the new corridor; do
not reuse the old sampled surface. The corrected current Derby export uses this
operation. Its `reference_validation.json` records actual downstream fit/chart
checks. No mod rebuild or automatic per-launch/per-frame sampler was added.
Full evaluated capture requires `capture --fresh --asset ASSET --output NEW_DIR`;
native terrain/material properties are queried only on actual terrain hits,
through a typed `scenetree.findObjectById(hit.obj:getID())` object. The generic
Engine ray-hit wrapper lacks TerrainBlock-specific methods even though its class
name says TerrainBlock. Do not silently catch that binding failure and assume
names are unavailable. Runtime mesh Material `groundtype` resolves managed
track names after defaults; do not guess it from the source filename.

Current four-asset and selected Derby-route examples are grouped in
`environments/dist/road_geometry/route_exports_v2/`. `export-json EXPORT
--route subtrack_1 --include-mesh --output NEW_FILE.json` writes measured route
arrays plus mesh/face normals for Python consumers; it is distinct from a
waypoint recipe. Native measured corridors omit `--route` and `--include-mesh`.
Nothing exports at scenario launch or on an update callback.

Package 0.3.1 fixes the old donut's redundant 4 m entry straight: remove that
piece, renumber material piece references, and move its design seam by the
removed straight's displacement so the existing 15 m circle stays in place.
The preset spawn is now `[34,-30,5.5]`, yaw 0. Corrected exports live in
`route_exports_v2`. Historical folders were deleted on request; comparison
results and source hashes remain recorded in the current collection manifest.
All managed assets explicitly choose closed/native-order/first-station routes
through manifest `route_defaults`, validated against native position and
forward/up seam frames. These are choices in route preparation, not tasks for
the trajectory optimizer. Derby still needs an explicit selected corridor.

Temporary HTTP preview servers must be stopped after use, per user preference.
Identify only the server processes started for this task, terminate them and
verify their localhost listening ports are closed. Geometry export itself
starts no server. Prefer offline self-contained HTML when no live editor is needed.

Saved managed tracks and measured native corridors now share `route_viewer.html`
and `route_viewer.js`; the original mesh-only template has been removed. The
full native map authoring page is still `scene_viewer.html`, sharing workspace
styles. Refresh a saved page with `inspect EXPORT`. Named route choices use
`0.1-route-selection` and `select-route EXPORT RECIPE --output NEW_FOLDER`;
the local export server saves choices under `selections/routes/` and creates
separate arrays under `exports/NAME/`. Closed loops replace only the terminal
seam copy; interior duplicates stay. Reverse reorders section samples and swaps
side ownership without flipping world normals. Direct file pages work offline
and download recipes; no server is needed for viewing or CLI materialization.

Nearby `context.npz`/`context.json` files are optional visualization observations.
Use `add-context EXPORT --survey SOURCE --radius 25` offline for native routes,
or `add-context MANAGED_EXPORT --fresh --radius 25 --spacing 1` to load a paused
zero-vehicle managed scene and query native collision. The deployed asset hash
must match the original capture. Existing context is never silently replaced.
Point clouds retain all observed Z layers within their source's first-hit band;
they do not certify drivability. No automatic acquisition occurs while viewing.

The sensor logger has a noninteractive session API:

```python
from bng_simulator.utils.log_session import LoggerClient, begin_run, end_run
import rclpy

rclpy.init()
client = LoggerClient()
try:
    run = begin_run(client)  # vehicle folder/config obtained from simulator
    # Wait/measure while keeping the vehicle unengaged.
    output = end_run(client, run, {
        "map_name": "Flat Circuit",
        "additional_info": "stationary road-export validation",
    })
finally:
    client.destroy_node()
    rclpy.shutdown()
```

Production test code must also stop/finalize its own session on errors. These
helpers start/stop raw sensor logging and consolidate pickle data; they do not
automatically launch rosbag or the CLI's MPC snapshot thread. Supply optional
metadata/snapshots explicitly when needed. `start_logs` is the interactive
wrapper; it records all ROS topics by default, supports MCAP and captures current
MPC tracking metadata when its service is present. Do not invoke its interactive
prompts unattended. Geometry extraction does not require new driving recordings.

`drive-run` uses the same vehicle name and `get_next_run_folder()` allocation as
standalone `start_logs`: `~/beamng_log_data/<vehicle>/run_XXX/`. It puts resolved
configuration and terminal logs under `session/`, and passes `--run-folder` plus
`--managed-run` to the logger. Recording data, bags and reference snapshots live
directly in the run folder. Discarding a managed recording preserves `session/`;
standalone discard still removes the entire run folder. `--managed-run` requires
an existing `session/` directory. No existing recordings are relocated.

## Retained evidence and frame rules

Runs `utv_canam_x3_r322_easylift/run_032` and `run_033` under the configured log
root are Flat Circuit and Rolling Hills respectively, confirmed by their asset
hashes in metadata. They are useful independent-position checks, not full maps.
Use raw `surface*_queryPos`, `surface*_height`, normal and validity fields.
`GtStateMsg` omits this raw data; do not modify that ROS message for this task.

Keep the established local ENU world frame exactly as GT logging uses it.
Body FLU is a separate frame. Geometry alignment must be an explicit recorded
proper rigid transform, never an undocumented axis flip or map-height offset.
Do not edit vanilla level files, assume a global centerline, fabricate closure,
or interpret unavailable friction as zero grip. Saved design height and default
mesh profile can differ from actual pavement height; validate with evaluated
mesh/collision geometry.

For each test, retain version/asset/source hashes, settings, commands, process
IDs, log excerpts, export files and inspection diagnostics. Report separately
what was inferred from source, observed at runtime, or validated numerically.

## Surface geometry and wheel material fields in gtState

Enable `sensors.gtstate.surface` (enabled in the LLC and attach defaults):

```yaml
surface:
  enable: true
  update_time: 0.0  # every GT sample
  normal_radius: 0.15
  max_distance: 100.0
  terrain_friction: true  # optional live terrain lookup at graphics cadence
  profile: true
  profile_interval: 5.0
```

The sampler publishes top-level fields with prefixes `surfaceCoG`, `surfaceFL`,
`surfaceFR`, `surfaceRL`, and `surfaceRR`. Each has `_queryPos`, `_height`,
`_heightValid`, `_normal`, and `_normalValid`. Positions and upward unit normals
use the same world frame as GT `pos` and `quat`; no additional axis conversion
is introduced. Wheel queries use the world position of the hub midpoint, not
the contact node. Geometry remains available while airborne.

Each geometry update uses at most 20 bounded downward static rays: one height
ray and three normal-stencil rays at each location. Rays start 0.5 m above the query
point; stencil rays start near the found surface. This samples static geometry,
not other vehicles or moving supports. The configured period limits geometry
updates to at most the GT sampling rate; intervening GT records copy the last
geometry, including its query position. There are no extra sample timestamps.
`update_time: 0` (the default) refreshes geometry on every GT sample. A positive
period trades cost against geometry freshness. Abrupt stencil
height differences greater than twice `normal_radius` invalidate the normal.
Height and normal failures are independently flagged; invalid numbers are zero.

Wheel prefixes also have `_contactMaterialID1`, `_contactMaterialID2`,
`_staticFriction`, `_slidingFriction`, and `_frictionValid`. IDs are copied from
BeamNG's existing graphics-rate wheel cache without invoking wheel-core getters
or requiring the contact-wrench sampler. These are collision material IDs,
not ground-model registry IDs. Material 4 is rubber in BeamNG 0.38.3; the other
ID is used only when paired with rubber. With `terrain_friction: true` (enabled
in the LLC/attach defaults), each vehicle graphics callback requests a batch of
four terrain-material lookups in GE Lua. Material `groundmodelName` selects the
exact live model through `be:getGroundModel(name)`. Each distinct model is read
once per batch, including current values changed by surface-emulator mods.
Replies update a vehicle-side cache; GT physics samples only copy the result.
This graphics cadence is independent of the GT `gfx_update_time` save period.

Only one batch may be pending. After one simulated second without a reply the
cache is invalidated and a new batch is issued; old replies are rejected by
sequence number. The result must match the copied contact-ID pair and the live
model's collision type. Missing terrain/material/model, unsupported bindings,
no rubber contact, or terrain height differing from the static surface hit by
more than 2 cm produce invalid friction with zero coefficients. A bridge or
raised mesh is therefore rejected; a coplanar mesh can still be misidentified.
Queued replies introduce a short lag; same-category material transitions can
retain the previous coefficients until the next reply. No sample timestamps are
added. Geometry remains available while airborne; friction needs a valid rubber
contact pair. IDs alone do not prove current wheel support.

Set `terrain_friction: false` to disable the live queries and cross-Lua messages.
The previous conservative startup lookup is then used: coefficients are valid
only when all registered ground models sharing the collision type agree. That
fallback does not track in-run model edits; recreate the sensor to refresh it.

The existing pickle logger expands the vectors into `_x`, `_y`, `_z` channels.
The surface fields are carried in raw GT readings and pickle logs;
`GtStateMsg` and its ROS conversion remain unchanged. Downstream processing can combine the CoG normal and
existing quaternion to calculate relative attitude; the model repository's
surface preprocessing is not changed by this producer addition.

With `profile: true`, `GtState.surface` prints one report every
`profile_interval` simulated seconds (default 5). `avg` and `max` measure the
complete sampler call, including queries and copies. `budget` is measured sampler
wall time as a percentage of elapsed simulated time; `sim/real` is simulated
seconds divided by wall seconds for that window. For example, `sim/real=0.8x`
means five simulated seconds took about 6.25 wall seconds. Pauses, slow motion,
external pacing and other game work affect that ratio: compare equivalent runs
with `surface.enable` on/off before attributing slowdown to geometry queries.
Profiling creates two timers once, reads them around calls, and formats output
only per report. Set `profile: false` to disable the instrumentation entirely.
These diagnostics do not add fields to GT records or change `GtStateMsg`.

