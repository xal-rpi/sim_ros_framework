# Compatible 3D and rollover checkpoint

This source checkpoint integrates the BeamNG/ROS bridge with these companion
revisions. The recorded revisions identify compatible source snapshots; branch
names alone do not pin a reproducible version.

| Repository | Branch | Commit |
|---|---|---|
| `xal_vehicle_models` | `exp-3d` | `b68008335b35ea705aa752b2f2f17e91e3242b7f` |
| `drive_trajopt` | `fast_3d_with_rollover` | `fcc3e1468bf7ee7e692912eb99ba4582986513b4` |
| `sim_ros_framework` | `fast_3d_with_rollover` | This checkpoint's Git commit |

## What is versioned

- ROS/Lua implementation, configuration, interfaces, tests and documentation.
- Road geometry tools, browser assets and their three test files.
- Saved Track Builder definitions, environment package metadata and builders.
- Active torque-map C/header/provenance bundles and the complete staged native
  vehicle source export, including its common runtime and source-hash manifest.

The native sources are generated or staged by the companion tooling but serve as
build inputs here. Keeping the selected snapshots makes the Lua mod compilable
without regenerating models. `luamod/build_vehicle.py` verifies their recorded
hashes before compilation. Ownership and regeneration remain upstream; this
repository compiles/packages the staged sources for Linux or Windows BeamNG.

Build outputs, shared libraries, ZIPs, caches, road captures and screenshots stay
ignored. The optional `utv_canam_r322_lift_wheel_torque_identified_20261028` bundle
is not part of this checkpoint. A local copy is retained outside the auto-compiled
policy directory under ignored `local_exports/torque_maps/`. Selecting that map
requires explicitly staging its C/header/provenance files into the policy inputs
and rebuilding the mod.

## Validation and limits

Checkpoint preparation passed 212 Python tests, the shared JavaScript checks
and LuaJIT syntax checks for the staged Lua files. Linux and Windows mod builds
and environment packaging passed using the selected source set. The staged
native source hashes and ZIP integrity were verified, and no generated output
or binary files were staged. Build destinations were temporary outputs, not
installed mods.

The [live Derby report](road_geometry_validation_20261101.md) records browser
save/reopen, native corridor extraction, reverse-route export and actual trajgen
surface fitting. That check did not drive a vehicle, run trajectory optimization
or certify arbitrary closed loops/junctions. Its capture artifacts remain local.

## Known follow-up issues from the bng_* audit

These are recorded for subsequent focused commits; this checkpoint does not claim
they are fixed:

- Sensor receiver failures can exit a worker while the ROS node remains alive;
  setup can skip invalid sensors/controllers and mutates sensor configuration.
- ROS command subscriptions currently depend on usable sensor broadcast streams.
- Bounded logger queues can block sensor callbacks. Logger write failures are
  swallowed, and stopping a crashed worker can report success.
- I/O source names, attach indices and UDP port indices need stricter validation.
- `full_path` expands `~` in the wrong order. Some steering calibration examples
  still describe the obsolete scalar workflow.

Dispatcher lifecycle/ownership, logging reliability and configuration/docs cleanup
should remain separate changes so their behavior and regressions are reviewable.
