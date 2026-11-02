# xlab feedback plugin contract (vlua)

Optional setpoint layer between the active **actuation controller** (`controller_<type>`)
and vehicle inputs. Loaded by `controller_manager` when YAML sets `feedback: <stem>`.

## Placement

```
control_listen → controller (store commands)
              → controller.resolveSetpoints(plant)
              → feedback.transform(ctx)   [optional]
              → controller.applySetpoints(plant, sp_eff)
```

For an LLC with `roll_filter` configured, the existing command timeout selects
processed manual input when no fresh external command exists. Manual steps skip
`feedback.transform`; the physical roll-filter hook remains downstream.

Feedback is **not** part of `controller_llc`. Any actuation plugin that implements the
split API below can use the same feedback modules.

## YAML config

```yaml
controllers:
  LowLevelController:
    controllerType: llc
    feedback: passthrough    # omit, null, or "none" → disabled
    feedback_config:          # plugin-specific; hot-tunable
      example_gain: 1.0
```

Keys are passed through `OpenController`; keep `feedback_config` beside `calibration`.

## Feedback plugin file

Path: `lua/vehicle/controller/xlab/feedback_<stem>.lua`

Plugins are loaded with plain `require` (no `pcall`). Load errors, `init` failures,
and `transform` runtime errors propagate to the BeamNG terminal — fail fast by design.

| Callback | Required | Purpose |
|----------|----------|---------|
| `init(common, cfg)` | yes | `cfg.settings`, `cfg.stem`; allocate private state |
| `transform(ctx)` | yes | Return effective setpoints for this tick |
| `onTune(data)` | no | Apply `data.feedback_config` |
| `reset()` | no | Clear internal state |
| `stop()` | no | Teardown |

## `transform(ctx)` input

| Field | Type | Description |
|-------|------|-------------|
| `sim_t` | number | Simulation time [s] |
| `dt` | number | Control period for this step [s] |
| `plant` | table | Actuation controller plant snapshot |
| `resolved` | table | Setpoints after controller projection (this tick) |
| `raw` | table or nil | Borrowed command view from `getRawTargets()`; consume synchronously, do not retain/mutate |

### `resolved` / output setpoint fields

| Field | Unit | Notes |
|-------|------|-------|
| `torque_des` | N·m | Feedforward torque |
| `omega_des` | m/s | Rear wheel speed target |
| `steer_des_rad` | rad | Roadwheel angle |
| `brake_des` | 0..1 | Brake demand |
| `throttle_override` | 0..1 | If set, actuation bypasses policy + speed PI |

Return a complete setpoint table: the manager does not merge missing fields.
`nil` (or another non-table except `false`) uses the original `resolved` table.
`false` skips actuation for that step; TVLQR clears external authority before returning it.
`passthrough` returns `resolved` as-is.

## Actuation controller split API (for feedback orchestration)

When `feedback` is configured, `controller_manager` requires:

| Method | Purpose |
|--------|---------|
| `prepareControlStep(dt, common)` | Rate decimation, timeout, gt read; returns step table or nil |
| `resolveSetpoints(plant, dt)` | Trajectory/scalar resolution → nominal setpoints and outgoing rates |
| `getRawTargets()` | Optional borrowed command view |
| `applySetpoints(plant, sp, dt, step)` | PI + policy + vehicle I/O |
| `finishControlStep(common, step)` | Per-tick cleanup |

If `feedback` is set but the actuation plugin lacks the split API above, the manager
**errors** (fail-fast) — there is no fallback to `update()`.

## Tuning

```json
{ "type": "tune", "data": { "feedback_config": { "k_slip": 0.5 } } }
```

Actuation gains (`gains`, `kp`, `ki`, …) still route to the active controller only.

## Built-in plugins

| Stem | File | Behavior |
|------|------|----------|
| `passthrough` | `feedback_passthrough.lua` | Returns `ctx.resolved` unchanged (pipeline smoke test) |
| `tvlqr` | `feedback_tvlqr.lua` | Direct steering/torque/optional brake correction from the paired XLG1 plan |


## Roll-filter command contract

The optional `roll_filter.lua` implements this contract at the final physical
command hook in `controller_llc`. Environment reception alone changes no commands.
Omitting `roll_filter` loads no roll adapter. The supplied
`roll_filter_shadow.yaml` bringup preset evaluates the filter without changing
commands; `mode: active` enables bounded QP/recovery execution. `mode: off` skips
roll observation/evaluation and restores manual input. Bypass also skips observation.

```text
trajectory/scalar resolution → optional feedback → physical target composition
    → nominal rate construction → admissible bounds → native QP
    → status decision → next physical targets → actuator conversion
```

### 1. Compose physical targets

Include torque PI before filtering. Keep supplied reference steering rate [rad/s]
and torque rate [N·m/s] alongside their angle/torque targets. These are reference
quantities, distinct from the rates of the commands actually emitted. Do not
silently replace a supplied reference rate with a derivative of measured motion.

The existing LLC still smooths and rate-limits throttle after inverse conversion,
as in previous experiments. These are retained actuator dynamics, not operations
on the QP torque variable. Consequently executed torque need not instantly equal
the QP target. No claim of exact barrier enforcement through that conditioning,
actuator lag or torque-map error is made.

### 2. Construct one consistent nominal steering command

For manual input, evaluate the current stock-processed pedal and measured raw
mean front-wheel rate from shared control state. Do not box the free human motion
before checking the row. A nonnegative unboxed nominal margin passes through even
when the automatic box is infeasible. During steering ownership, use the retained
driver angle minus the owned reference, divided by dt. Release requires both a
favorable joint row and a reachable driver target; assisted measured motion alone
cannot authorize handback. An out-of-range retained target may keep assistance
owned until reachable, bypassed or filtering becomes unavailable.

Per-axis ownership comes from the same joint solution. Recheck the emitted pair
when releasing a channel; dropping a small necessary correction cannot invalidate
an otherwise feasible row. Measurements stay unmodified outside automatic bounds.
The normalized command-map endpoint is separate from automatic angle limits;
project an unrepresentable takeover anchor before constructing bounds. Reject an
unrepresentable output pair before either axis is emitted, never post-clip one
axis of a solved pair. Reference rate limits do not bound actual wheel motion.


The consumer must provide a steering anchor, a nominal rate and a duration for
which `delta_nom_next = delta_anchor + dt * rate_nom`. The current adapter consumes absolute next targets and infers
`(delta_nom_next - delta_anchor)/dt`. The command anchor is the previous emitted
angle target. On activation only, initialize it from measured steering rather than
an arbitrary zero. Optional supplied reference rates accompany the target as
feedforward metadata; they are not added a second time. Rate-only execution is
not implemented yet.

An absolute next target is already a target for this actuation interval: do not
add its finite-difference rate again. If a reference angle and feedforward rate
are both supplied, their timing/alignment belongs to target resolution. Preserve
the feedforward rate for local feedback and logging even when target matching
requires a different nominal command rate.

Any optional rate smoothing belongs before bound construction and the QP. If it
changes the nominal rate, reconstruct its nominal next target from the same anchor
and dt; do not retain an inconsistent original angle target. Smoothing adds nominal
command lag and must not be silently enabled.

Use the actual control interval, not the MPC solve period or packet interarrival
time. Reference replacement must not reset the emitted-command anchor. Measured
steering belongs to the model state; a command anchor is not a measurement of
instantaneous wheel motion. The latter distinction matters with actuator lag.

Torque is an absolute QP variable. A supplied torque rate may advance a rate-based
reference, but the filtered absolute torque must not be integrated a second time.

### 3. Form bounds before solving

Use the live GTState torque capability interval. Missing raw measurements are
reported invalid rather than being interpreted as a zero capability interval. If a
finite command slew rate is configured, further intersect with
`[T_previous - torque_rate_max * dt, T_previous + torque_rate_max * dt]`.
If the physical and slew intervals are disjoint, physical rails take priority:
restrict torque to the physical point nearest the previous target, then solve the
same QP with steering still available. This minimizes unavoidable command-slew
violation. `rf_torque_slew_excess` records that minimum excess [Nm beyond the
allowed per-step change]. It is zero when the intervals overlap. Native QP status
still separately reports barrier feasibility. Shadow mode remains passthrough.
`T_previous` is the last emitted physical torque target, not the previous raw
reference or a fresh noisy torque estimate. Initialize it explicitly on activation.

Intersect steering-rate limits with angle reachability from the same command
anchor: `[(delta_min-anchor)/dt, (delta_max-anchor)/dt]`. The native QP must see
these final bounds; do not perform a second independent clip after its solution.

Disjoint physical torque/slew intervals use the documented nearest-physical-point
rule above and report the minimum slew excess. Invalid physical rails or an empty
steering-rate/angle intersection still report status -4 and retain the finite
nominal target; that error path provides no safety guarantee. Endpoints are never
swapped. Torque slew is relaxed only for the explicit physical-priority case.

### 4. Keep the QP metric fixed

Use positive, fixed steering-rate and torque scales in the objective. Defaults
may be chosen once from configured nominal ranges; live envelopes and per-step
slew intersections change feasibility, not the metric. Only an explicit tune
update changes the scales. Zero-width current bounds do not create a zero scale.

### 5. Interpret status and reconstruct targets

For `RF_OK`, reconstruct `delta_next = anchor + dt * rate_filtered` and use the
returned absolute torque directly. The emitted torque rate is
`(T_filtered - T_previous)/dt`; it is not another QP decision variable.

`RF_INFEASIBLE` returns a finite bounded maximum-margin candidate. It is neither
an invalid output nor a certified safe recovery. With `recovery_enabled: true`
(default), selected infeasibility invokes native front-wheel alignment using the
same prepared patch/parameters. The small Lua policy advances the reference toward
that target within the existing rate/angle interval and selects
`min(nominal_torque, 0)` within physical/slew torque bounds. It serves manual and
external angle/torque commands; no separate angle threshold or dwell is added.
Zero torque differs from zero pedal/coasting torque. `torque_correction: false`
still prevents throttle ownership. Disable recovery explicitly to compare the
native corner or the existing steering-only penalty. Alignment minimizes weighted
front slip; it does not certify zero lateral force or recovery. Other failure statuses can return NaNs: never integrate or execute them.
In diagnostic-only operation, log candidates but retain the nominal actuation
path; do not advance an emitted-command anchor using a candidate that was not sent.

Diagnostics use a small fixed set of GTState custom fields, registered through
`registerCustomField` and updated with `setCustomField`. Reuse existing emitted
steering/torque fields. Do not add a stream or duplicate state, terrain packets,
full QP rows or configuration matrices in every sample.


### Configuration, tuning and lifecycle

Vehicle-specific export selection and startup filter settings live in
`bng_bringup/config/vehicle_catalog.yaml`, under the vehicle’s
`controllers.LowLevelController`. Presets only override these settings. All native settings and
units are also documented in `roll_filter.lua` and `roll_filter.h`. Defaults are
35 degrees, tau=0.3 s, alpha=1/tau, packed
P=[37,31,8,52,13,7]/16, observer gain=10/s, observer maximum gap=0.05 s.
Vehicle catalogs and run overrides select the actual settings; inspect their
resolved configuration rather than assuming the module defaults. Fixed objective scales are 10 rad/s and 500 Nm. Explicit startup steering angle
limits must fit the calibrated BeamNG input range. A steering calibration change
with roll filtering configured requires controller recreation.

A total-speed gate uses sqrt(vx²+vy²+vz²): enable at 1.0 m/s and disable at
0.5 m/s by default. Below the gate, model/QP evaluation is skipped, nominal
commands pass through, command anchors are cleared and the observer is reset.
Reactivation seeds steering from measurement and initializes a fresh observer.
Both thresholds are tunable; the gate is not a low-speed safety guarantee.

The manager prepares observations on new GTState samples independently of command
activity. Timeout clears command anchors. Bypass skips observation and releases controls;
on resume, the existing observer gap handling applies. Controller reset clears
the observer and measured-rate histories too. Gain changes, time regression and an explicit `reset_observer: true`
tune reset the estimate. Ordinary patch replacement does not. Call that explicit
reset after a discontinuous physical-model/parameter change; a new export requires
a fresh vehicle VM. Bias application is independent of estimation.

Tune only the `roll_filter` namespace; invalid changes preserve its previous
settings. Steering angles are startup-only; steering-rate tuning cannot exceed
its startup capability. Updates do not rebuild native code. Example:

```json
{"type":"tune","data":{"roll_filter":{"mode":"active","apply_bias":true}}}
```

Scalar packets may carry optional `steering_rate` [rad/s] and `torque_rate` [Nm/s].
Trajectory plans carry actuator targets without rate arrays. The filter derives
target-matching rates from the previous issued command and the current interval. It handles full-authority angle/torque paths with supplied service brake.
Direct throttle/input commands and partial authority retain existing actuation
and report unavailable (-3), without claiming intervention.

Service brake is exogenous: the QP still has only steering rate and rear drive
torque as decisions. `rf_prepare` (ABI 4) takes normalized brake [0,1] and brake
rate [1/s] after measured steering. Native continuous-time rows include the brake
chain-rule term. This adapter uses held values with rate zero. It updates the
observer from GTState's applied brake, then evaluates the row and alignment on a
separate query with the exact normalized brake the LLC will emit; manual mode
uses the captured stock-processed pedal. It does not differentiate pedal steps
or advance the observer twice for one state timestamp. Drive torque excludes
service-brake torque, which is already accounted for in the physical model.

Brake presses may change h discontinuously. A feasible derivative row after the
press cannot certify the jump or unsampled pedal changes. `rf_brake_h_jump`
reports the same-state h change from observed to scheduled brake. Parking brake
uses a different actuation distribution and reports unavailable (-3). Missing,
nonfinite or out-of-range service-brake measurements report invalid state (-6).
Brake remains driver/external owned; filter-off manual input remains passthrough.

### GTState diagnostics

A fixed set of custom scalars is registered. Existing `target_steer`, `target_torque`
and `target_pi_torque` report final physical targets and nominal PI contribution.
No additional stream is created.

| Field | Meaning / unit |
|---|---|
| `rf_status` | Native 0 OK, 1 bounded infeasible, 2..5 error; adapter codes below. |
| `rf_intervened` | 1 only when active execution changed a physical target. |
| `rf_score` | Dimensionless roll score; meaningful on native status 0/1. |
| `rf_margin` | Selected/emitted affine-row margin; meaningful on native status 0/1. |
| `rf_bias` | Estimated acceleration disturbance [rad/s²], even with application off. |
| `rf_nominal_rate` | Free measured continuation or owned/external target-matching rate before QP [rad/s]. |
| `rf_nominal_torque` | Total nominal torque including PI [Nm]. |
| `rf_filtered_rate` | QP candidate steering rate [rad/s], also in shadow. |
| `rf_filtered_torque` | QP candidate absolute torque [Nm], also in shadow. |
| `rf_torque_slew_excess` | Minimum unavoidable slew excess [Nm] when physical rails take priority; candidate bounds, also evaluated in shadow. |
| `rf_observer_status` | Native observer status; repeated state samples skip updates. |
| `rf_brake_measured` | Applied normalized service brake associated with GTState. |
| `rf_brake` | Normalized service brake used by the command row and recovery. |
| `rf_brake_h_jump` | Same-state h change from measured to commanded brake, with the same bias. |

Additional fields: `rf_steering_override`, `rf_torque_override`, `rf_manual`,
`rf_branch_override`, `rf_handback_pending`, `rf_angle_reentry` are flags;
`rf_measured_rate` [rad/s] and `rf_measured_rate_valid` describe the raw rate;
`rf_nominal_margin` and `rf_maximum_margin` describe the native row;
`rf_command_anchor_offset` [rad] records interface projection;
`rf_alignment_target` [rad] and `rf_alignment_cost` [rad²] apply when recovery is
selected. Actual-emitted commands remain the source for intervention accounting.
Shared control-state measured-rate fields and smoothing settings are documented
in [README.md](README.md#transport-and-diagnostics). GTState `torque_est_valid` identifies an available finite map estimate.

Adapter statuses: -1 context/projection unavailable, -2 execution interval invalid,
-3 unsupported command path, -4 invalid physical/steering bounds, -5 stale state, -6 missing/invalid
measurement, -7 idle/off, -8 low-speed nominal passthrough, -9 measured steering rate unavailable,
-10 alignment failure, -11 command-map capability invalid, -12 output pair unrepresentable. Unavailable diagnostics are zeroed; always interpret values
with status. Nonfinite upstream physical commands are rejected before execution;
invalid native results retain the finite nominal command with an explicit error.

With `roll_filter.torque_correction: false`, the QP torque interval collapses to
current nominal rear torque (physical-rail clipped for external commands). Only
steering can be corrected; filter-side `torque_rate_max` is ignored. Manual pedal
input remains released even during recovery.

For a steering-only experiment, `infeasible_steering_penalty: rho` retains command
cost if the hard row is infeasible: minimize `((r-r_nom)/rate_scale)^2 +
rho*(slack/(alpha*C))^2` over the current rate interval. With recovery disabled, this changes only the
infeasible branch; status remains 1 and the logged margin describes the applied
candidate. It provides no recovery guarantee. Omit the setting for max-margin.

## Execution plans and Cartesian TVLQR

The manager routes `XLP1` (nominal) and `XLG1` (nominal plus gains) to
`controller_llc.onPlanPacket(bytes, feedback)`. `execution_plan.decode` validates
the complete datagram without side effects; the LLC owns nominal installation, including when `feedback: none`. The TVLQR plugin
receives the same decoded plan through `setPlan`; it has no separate decoder.
The complete versioned header/layout is documented in the companion
`drive_trajopt/plant_io/README.md` under execution-plan transport.

Protocol version 2 nominal slots contain station, relative time, Cartesian
chassis14, targets3 and desired inputs3. Actuator/desired masks distinguish unused
slots from active zeros in steering/torque/brake order. Gain-bearing plans append
13 chassis-error columns per active actuator. Five steering/torque gain nodes use
1016 bytes; four three-actuator gain nodes use 1032 bytes. The ceiling is 1200 bytes.
Old `XLF1` and version 1 execution packets are rejected.

`calibration.execution_plan` sets `frame_id`, `epoch_id` and
`max_source_age_s` (defaults 1, 1, 1.0). Full datagrams are checked before reference
replacement: version, flags, field masks, length, finite values, monotonic s/time,
unit quaternions, frame/epoch, sequence and original source age. Malformed packets
log an error and preserve the previous plan until its source/coverage expires.
Exact UDP duplicates neither reinstall the reference nor renew its age. Source
time cannot regress. After authority is released, a strictly newer source can
start a new sender counter; active control still requires increasing sequence.
Release
packets clear authority. Gain packets require the configured TVLQR consumer.

Configure `feedback: tvlqr` and matching `feedback_config` frame/epoch, with optional
source/state age limits. The plugin uses the LLC's closest-XY current segment and
fraction, shortest-arc quaternion SLERP and `Log(Rref.T R)` error. Chassis error is
world position3, body-reference rotation3, body FLU velocity3, p/q/r and rear
peripheral speed. Feedback computes `target = previewed_nominal - K @ error`;
steering/torque/brake correction units are rad, Nm and normalized pedal.
Brake rows are present only when enabled by the active-control mask; corrected
brake is clipped to [0,1] and continues through the LLC brake coordination. There are no actuator-error
coordinates or rate integrators in this path. Wheel-speed preview and PI remain
independently configured. Feedback clips to live actuator limits, independently
of trajectory planning bounds.

Autonomous commands pass through steering/wheel-speed PI, then physical target
slew limiting, then the optional rollover filter. Configure independent limits in
`calibration.autonomous_rate_limits`: required `steering_rate_max` [rad/s] and
optional `torque_rate_max` [Nm/s], both finite and positive. For example:

```yaml
calibration:
  autonomous_rate_limits: {steering_rate_max: 0.9, torque_rate_max: 8500}
```

These limits work without a rollover model. If omitted, a configured roll filter
supplies its resolved rate limits, including in off/shadow/unavailable modes.
An omitted torque rate means no autonomous torque slew cap. Without either
configuration this stage is bypassed. Manual inputs bypass it in all cases.

The allowed change is rate times actual elapsed LLC time, not preview. Measurements
seed the first autonomous command; subsequent ticks anchor to the final physical
pair issued after the filter. New packets preserve history. Release, bypass,
manual takeover and reset clear it. A speed/steering integral update is held if it
pushes into its slew bound. No additional physical rails are applied here; the
existing actuator/filter paths retain them. The filter retains its own feasible
rate interval for safety corrections.

`calibration.trajectory_execution` remains retired. Optimizer rates are not needed
on the wire. The existing custom-field channel reports `llc_slew_steer_rad`,
`llc_slew_torque`, `llc_slew_dt`, `llc_slew_steer_limited` and
`llc_slew_torque_limited`. `llc_nominal_total_*` remains the raw post-PI target;
`llc_filter_delta_*` measures only the subsequent safety-filter change.

`transform(ctx)==false` releases external control for stale state/source or
exhausted coverage. Missing/nonfinite required measurements and lost reference/gain
pairing raise. Nominal-only plans pass through even when TVLQR is loaded. Status
exports sequence, station, source age, steering/torque correction and live-rail
saturation. PI remains governed by fixed `kp`, `ki`, `pi_enable`
and the existing combined correction bounds; no speed-based P switch is added.

The existing GT custom-field channel now includes `llc_plan_steer_rad`,
`llc_plan_torque`, `llc_plan_steer_gap`, `llc_plan_torque_gap`,
`llc_nominal_total_steer`, `llc_nominal_total_torque`,
`llc_filter_delta_steer`, `llc_filter_delta_torque`, `llc_effective_kp`. Existing target fields retain their meanings. The
environment status additionally reports last projection status/residual/iterations
and `extrapolated_preparations`; that counter counts calls, not unique states.

TVLQR `feedback_config` contains receiver settings, not matrix K. Partial UDP tunes
may change positive `max_source_age_s` and `max_state_age_s` atomically. Frame/epoch
changes require restart. Invalid tunes log an error and preserve existing settings.
K is delivered only in XLG1 plans.

## Target selection, brake coordination and tuning

`calibration.trajectory_pick` selects `next` (segment end), `closest` (segment
start), `lerp` (projected fraction), or `preview`. Projection uses the nearest XY
segment of the received predicted CoG polyline. For `preview`, specify exactly
one clock with nonnegative per-channel offsets:

```yaml
calibration:
  trajectory_pick: preview
  trajectory_preview:
    time_s: {steering: 0.05, torque: 0.05, wheel_speed: 0.05}
  # Alternative: distance_m: {steering: 0.5, torque: 0.5, wheel_speed: 0.5}
```

Time preview advances from the plan clock interpolated at the projection; it
adds neither solve age nor time since receipt. Distance preview advances along
the 3D polyline. Brake uses the torque query. Omitted channels have zero preview;
zero preview is interpolation at the current projected phase. Endpoint queries
clamp; TVLQR releases on exhausted coverage. Neither clamping nor retransmission
extends the original source lifetime.

`llc_segment`/`llc_fraction` identify the selected phase. `llc_phase_time` is
vehicle seconds; `llc_phase_distance` is polyline distance. `llc_preview_kind`
is 0 without preview, 1 seconds, 2 metres; `llc_query_*` follows that unit.
`llc_plan_age` is vehicle time minus the installed packet timestamp.
`llc_trajectory_id` increments on nominal installation, not on duplicate receipt.

`calibration.brake_coordination` preserves the selected pedal and caps positive
drive torque so `brake * torque <= overlap_torque_nm`. Its low-speed hysteresis
can suppress autonomous service brake to avoid arcade reverse; manual brake is
unchanged. `pedal_deadband` explicitly zeros tiny autonomous requests. The overlap
cap is applied to post-PI torque and again after throttle conditioning. Logs expose
`llc_brake_limited`, `llc_brake_rear_torque`, and `llc_brake_drive_excess`.

`llc_config.resolve(params, current, mode)` validates without mutating settings.
`controller_llc.calibrate` then commits once. Unknown keys, invalid finite values,
inverted bounds, invalid preview or active plan-ID changes log an error and return
false; the previous calibration/command history remains intact. Changing the
steering map while active or while a roll adapter exists requires recreation.
Manual ramp settings are also validated before a combined LLC/manual tune commits.
Roll-filter and feedback namespaces have independent commits; a multi-plugin
`tune` message is not globally atomic.

## Lua execution conventions

Module imports, math bindings, FFI setup and buffer allocation belong at module
load/startup. Configuration validation and plan decoding run only on tune/receipt.
Physics paths reuse private work buffers and borrowed context; do not retain or
modify another module's borrowed tables. Public status snapshots and returned
setpoints have their stated ownership. Keep units and manual/autonomous authority
explicit at the function boundary. Missing capabilities fail or report a status;
there is no synthetic zero measurement or guessed steering calibration.
