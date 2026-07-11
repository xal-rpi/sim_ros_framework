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

Feedback is **not** part of `controller_llc`. Any actuation plugin that implements the
split API below can use the same feedback modules.

## YAML config

```yaml
controllers:
  LowLevelController:
    controllerType: llc
    feedback: passthrough    # omit, null, or "none" → disabled
    feedback_gains:          # plugin-specific; hot-tunable
      example_gain: 1.0
```

Keys are passed through `OpenController` unchanged. `feedback_gains` may also live under
`calibration.feedback_gains`.

## Feedback plugin file

Path: `lua/vehicle/controller/xlab/feedback_<stem>.lua`

Plugins are loaded with plain `require` (no `pcall`). Load errors, `init` failures,
and `transform` runtime errors propagate to the BeamNG terminal — fail fast by design.

| Callback | Required | Purpose |
|----------|----------|---------|
| `init(common, cfg)` | yes | `cfg.gains`, `cfg.stem`; allocate private state |
| `transform(ctx)` | yes | Return effective setpoints for this tick |
| `onTune(data)` | no | Apply `data.feedback_gains` |
| `reset()` | no | Clear internal state |
| `stop()` | no | Teardown |

## `transform(ctx)` input

| Field | Type | Description |
|-------|------|-------------|
| `sim_t` | number | Simulation time [s] |
| `dt` | number | Control period for this step [s] |
| `plant` | table | Actuation controller plant snapshot |
| `resolved` | table | Setpoints after controller projection (this tick) |
| `raw` | table or nil | Full command buffers if controller exposes `getRawTargets()` |

### `resolved` / output setpoint fields

| Field | Unit | Notes |
|-------|------|-------|
| `torque_des` | N·m | Feedforward torque |
| `omega_des` | m/s | Rear wheel speed target |
| `steer_des_rad` | rad | Roadwheel angle |
| `brake_des` | 0..1 | Brake demand |
| `throttle_override` | 0..1 | If set, actuation bypasses policy + speed PI |

Return `nil` or omit a field to leave the resolved value unchanged (plugin-dependent;
`passthrough` returns `resolved` as-is).

## Actuation controller split API (for feedback orchestration)

When `feedback` is configured, `controller_manager` requires:

| Method | Purpose |
|--------|---------|
| `prepareControlStep(dt, common)` | Rate decimation, timeout, gt read; returns step table or nil |
| `resolveSetpoints(plant)` | Trajectory pick or scalar → setpoint table |
| `getRawTargets()` | Optional raw command snapshot |
| `applySetpoints(plant, sp, dt, step)` | PI + policy + vehicle I/O |
| `finishControlStep(common, step)` | Per-tick cleanup |

If `feedback` is set but the actuation plugin lacks the split API above, the manager
**errors** (fail-fast) — there is no fallback to `update()`.

## Tuning

```json
{ "type": "tune", "data": { "feedback_gains": { "k_slip": 0.5 } } }
```

Actuation gains (`gains`, `kp`, `ki`, …) still route to the active controller only.

## Built-in plugins

| Stem | File | Behavior |
|------|------|----------|
| `passthrough` | `feedback_passthrough.lua` | Returns `ctx.resolved` unchanged (pipeline smoke test) |
