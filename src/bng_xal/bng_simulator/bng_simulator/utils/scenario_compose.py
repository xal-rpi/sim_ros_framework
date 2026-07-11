"""Compose scenario YAML from catalog, defaults, levels, and run files.

Pipeline (read top → bottom in this file)
-----------------------------------------
::

    compose_scenario(path, launch_overrides?)
        │
        ├─ resolve_config_path + load_yaml(run)
        ├─ _apply_user_preset       ← optional preset YAML overlay
        ├─ _apply_launch_overrides   ← dict patch; shorthand runs only
        └─ _compose_from_run
               ├─ profile attach  → _compose_attach_run
               └─ profile create  → _compose_create_run  (default)

CREATE profile (_compose_create_run)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
::

    compose.level ──► levels/<level>.yaml     (spawn presets, extra_objects)
    compose.sim   ──► defaults/sim.yaml       (optional fragment)
    compose.vehicles[] OR shorthand vehicle/spawn/vehicle_id
         │
         └─► per vehicle: _build_create_vehicle
                ├─ vehicle_catalog.yaml      (model, yaw_offset, torque_map, io)
                ├─ defaults/llc.yaml         (sensors, LLC stack)
                ├─ _resolve_spawn + _apply_spawn_adjustments
                └─ defaults/udp_io.yaml      (port_stride; ports injected at launch)

    Then: ros_poll_config per vehicle, _apply_run_overlays

ATTACH profile (_compose_attach_run)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
::

    compose.vehicles[] OR shorthand vehicle_id
         │
         └─► per id: defaults/attach_sensors.yaml (no spawn, no LLC)
    ros_poll_config (publish_all), _apply_run_overlays

Run file knobs (see config/runs/compose_reference.yaml)
-------------------------------------------------------
- ``compose`` — profile, level, sim, vehicles[] or shorthand
- ``overrides`` — deep-merge after compose (beamng, extra_objects, …)
- ``overrides.ros_poll`` — per-vehicle or flat (single-vehicle only)
- Launch dict — vehicle, vehicle_id, level, spawn, yaw, pos, preset (shorthand only)
- User preset — optional YAML overlay (config/presets/user/ or ~/.config/bng_bringup/presets/)

Shared between CREATE and ATTACH
--------------------------------
Both profiles: load sim fragment, build ``ros_poll_config`` per vehicle id,
set ``ros_poll_sensor_defaults``, then ``_apply_run_overlays``. CREATE adds
level/scenario/vehicles with LLC; ATTACH grafts minimal sensors onto existing ids.
"""

from __future__ import annotations

import os
from copy import deepcopy
from typing import Any, Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory

from bng_simulator.utils.io_dict_utils import load_yaml


# ---------------------------------------------------------------------------
# Path resolution & YAML loading
# ---------------------------------------------------------------------------

def bringup_config_dir() -> str:
    return os.path.join(get_package_share_directory("bng_bringup"), "config")


def resolve_config_path(config_name: str) -> str:
    """Resolve a run/scenario filename to an absolute path under bng_bringup/config."""
    if os.path.isabs(config_name) and os.path.isfile(config_name):
        return config_name

    base = bringup_config_dir()
    candidates = [
        os.path.join(base, config_name),
        os.path.join(base, "runs", config_name),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path

    raise FileNotFoundError(
        f"Config '{config_name}' not found. Tried: {', '.join(candidates)}"
    )


def user_preset_search_dirs() -> List[str]:
    """Directories searched for named user presets (after absolute/~ paths)."""
    base = bringup_config_dir()
    return [
        os.path.join(base, "presets", "user"),
        os.path.expanduser("~/.config/bng_bringup/presets"),
    ]


def resolve_preset_path(preset_name: str) -> str:
    """Resolve a user preset filename to an absolute path."""
    expanded = os.path.expanduser(preset_name)
    if os.path.isabs(expanded) and os.path.isfile(expanded):
        return expanded
    if os.path.isfile(expanded):
        return os.path.abspath(expanded)

    names = [preset_name]
    if not preset_name.endswith((".yaml", ".yml")):
        names.extend([f"{preset_name}.yaml", f"{preset_name}.yml"])

    candidates: List[str] = []
    for directory in user_preset_search_dirs():
        for name in names:
            candidates.append(os.path.join(directory, name))

    for path in candidates:
        if os.path.isfile(path):
            return path

    raise FileNotFoundError(
        f"User preset '{preset_name}' not found. Tried: {', '.join(candidates)}"
    )


def deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge overlay into base (overlay wins on conflicts)."""
    result = deepcopy(base)
    for key, value in overlay.items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(value, dict)
        ):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _load_fragment(relative_path: str) -> Dict[str, Any]:
    path = os.path.join(bringup_config_dir(), relative_path)
    data = load_yaml(path)
    return data if isinstance(data, dict) else {}


# ---------------------------------------------------------------------------
# Spawn: level preset → xlab/BeamNG yaw, optional pos / pos_offset
# ---------------------------------------------------------------------------

def _resolve_spawn(
    level_data: Dict[str, Any],
    spawn_name: str,
    yaw_offset_deg: float,
    yaw_override: Optional[float],
) -> Dict[str, Any]:
    spawns = level_data.get("spawns", {})
    if spawn_name not in spawns:
        known = ", ".join(sorted(spawns.keys())) or "(empty)"
        raise KeyError(
            f"Unknown spawn preset '{spawn_name}' for level. Known: {known}"
        )

    spawn = deepcopy(spawns[spawn_name])
    xlab_yaw = yaw_override if yaw_override is not None else spawn.get("yaw_angle", 0)
    spawn["xlab_yaw_deg"] = xlab_yaw
    # utv: gtState yaw ≈ beamng_spawn_yaw + offset → xlab 0 needs spawn = xlab - offset
    spawn["yaw_angle"] = xlab_yaw - yaw_offset_deg
    return spawn


def _apply_spawn_adjustments(spawn: Dict[str, Any], veh_spec: Dict[str, Any]) -> None:
    """Apply per-vehicle spawn tweaks on top of a level preset."""
    if "pos" in veh_spec:
        pos = veh_spec["pos"]
        if not isinstance(pos, (list, tuple)) or len(pos) != 3:
            raise ValueError("spawn pos must be [x, y, z]")
        spawn["pos"] = [float(v) for v in pos]
    elif "pos_offset" in veh_spec:
        offset = veh_spec["pos_offset"]
        if not isinstance(offset, (list, tuple)) or len(offset) != 3:
            raise ValueError("pos_offset must be [dx, dy, dz]")
        base = spawn.get("pos", [0, 0, 0])
        spawn["pos"] = [
            float(base[i]) + float(offset[i]) for i in range(3)
        ]

    if "cling" in veh_spec:
        spawn["cling"] = bool(veh_spec["cling"])


# ---------------------------------------------------------------------------
# Catalog → LLC calibration (steering_to_input, torque_map, yaw frame)
# ---------------------------------------------------------------------------

def _catalog_entry(
    catalog: Dict[str, Any],
    vehicle_key: str,
) -> Dict[str, Any]:
    entries = catalog.get("catalog", {})
    if vehicle_key not in entries:
        known = ", ".join(sorted(entries.keys())) or "(empty)"
        raise KeyError(f"Unknown vehicle catalog id '{vehicle_key}'. Known: {known}")
    defaults = catalog.get("defaults", {})
    entry = deepcopy(entries[vehicle_key])
    if "yaw_offset_deg" not in entry and "yaw_offset_deg" in defaults:
        entry["yaw_offset_deg"] = defaults["yaw_offset_deg"]
    return entry


def _apply_torque_map_rules(
    vehicle_cfg: Dict[str, Any],
    torque_map: Optional[str],
) -> None:
    """Inject or strip torque_map on LLC + gtstate depending on catalog entry."""
    llc = vehicle_cfg.setdefault("controllers", {}).setdefault("LowLevelController", {})
    calibration = llc.setdefault("calibration", {})

    if torque_map:
        calibration["torque_map"] = torque_map
        gtstate = vehicle_cfg.setdefault("sensors", {}).setdefault("gtstate", {})
        gtstate["torque_map"] = {
            "field_name": "rear_wheel_torque_est",
        }
    else:
        calibration.pop("torque_map", None)
        sensors = vehicle_cfg.get("sensors", {})
        if "gtstate" in sensors:
            sensors["gtstate"].pop("torque_map", None)


def _apply_catalog_to_vehicle(
    vehicle_cfg: Dict[str, Any],
    catalog_entry: Dict[str, Any],
) -> None:
    steering_to_input = catalog_entry.get("steering_to_input")
    if steering_to_input is not None:
        llc = vehicle_cfg.setdefault("controllers", {}).setdefault(
            "LowLevelController", {}
        )
        gains = llc.setdefault("calibration", {}).setdefault("gains", {})
        gains["steering_to_input"] = steering_to_input

    _apply_torque_map_rules(vehicle_cfg, catalog_entry.get("torque_map"))

    vehicle_cfg["frame"] = {
        "yaw_offset_deg": catalog_entry.get("yaw_offset_deg", 0),
    }


# ---------------------------------------------------------------------------
# Fragment loaders & run-level overlays
# ---------------------------------------------------------------------------

def _load_sim_defaults(compose: Dict[str, Any]) -> Dict[str, Any]:
    """Load sim fragment (defaults/sim.yaml unless compose.sim overrides)."""
    sim_fragment = compose.get("sim", "defaults/sim.yaml")
    return deepcopy(_load_fragment(sim_fragment))


def _ros_poll_overrides_for_vehicle(
    run_data: Dict[str, Any],
    vehicle_id: str,
    vehicle_ids: List[str],
    poll_defaults: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    overrides = run_data.get("overrides")
    if not isinstance(overrides, dict):
        return None
    ros_poll = overrides.get("ros_poll")
    if not isinstance(ros_poll, dict) or not ros_poll:
        return None

    if vehicle_id in ros_poll:
        per_vehicle = ros_poll[vehicle_id]
        return per_vehicle if isinstance(per_vehicle, dict) else None

    sensor_names = set(poll_defaults.keys())
    if set(ros_poll.keys()).issubset(sensor_names):
        if len(vehicle_ids) == 1:
            return ros_poll
        raise ValueError(
            "overrides.ros_poll uses flat sensor keys but multiple vehicles are "
            f"defined — use overrides.ros_poll.<vehicle_id>.<sensor> "
            f"(vehicles: {', '.join(vehicle_ids)})"
        )
    return None


def _apply_run_overlays(config: Dict[str, Any], run_data: Dict[str, Any]) -> Dict[str, Any]:
    """Merge run-level keys and overrides.* onto the composed config."""
    for key in ("scenario_mode", "attach_fallback", "beamng"):
        if key in run_data:
            config[key] = deepcopy(run_data[key])

    overrides = run_data.get("overrides")
    if not isinstance(overrides, dict):
        return config

    overlay = {k: v for k, v in overrides.items() if k != "ros_poll"}
    if overlay:
        config = deep_merge(config, overlay)
    return config


def _load_udp_io_defaults() -> Dict[str, Any]:
    return deepcopy(_load_fragment("defaults/udp_io.yaml"))


def _attach_vehicle_io_slot(vehicle_cfg: Dict[str, Any], port_index: int = 0) -> None:
    """Record per-vehicle UDP port slot (see defaults/udp_io.yaml)."""
    vehicle_cfg["io"] = {"port_index": int(port_index)}


def _merge_level_extras(config: Dict[str, Any], level_data: Dict[str, Any]) -> None:
    if level_data.get("extra_objects"):
        config["extra_objects"] = deepcopy(level_data["extra_objects"])
    else:
        config.setdefault("extra_objects", {})


def _load_poll_defaults() -> Dict[str, Any]:
    fragment = _load_fragment("defaults/ros_poll.yaml")
    return deepcopy(fragment.get("sensor_defaults", {}))


def _build_ros_poll_config(
    vehicle_id: str,
    sensors: Dict[str, Any],
    poll_defaults: Dict[str, Any],
    *,
    publish_all: bool = False,
    per_sensor_overrides: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Build ros_poll_config for one vehicle from actual sensor keys.

    Only sensors present in ``sensors`` AND ``poll_defaults`` get a poll timer.
    """
    veh_poll: Dict[str, Any] = {}
    for sensor_name in sensors:
        if sensor_name not in poll_defaults:
            continue
        entry = deepcopy(poll_defaults[sensor_name])
        if per_sensor_overrides and sensor_name in per_sensor_overrides:
            entry = deep_merge(entry, per_sensor_overrides[sensor_name])
        if publish_all:
            entry["publish"] = 1
        veh_poll[sensor_name] = entry
    return {vehicle_id: veh_poll}


# ---------------------------------------------------------------------------
# Launch overrides (single-vehicle shorthand) & vehicle spec normalization
# ---------------------------------------------------------------------------

LAUNCH_SCALAR_KEYS = ("vehicle", "vehicle_id", "level", "spawn", "yaw")
PRESET_COMPOSE_KEYS = (
    "vehicle",
    "vehicle_id",
    "level",
    "spawn",
    "yaw",
    "pos",
    "pos_offset",
    "cling",
    "port_index",
    "sim",
    "profile",
)


def _parse_pos_launch_value(text: str) -> List[float]:
    """Parse launch ``pos:="x,y,z"`` into a 3-vector."""
    parts = [piece.strip() for piece in str(text).split(",")]
    if len(parts) != 3:
        raise ValueError(f"launch pos must be 'x,y,z' — got '{text}'")
    try:
        return [float(part) for part in parts]
    except ValueError as exc:
        raise ValueError(f"launch pos must be numeric 'x,y,z' — got '{text}'") from exc


def launch_overrides_from_ros(node) -> Dict[str, Any]:
    """Read single-vehicle launch overrides from declared ROS parameters."""
    overrides: Dict[str, Any] = {}

    if node.has_parameter("preset"):
        val = node.get_parameter("preset").value
        if val is not None:
            text = str(val).strip()
            if text:
                overrides["preset"] = text

    if node.has_parameter("pos"):
        val = node.get_parameter("pos").value
        if val is not None:
            text = str(val).strip()
            if text:
                overrides["pos"] = _parse_pos_launch_value(text)

    for key in LAUNCH_SCALAR_KEYS:
        if not node.has_parameter(key):
            continue
        val = node.get_parameter(key).value
        if val is None:
            continue
        text = str(val).strip()
        if not text:
            continue
        overrides[key] = float(text) if key == "yaw" else text
    return overrides


def _apply_user_preset(run_data: Dict[str, Any], preset_name: str) -> None:
    """Merge a user preset YAML overlay onto a shorthand run file."""
    compose = run_data.setdefault("compose", {})
    if "vehicles" in compose:
        raise ValueError(
            "user preset applies only to single-vehicle shorthand runs — "
            "edit compose.vehicles in the run YAML"
        )

    path = resolve_preset_path(preset_name)
    preset = load_yaml(path)
    if not isinstance(preset, dict):
        raise ValueError(f"Invalid user preset at {path}: expected mapping")

    if "compose" in preset:
        nested = preset["compose"]
        if not isinstance(nested, dict):
            raise ValueError(f"preset compose section at {path} must be a mapping")
        if "vehicles" in nested:
            raise ValueError("user preset cannot define compose.vehicles")
        run_data["compose"] = deep_merge(compose, nested)
    else:
        shorthand = {
            key: deepcopy(value)
            for key, value in preset.items()
            if key in PRESET_COMPOSE_KEYS
        }
        for key, value in shorthand.items():
            compose[key] = value

    if "overrides" in preset:
        if not isinstance(preset["overrides"], dict):
            raise ValueError(f"preset overrides at {path} must be a mapping")
        run_data["overrides"] = deep_merge(
            run_data.get("overrides", {}),
            preset["overrides"],
        )

    for key in ("scenario_mode", "attach_fallback"):
        if key in preset:
            run_data[key] = deepcopy(preset[key])


def _apply_launch_overrides(
    compose: Dict[str, Any],
    overrides: Optional[Dict[str, Any]],
) -> None:
    """Patch compose shorthand in-place (single-vehicle runs only)."""
    if not overrides:
        return

    active = {
        k: v
        for k, v in overrides.items()
        if k in LAUNCH_SCALAR_KEYS or k == "pos"
        if v not in (None, "")
    }
    if not active:
        return

    if "vehicles" in compose:
        raise ValueError(
            "launch overrides (vehicle, level, spawn, ...) apply only to "
            "single-vehicle shorthand runs — edit compose.vehicles in the run YAML"
        )

    for key, val in active.items():
        compose[key] = val


def _vehicle_specs_from_compose(compose: Dict[str, Any], profile: str) -> List[Dict[str, Any]]:
    """Normalize compose.vehicles list or single-vehicle shorthand."""
    if "vehicles" in compose:
        specs = deepcopy(compose["vehicles"])
        if not isinstance(specs, list) or not specs:
            raise ValueError("compose.vehicles must be a non-empty list")
        for index, spec in enumerate(specs):
            if not isinstance(spec, dict):
                raise ValueError(f"compose.vehicles[{index}] must be a mapping")
            if "id" not in spec:
                raise ValueError(f"compose.vehicles[{index}] missing required 'id'")
        return specs

    if profile == "attach":
        vehicle_id = compose.get("vehicle_id", "thePlayer")
        return [{"id": vehicle_id}]

    vehicle_key = compose.get("vehicle")
    spawn_key = compose.get("spawn")
    if not vehicle_key or not spawn_key:
        raise ValueError(
            "CREATE compose must define compose.vehicles[] or shorthand "
            "compose.vehicle + compose.spawn (+ compose.level)"
        )
    spec: Dict[str, Any] = {
        "id": compose.get("vehicle_id", "EGO"),
        "vehicle": vehicle_key,
        "spawn": spawn_key,
    }
    if "yaw" in compose:
        spec["yaw"] = compose["yaw"]
    if "port_index" in compose:
        spec["port_index"] = compose["port_index"]
    if "pos_offset" in compose:
        spec["pos_offset"] = compose["pos_offset"]
    if "pos" in compose:
        spec["pos"] = compose["pos"]
    return [spec]


# ---------------------------------------------------------------------------
# One CREATE vehicle: catalog + llc defaults + spawn + io.port_index
# ---------------------------------------------------------------------------

def _build_create_vehicle(
    veh_spec: Dict[str, Any],
    index: int,
    level_data: Dict[str, Any],
    catalog: Dict[str, Any],
    llc_defaults: Dict[str, Any],
) -> Tuple[str, Dict[str, Any]]:
    vehicle_id = str(veh_spec["id"])
    vehicle_key = veh_spec.get("vehicle")
    spawn_key = veh_spec.get("spawn")
    if not vehicle_key or not spawn_key:
        raise ValueError(
            f"vehicle '{vehicle_id}' requires 'vehicle' (catalog id) and 'spawn' (level preset)"
        )

    catalog_entry = _catalog_entry(catalog, str(vehicle_key))
    yaw_offset = float(catalog_entry.get("yaw_offset_deg", 0))
    yaw_override = veh_spec.get("yaw")
    if yaw_override is not None:
        yaw_override = float(yaw_override)

    spawn = _resolve_spawn(level_data, str(spawn_key), yaw_offset, yaw_override)
    _apply_spawn_adjustments(spawn, veh_spec)

    vehicle_cfg = deep_merge(llc_defaults, {})
    vehicle_cfg["model_args"] = {
        "model": catalog_entry["model"],
        "part_config": catalog_entry["part_config"],
        "license": vehicle_id,
    }
    vehicle_cfg["spawn"] = spawn
    _apply_catalog_to_vehicle(vehicle_cfg, catalog_entry)

    port_index = veh_spec.get("port_index")
    if port_index is None:
        port_index = catalog_entry.get("io", {}).get("port_index", index)
    _attach_vehicle_io_slot(vehicle_cfg, int(port_index))

    if catalog_entry.get("sensors"):
        vehicle_cfg["sensors"] = deep_merge(
            vehicle_cfg.get("sensors", {}),
            catalog_entry["sensors"],
        )
    if catalog_entry.get("controllers"):
        vehicle_cfg["controllers"] = deep_merge(
            vehicle_cfg.get("controllers", {}),
            catalog_entry["controllers"],
        )
    if catalog_entry.get("llc_overrides"):
        vehicle_cfg["controllers"] = deep_merge(
            vehicle_cfg.get("controllers", {}),
            catalog_entry["llc_overrides"],
        )
    if veh_spec.get("llc_overrides"):
        vehicle_cfg["controllers"] = deep_merge(
            vehicle_cfg.get("controllers", {}),
            veh_spec["llc_overrides"],
        )

    return vehicle_id, vehicle_cfg


# ---------------------------------------------------------------------------
# Profile assembly (attach vs create share ros_poll + _apply_run_overlays tail)
# ---------------------------------------------------------------------------

def _compose_attach_run(run_data: Dict[str, Any]) -> Dict[str, Any]:
    """Attach: minimal sensors per in-game vehicle id; no spawn/LLC."""
    compose = run_data.get("compose", {})
    vehicle_specs = _vehicle_specs_from_compose(compose, profile="attach")

    sim_defaults = _load_sim_defaults(compose)
    attach_sensors = _load_fragment("defaults/attach_sensors.yaml")
    poll_defaults = _load_poll_defaults()

    vehicles: Dict[str, Any] = {}
    ros_poll_config: Dict[str, Any] = {}
    vehicle_ids = [str(spec["id"]) for spec in vehicle_specs]

    for spec in vehicle_specs:
        vehicle_id = str(spec["id"])
        vehicle_cfg = deepcopy(attach_sensors)
        sensors = vehicle_cfg.get("sensors", {})
        vehicles[vehicle_id] = vehicle_cfg
        ros_poll_config.update(
            _build_ros_poll_config(
                vehicle_id,
                sensors,
                poll_defaults,
                publish_all=True,
                per_sensor_overrides=_ros_poll_overrides_for_vehicle(
                    run_data, vehicle_id, vehicle_ids, poll_defaults
                ),
            )
        )

    config = deep_merge(sim_defaults, {})
    config["vehicles"] = vehicles
    config["scenario_mode"] = run_data.get("scenario_mode", "attach")
    config["ros_poll_config"] = ros_poll_config
    config["ros_poll_sensor_defaults"] = poll_defaults

    return _apply_run_overlays(config, run_data)


def _compose_create_run(run_data: Dict[str, Any]) -> Dict[str, Any]:
    """Create: spawn vehicles from catalog + level presets; full LLC stack."""
    compose = run_data.get("compose", {})
    vehicle_specs = _vehicle_specs_from_compose(compose, profile="create")

    level_key = compose.get("level")
    if not level_key:
        raise ValueError("compose.level is required for CREATE profile")

    catalog = _load_fragment("vehicle_catalog.yaml")
    level_data = _load_fragment(os.path.join("levels", f"{level_key}.yaml"))
    sim_defaults = _load_sim_defaults(compose)
    llc_defaults = _load_fragment("defaults/llc.yaml")
    poll_defaults = _load_poll_defaults()

    vehicles: Dict[str, Any] = {}
    ros_poll_config: Dict[str, Any] = {}
    vehicle_ids = [str(spec["id"]) for spec in vehicle_specs]

    if len(set(vehicle_ids)) != len(vehicle_ids):
        raise ValueError(f"duplicate vehicle ids in compose.vehicles: {vehicle_ids}")

    port_slots: List[int] = []
    for index, spec in enumerate(vehicle_specs):
        vehicle_id, vehicle_cfg = _build_create_vehicle(
            spec, index, level_data, catalog, llc_defaults
        )
        slot = int(vehicle_cfg["io"]["port_index"])
        if slot in port_slots:
            raise ValueError(
                f"duplicate io.port_index {slot} for vehicles "
                f"{vehicle_ids[port_slots.index(slot)]} and {vehicle_id}"
            )
        port_slots.append(slot)
        vehicles[vehicle_id] = vehicle_cfg
        ros_poll_config.update(
            _build_ros_poll_config(
                vehicle_id,
                vehicle_cfg.get("sensors", {}),
                poll_defaults,
                per_sensor_overrides=_ros_poll_overrides_for_vehicle(
                    run_data, vehicle_id, vehicle_ids, poll_defaults
                ),
            )
        )

    scenario = {
        "level": level_data["level"],
        "name": level_data.get("scenario_name", "basic"),
    }

    config = deep_merge(sim_defaults, {})
    config["scenario"] = scenario
    config["vehicles"] = vehicles
    config["udp_io"] = _load_udp_io_defaults()
    _merge_level_extras(config, level_data)
    config["ros_poll_config"] = ros_poll_config
    config["ros_poll_sensor_defaults"] = poll_defaults

    return _apply_run_overlays(config, run_data)


def _compose_from_run(run_data: Dict[str, Any]) -> Dict[str, Any]:
    compose = run_data.get("compose", {})
    if not compose:
        raise ValueError("Run file missing 'compose' section")

    profile = compose.get("profile")
    if profile == "attach":
        return _compose_attach_run(run_data)

    return _compose_create_run(run_data)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def compose_scenario(
    config_path: str,
    launch_overrides: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Load a run file and compose fragments into a full scenario.

    ``launch_overrides`` — optional dict for single-vehicle shorthand only
    (keys: vehicle, vehicle_id, level, spawn, yaw, pos, preset). Ignored when
    the run defines ``compose.vehicles``.

    Merge order: base run → user preset (``preset``) → launch scalar overrides.
    """
    abs_path = resolve_config_path(config_path)
    raw = load_yaml(abs_path)

    if not isinstance(raw, dict):
        raise ValueError(f"Invalid config at {abs_path}: expected mapping")

    if "compose" not in raw:
        raise ValueError(
            f"Config at {abs_path} must have a 'compose' section. "
            "See config/runs/compose_reference.yaml for all options."
        )

    overrides = launch_overrides or {}
    if overrides.get("preset"):
        _apply_user_preset(raw, str(overrides["preset"]))

    compose = raw.setdefault("compose", {})
    _apply_launch_overrides(compose, overrides)
    return _compose_from_run(raw)


def vehicle_yaw_offset_deg(vehicle_cfg: Dict[str, Any]) -> float:
    frame = vehicle_cfg.get("frame", {})
    return float(frame.get("yaw_offset_deg", 0))


def beamng_yaw_from_xlab(xlab_yaw_deg: float, yaw_offset_deg: float) -> float:
    return xlab_yaw_deg - yaw_offset_deg


def summarize_config(
    config: Dict[str, Any],
    *,
    config_path: Optional[str] = None,
    launch_overrides: Optional[Dict[str, Any]] = None,
) -> str:
    """Return a compact multi-line manifest of composed scenario wiring."""
    lines = ["=== xlab scenario manifest ==="]

    if config_path:
        lines.append(f"run:        {config_path}")

    overrides = launch_overrides or {}
    preset = overrides.get("preset")
    lines.append(f"preset:     {preset or '—'}")

    launch_bits = []
    for key in ("level", "spawn", "vehicle", "vehicle_id", "yaw", "pos"):
        if key in overrides and overrides[key] not in (None, ""):
            launch_bits.append(f"{key}={overrides[key]}")
    lines.append(f"overrides:  {', '.join(launch_bits) or '—'}")

    scenario = config.get("scenario", {})
    level = scenario.get("level", "—")
    name = scenario.get("name", "—")
    mode = config.get("scenario_mode", "create")
    lines.append(f"scenario:   level={level} name={name} mode={mode}")

    beamng = config.get("beamng", {})
    if beamng:
        lines.append(
            f"beamng:     host={beamng.get('host', '—')} port={beamng.get('port', '—')}"
        )
    lines.append("")

    udp_io = config.get("udp_io", {})
    stride = int(udp_io.get("port_stride", 10))
    base_listen = int(udp_io.get("control_listen", 0))
    base_send = int(udp_io.get("control_state_send", 0))
    base_sensor = int(udp_io.get("sensor_send", 0))

    vehicles = config.get("vehicles", {})
    ros_poll = config.get("ros_poll_config", {})
    for vehicle_id in sorted(vehicles.keys()):
        vehicle_cfg = vehicles[vehicle_id]
        port_index = int(vehicle_cfg.get("io", {}).get("port_index", 0))
        lines.append(f"vehicle {vehicle_id} (port_index={port_index}):")

        model_args = vehicle_cfg.get("model_args", {})
        model = model_args.get("model", "—")
        part = model_args.get("part_config", "—")
        lines.append(f"  catalog:    model={model} part={part}")

        spawn = vehicle_cfg.get("spawn", {})
        pos = spawn.get("pos", [])
        xlab_yaw = spawn.get("xlab_yaw_deg", spawn.get("yaw_angle", 0))
        beamng_yaw = spawn.get("yaw_angle", 0)
        if isinstance(pos, (list, tuple)) and len(pos) == 3:
            pos_str = f"[{pos[0]}, {pos[1]}, {pos[2]}]"
        else:
            pos_str = "—"
        lines.append(
            f"  spawn:      pos={pos_str} xlab_yaw={xlab_yaw}° beamng_yaw={beamng_yaw}°"
        )

        offset = port_index * stride
        lines.append(
            f"  udp:        listen={base_listen + offset} "
            f"state={base_send + offset} sensor={base_sensor + offset}"
        )

        llc = vehicle_cfg.get("controllers", {}).get("LowLevelController", {})
        calibration = llc.get("calibration", {})
        torque_map = calibration.get("torque_map", "—")
        control_mode = calibration.get("control_mode", "—")
        timeout = calibration.get("command_timeout", "—")
        gains = calibration.get("gains", {})
        steering_to_input = gains.get("steering_to_input", "—")
        steer_pi = "on" if gains.get("steer_pi_enable", 0) else "off"
        llc_verbose = calibration.get("verbose", False)
        lines.append(
            f"  llc:        torque_map={torque_map} control_mode={control_mode} "
            f"timeout={timeout}s verbose={llc_verbose} "
            f"steering_to_input={steering_to_input} steer_pi={steer_pi}"
        )

        ge_sensors = sorted(vehicle_cfg.get("sensors", {}).keys())
        broadcast = llc.get("sensor_broadcast", {})
        udp_streams = [
            f"{stream}@{entry.get('rate', '?')}"
            for stream, entry in sorted(broadcast.items())
        ]
        lines.append(
            f"  sensors:    GE: {', '.join(ge_sensors) or '—'} "
            f"| UDP: {', '.join(udp_streams) or '—'}"
        )

        ros_topics = []
        for sensor_name, poll in sorted(ros_poll.get(vehicle_id, {}).items()):
            if poll.get("publish", 0) > 0:
                topic = poll.get("topic", f"/{vehicle_id}/{sensor_name}")
                ros_topics.append(topic)
        for stream_name, entry in sorted(broadcast.items()):
            sensor_type = entry.get("sensor", stream_name)
            ros_topics.append(
                f"/{vehicle_id}/sensors/{stream_name}/{sensor_type}"
            )
        if llc:
            ros_topics.append(f"/{vehicle_id}/control/cmd")
        lines.append(f"  ros:        {', '.join(ros_topics) or '—'}")
        lines.append("")

    lines.append("=== end manifest ===")
    return "\n".join(lines)
