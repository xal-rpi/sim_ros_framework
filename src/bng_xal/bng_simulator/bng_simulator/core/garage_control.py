"""Plan and supervise opt-in garage experiments through execute_request.

Python owns node selection, plan validation and host artifacts. Vehicle Lua owns
physics commands, mass restoration and the game-side JSONL. The unique output
path identifies a session across start, monitoring, stop and transfer.
"""

from copy import deepcopy
from pathlib import Path
import json
import math
import os
import re
import shutil
import sys
import tempfile
import time
import uuid

import yaml

WEIGHT_TOLERANCE = 1e-6

PLAN_KEYS = {
    "schemaVersion", "config", "referenceNodes", "vehicle", "selectedNodeReference",
    "note", "gtControllerResolution",
}
CONFIG_DEFAULTS = {
    "mode": "static_ballast", "levels": 1, "ballastKg": 10, "forceN": 200,
    "holdSeconds": 8, "maxSpeed": 0.5, "maxRate": 0.5,
    "massReadbackGrace": 0.5, "forceDirection": "down",
    "minLoadFraction": 0.1, "maxLoadAge": 0.05,
}
CONFIG_KEYS = set(CONFIG_DEFAULTS) | {
    "gtController", "cogHeightEstimate", "nodes", "testPlacements", "referenceNodes",
}


def check_keys(value, allowed, label, required=()):
    """Reject malformed mappings and unknown/missing user configuration keys."""
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    unknown, missing = set(value) - allowed, set(required) - set(value)
    if unknown or missing:
        raise ValueError(f"{label}: unknown keys {sorted(unknown, key=str)}; missing keys {sorted(missing)}")


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def node_id(value):
    return finite(value) and value >= 0 and value == math.floor(value)


def coordinates(value):
    return isinstance(value, (list, tuple)) and len(value) == 3 and all(finite(x) for x in value)


def validate_config(config):
    """Validate a host experiment configuration; return explicit, copied defaults.

    Lua repeats validation before touching physics. Host plans cannot choose the
    recording path: run() supplies a unique game-side identity for each attempt.
    """
    check_keys(config, CONFIG_KEYS, "config", ("cogHeightEstimate", "nodes", "testPlacements"))
    cfg = {**CONFIG_DEFAULTS, **deepcopy(config)}
    if cfg["mode"] not in ("static_ballast", "baseline_release"):
        raise ValueError("config.mode must be static_ballast or baseline_release")
    if not node_id(cfg["levels"]) or cfg["levels"] not in (1, 2, 3):
        raise ValueError("levels must be 1, 2 or 3")
    for key, low, high in (
        ("ballastKg", 0, 200), ("forceN", 0, 2000), ("cogHeightEstimate", 0, math.inf),
        ("maxSpeed", 0, math.inf), ("maxRate", 0, math.inf),
    ):
        if not finite(cfg[key]) or not low < cfg[key] <= high:
            raise ValueError(f"{key} must be finite and in ({low}, {high}]")
    for key, low, high in (("holdSeconds", 3, math.inf), ("massReadbackGrace", 0.1, 2)):
        if not finite(cfg[key]) or not low <= cfg[key] <= high:
            raise ValueError(f"{key} must be finite and in [{low}, {high}]")
    if not finite(cfg["minLoadFraction"]) or not 0 < cfg["minLoadFraction"] < 1:
        raise ValueError("minLoadFraction must be in (0,1)")
    if not finite(cfg["maxLoadAge"]) or not 0 < cfg["maxLoadAge"] <= 0.05:
        raise ValueError("maxLoadAge must be in (0,0.05] seconds")
    if cfg["forceDirection"] not in ("up", "down"):
        raise ValueError("forceDirection must be up or down")
    if cfg["mode"] == "static_ballast" and cfg["forceDirection"] != "down":
        raise ValueError("forceDirection applies only to baseline_release")
    if "gtController" in cfg and (not isinstance(cfg["gtController"], str) or not cfg["gtController"]):
        raise ValueError("gtController must be a nonempty name")
    if "referenceNodes" in cfg:
        if not isinstance(cfg["referenceNodes"], dict) or not all(
            node_id(cfg["referenceNodes"].get(key)) for key in ("ref", "back", "up")
        ):
            raise ValueError("referenceNodes requires integer ref/back/up IDs")
    if not isinstance(cfg["nodes"], dict) or not cfg["nodes"]:
        raise ValueError("nodes must contain explicitly selected structural locations")
    used = set()
    for location, entries in cfg["nodes"].items():
        if not isinstance(location, str) or not location or not isinstance(entries, list) or not entries:
            raise ValueError("each named node location requires a nonempty list")
        weights = 0
        for entry in entries:
            if isinstance(entry, dict):
                check_keys(entry, {"cid", "weight"}, "node", ("cid", "weight"))
                cid, weight = entry["cid"], entry["weight"]
            else:
                cid, weight = entry, 1 / len(entries)
            if not node_id(cid) or cid in used or not finite(weight) or weight <= 0:
                raise ValueError("node IDs must be unique nonnegative integers with positive weights")
            used.add(cid)
            weights += weight
        if not math.isclose(weights, 1, abs_tol=WEIGHT_TOLERANCE, rel_tol=0):
            raise ValueError("node weights must sum to one within each location")
    if "testPlacements" in cfg:
        placements = cfg["testPlacements"]
        if not isinstance(placements, list) or not placements:
            raise ValueError("testPlacements must be a nonempty list")
        names = set()
        for placement in placements:
            check_keys(placement, {"name", "weights", "forceDirection"}, "test placement", ("name", "weights"))
            name, weights = placement["name"], placement["weights"]
            if not isinstance(name, str) or not name or name in names:
                raise ValueError("test placement names must be nonempty and unique")
            names.add(name)
            if not isinstance(weights, dict) or not weights:
                raise ValueError("test placement weights must be a nonempty mapping")
            if any(k not in cfg["nodes"] or not finite(w) or w < 0 for k, w in weights.items()):
                raise ValueError("test placement uses an unknown location or invalid weight")
            if not math.isclose(sum(weights.values()), 1, abs_tol=WEIGHT_TOLERANCE, rel_tol=0):
                raise ValueError("test placement weights must sum to one")
            direction = placement.get("forceDirection", cfg["forceDirection"])
            if direction not in ("up", "down") or (cfg["mode"] == "static_ballast" and direction != "down"):
                raise ValueError("up/down forceDirection applies only to baseline_release")
    return cfg


def request(send, action, vehicle=None, **kwargs):
    """Relay one request and reject errors or a response from another session."""
    args = dict(action=action, **kwargs)
    if vehicle:
        args["vehicle_name"] = vehicle
    result = send("garage_experiment", args)
    if not isinstance(result, dict):
        raise TimeoutError(f"garage {action}: simulator service returned no response")
    if result.get("error"):
        raise RuntimeError(result["error"])
    if action in ("status", "stop") and type(result.get("active")) is not bool:
        raise ValueError(f"garage {action}: active must be boolean")
    expected = kwargs.get("expected_output")
    if expected is not None and result.get("output") != expected:
        raise RuntimeError(f"garage {action}: session identity changed; refusing to stop or export another session")
    return result


class GarageServiceClient:
    """Own one ROS node/client, and only shut down a context created here."""

    def __init__(self, timeout=10):
        self.timeout = timeout

    def __enter__(self):
        import rclpy
        from bng_msgs.srv import ExecuteRequest

        self.ros, self.service_type = rclpy, ExecuteRequest
        self.started = not rclpy.ok()
        self.node = None
        try:
            if self.started:
                rclpy.init()
            self.node = rclpy.create_node("garage_control_" + str(os.getpid()))
            self.client = self.node.create_client(ExecuteRequest, "execute_request")
            if not self.client.wait_for_service(timeout_sec=self.timeout):
                raise TimeoutError("execute_request service unavailable; check simulator manager")
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def send(self, name, payload):
        from bng_simulator.utils.io_dict_utils import convert_dict_to_str, convert_str_to_dict

        message = self.service_type.Request()
        message.function_name = name
        message.arguments = convert_dict_to_str(payload)
        future = self.client.call_async(message)
        self.ros.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        if not future.done():
            self.client.remove_pending_request(future)
            raise TimeoutError(f"simulator request timed out: {payload.get('action', name)}")
        response = future.result()
        if response is None:
            raise TimeoutError("simulator returned no response")
        return convert_str_to_dict(response.result)

    def __exit__(self, *unused):
        try:
            if self.node is not None:
                self.node.destroy_node()
        finally:
            if self.started and self.ros.ok():
                self.ros.shutdown()


def select_nodes(inventory, placements):
    """Select unique structural nodes by verified names or chassis-FLU proximity [m]."""
    if not isinstance(placements, dict) or not placements:
        raise ValueError("placements must be a nonempty mapping")
    nodes, selected, used = inventory["nodes"], {}, set()
    for location, spec in placements.items():
        if not isinstance(location, str) or not location:
            raise ValueError("placement names must be nonempty strings")
        if not isinstance(spec, dict):
            raise ValueError(f"{location}: placement must be a mapping")
        named = "node_names" in spec
        allowed = {"node_names", "node_pattern", "verified_frame_origin"} if named else {
            "position_chassis_flu_m", "node_pattern", "radius_m", "count",
        }
        required = ("node_names", "node_pattern") if named else ("position_chassis_flu_m", "node_pattern")
        check_keys(spec, allowed, location, required)
        if not isinstance(spec["node_pattern"], str) or not spec["node_pattern"]:
            raise ValueError(f"{location}: explicit node_pattern required")
        pattern = re.compile(spec["node_pattern"])
        if named:
            names = spec["node_names"]
            if not isinstance(names, list) or not names or not all(isinstance(n, str) and n for n in names):
                raise ValueError(f"{location}: node_names must be a nonempty list of names")
            matching = []
            expected_frame = spec.get("verified_frame_origin")
            if expected_frame is not None and (not isinstance(expected_frame, str) or not expected_frame):
                raise ValueError(f"{location}: verified_frame_origin must be a nonempty name")
            verified = expected_frame is not None and inventory["referenceNodes"].get("partOrigin") == expected_frame
            if expected_frame is not None and not verified:
                raise ValueError(f"{location}: source-verified frame origin does not match")
            for name in names:
                candidates = [n for n in nodes if n.get("name") == name and n["cid"] not in used]
                if len(candidates) != 1:
                    raise ValueError(f"{location}: named structural node {name} missing/ambiguous")
                group = candidates[0].get("group")
                if group and not pattern.search(str(group)):
                    raise ValueError(f"{location}: node {name} has wrong structural group")
                if not group and not verified:
                    raise ValueError(f"{location}: node {name} has no runtime group or verified frame")
                matching.append(candidates[0]["cid"])
                used.add(candidates[0]["cid"])
            selected[location] = matching
            continue
        position = spec["position_chassis_flu_m"]
        if not coordinates(position):
            raise ValueError(f"{location}: three finite placement coordinates required")
        radius, count = spec.get("radius_m", 0.2), spec.get("count", 3)
        if not finite(radius) or radius <= 0 or not node_id(count) or count == 0:
            raise ValueError("radius must be finite/positive and count a positive integer")
        count = int(count)
        candidates = []
        for node in nodes:
            if node["cid"] in used:
                continue
            text = str(node.get("name", "")) + " " + str(node.get("group", ""))
            if pattern.search(text):
                distance = math.dist(position, node["chassisFLU"])
                if distance <= radius:
                    candidates.append((distance, node["cid"]))
        candidates.sort()
        if len(candidates) < count:
            raise ValueError(f"{location}: {len(candidates)} nodes within {radius} m; need {count}")
        selected[location] = [cid for _, cid in candidates[:count]]
        used.update(selected[location])
    return selected


def utv_seat_profile(inventory):
    """Source-verified frame mapping; users must still supply the height estimate."""
    if inventory.get("vehicle", {}).get("model") != "utv":
        raise ValueError("utv_seats_rear preset requires model=utv")
    from bng_simulator.utils.io_dict_utils import load_yaml
    from bng_simulator.utils.scenario_compose import bringup_config_dir

    return load_yaml(str(Path(bringup_config_dir()) / "garage/utv_seats_rear.yaml"))


def make_plan(inventory, profile=None):
    """Build schema-1 YAML data; drafts have a null height and cannot actuate."""
    config = {**CONFIG_DEFAULTS, "cogHeightEstimate": None, "nodes": {}, "testPlacements": []}
    if profile is not None:
        check_keys(profile, {"placements", "experiment"}, "profile", ("placements",))
        experiment = profile.get("experiment", {})
        check_keys(experiment, CONFIG_KEYS - {"nodes", "referenceNodes"}, "experiment")
        config.update(deepcopy(experiment))
        config["nodes"] = select_nodes(inventory, profile["placements"])
    config["gtController"] = resolve_gt_controller(config, inventory)
    selected = {
        entry["cid"] if isinstance(entry, dict) else entry
        for ids in config["nodes"].values() for entry in ids
    }
    return {
        "schemaVersion": 1, "config": config,
        "referenceNodes": deepcopy(inventory["referenceNodes"]),
        "vehicle": inventory.get("vehicle"),
        "selectedNodeReference": [n for n in inventory["nodes"] if n["cid"] in selected],
        "note": "Chassis-FLU coordinates, not CoG. Supply a height estimate and explicit testPlacements before running.",
    }


def validate_plan(plan, inventory):
    """Reject unsupported schemas, settings and changed node geometry before start."""
    check_keys(plan, PLAN_KEYS, "plan", ("schemaVersion", "config", "referenceNodes", "selectedNodeReference", "vehicle"))
    if type(plan["schemaVersion"]) is not int or plan["schemaVersion"] != 1:
        raise ValueError("unsupported garage plan schemaVersion; expected 1")
    cfg = validate_config(plan["config"])
    if not isinstance(plan["vehicle"], dict):
        raise ValueError("plan.vehicle must contain the inspected vehicle identity")
    if plan["vehicle"] != inventory.get("vehicle"):
        raise ValueError("vehicle/config differs from plan; regenerate mapping")
    if not isinstance(plan["referenceNodes"], dict) or not all(
        node_id(plan["referenceNodes"].get(key)) for key in ("ref", "back", "up")
    ):
        raise ValueError("referenceNodes requires integer ref/back/up IDs")
    if plan["referenceNodes"] != inventory["referenceNodes"]:
        raise ValueError("reference triangle differs from plan")
    if "referenceNodes" in cfg and cfg["referenceNodes"] != plan["referenceNodes"]:
        raise ValueError("config.referenceNodes differs from the verified plan frame")
    refs = plan["selectedNodeReference"]
    if not isinstance(refs, list):
        raise ValueError("selectedNodeReference must be a list")
    actual = {n["cid"]: n for n in inventory["nodes"]}
    reference = {}
    for node in refs:
        if not isinstance(node, dict) or not node_id(node.get("cid")) or not coordinates(node.get("chassisFLU")):
            raise ValueError("selectedNodeReference requires integer IDs and finite chassis-FLU coordinates")
        if node["cid"] in reference:
            raise ValueError("duplicate selectedNodeReference ID")
        reference[node["cid"]] = node
    for ids in cfg["nodes"].values():
        for entry in ids:
            cid = entry["cid"] if isinstance(entry, dict) else entry
            if cid not in actual or cid not in reference:
                raise ValueError("selected node missing from inventory/reference; regenerate plan")
            old, new = reference[cid], actual[cid]
            if not coordinates(new.get("chassisFLU")):
                raise ValueError(f"node {cid} has invalid live coordinates")
            if old.get("name") != new.get("name") or math.dist(old["chassisFLU"], new["chassisFLU"]) > 0.05:
                raise ValueError(f"node {cid} geometry/name changed; regenerate plan")
    return cfg


def resolve_gt_controller(config, inventory):
    """Use the named contact-enabled sensor, or the sole current instance."""
    active = [c["name"] for c in inventory["gtControllers"] if c.get("contactEnabled")]
    preferred = config.get("gtController")
    if preferred in active:
        return preferred
    if len(active) == 1:
        return active[0]
    if not active:
        raise ValueError("no contact-enabled GT sensor; enable it and launch the simulator")
    raise ValueError("select one current gtController from: " + str(active))


def copy_local_recording(status, destination):
    """Use only explicit BEAMNG_USER_PATH; an unset path selects service transfer."""
    if status.get("active"):
        raise ValueError("cannot copy an active recording")
    if destination.exists():
        raise FileExistsError(destination)
    root = os.environ.get("BEAMNG_USER_PATH")
    if not root:
        return False
    game_path = status.get("output", "")
    if not re.fullmatch(r"/settings/support_garage_[a-zA-Z0-9_]+\.jsonl", game_path):
        raise ValueError("unexpected garage recording path")
    source = Path(root).expanduser() / game_path.lstrip("/")
    with tempfile.NamedTemporaryFile(dir=destination.parent, prefix=destination.name + ".", delete=False) as temp:
        temporary = Path(temp.name)
    try:
        before = source.stat()
        shutil.copyfile(source, temporary)
        after = source.stat()
        if (before.st_size != after.st_size or before.st_mtime_ns != after.st_mtime_ns
                or temporary.stat().st_size != after.st_size):
            raise RuntimeError("game recording changed during copy")
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)
    return True


def download_recording(send, status, destination, vehicle=None):
    """Atomically export a stopped, identified session; partial files stay temporary."""
    if status.get("active") or not status.get("output"):
        raise ValueError("a stopped recording with an output identity is required")
    if destination.exists():
        raise FileExistsError(destination)
    print("[garage] Exporting " + str(destination), file=sys.stderr, flush=True)
    if copy_local_recording(status, destination):
        print("[garage] Direct copy complete", file=sys.stderr, flush=True)
        return
    with tempfile.NamedTemporaryFile(dir=destination.parent, prefix=destination.name + ".", delete=False) as stream:
        temporary = Path(stream.name)
    try:
        with temporary.open("wb") as stream:
            offset = 0
            reported = time.monotonic()
            while True:
                chunk = request(send, "read", vehicle, expected_output=status["output"], offset=offset, limit=262144)
                data, next_offset, eof = chunk.get("data"), chunk.get("nextOffset"), chunk.get("eof")
                if not isinstance(data, str) or not node_id(next_offset) or type(eof) is not bool:
                    raise ValueError("invalid recording chunk")
                encoded = data.encode("utf-8")
                if next_offset != offset + len(encoded) or (not eof and next_offset <= offset):
                    raise ValueError("recording export made no byte progress or returned an inconsistent offset")
                stream.write(encoded)
                now = time.monotonic()
                if now - reported >= 5:
                    print(f"[garage] Downloaded {next_offset / 1048576:.1f} MiB", file=sys.stderr, flush=True)
                    reported = now
                if eof:
                    break
                offset = next_offset
            stream.flush()
        temporary.replace(destination)
        print("[garage] Export complete", file=sys.stderr, flush=True)
    finally:
        temporary.unlink(missing_ok=True)


def run(send, plan, output, vehicle=None, max_wall_seconds=1800, sleep=time.sleep, clock=time.monotonic):
    """Run one session, restore/stop on failure, export partial data, then re-raise.

    finished.json separates experiment failure from cleanup/transfer failures.
    A session replaced by another client is neither stopped nor exported here.
    """
    if not finite(max_wall_seconds) or max_wall_seconds <= 0:
        raise ValueError("max_wall_seconds must be positive")
    output = Path(output).expanduser()
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    inventory = request(send, "inspect", vehicle)
    cfg = validate_plan(plan, inventory)
    if inventory.get("status", {}).get("active"):
        raise RuntimeError("garage session already active")
    cfg["gtController"] = resolve_gt_controller(cfg, inventory)
    resolved = dict(plan, config=deepcopy(cfg), gtControllerResolution={
        "saved": plan["config"].get("gtController"), "current": cfg["gtController"],
    })
    output.mkdir(parents=True)
    (output / "inventory.json").write_text(json.dumps(inventory, indent=2) + "\n")
    (output / "plan.yaml").write_text(yaml.safe_dump(resolved, sort_keys=False))
    cfg["referenceNodes"] = deepcopy(plan["referenceNodes"])
    cfg["output"] = "/settings/support_garage_" + uuid.uuid4().hex + ".jsonl"
    # Persist the identity before start: a lost reply may still mean Lua is running.
    (output / "started.json").write_text(json.dumps({"output": cfg["output"], "attempted": True}, indent=2) + "\n")
    status, failure, cleanup_errors = {}, None, []
    try:
        print("[garage] Starting " + cfg["output"], file=sys.stderr, flush=True)
        info = request(send, "start", vehicle, expected_output=cfg["output"], config=cfg)
        (output / "started.json").write_text(json.dumps(info, indent=2) + "\n")
        deadline, last_phase = clock() + max_wall_seconds, None
        while True:
            status = request(send, "status", vehicle, expected_output=cfg["output"])
            if not status.get("active"):
                break
            if status.get("phase") != last_phase:
                print("[garage] Phase " + str(status.get("phase")), file=sys.stderr, flush=True)
                last_phase = status.get("phase")
            if clock() > deadline:
                raise TimeoutError("garage wall-time limit reached; check paused simulation")
            sleep(0.5)
        if status.get("reason") != "complete" or status.get("recordingErrors") or status.get("restoreErrors"):
            raise RuntimeError(f"garage stopped: {status}")
    except BaseException as exc:
        failure = exc
    finally:
        try:
            status = request(send, "status", vehicle, expected_output=cfg["output"])
            if status.get("active"):
                try:
                    status = request(send, "stop", vehicle, expected_output=cfg["output"])
                except Exception as exc:
                    cleanup_errors.append(str(exc))
                    if failure is None:
                        failure = exc
                    status = request(send, "status", vehicle, expected_output=cfg["output"])
            if status.get("active"):
                raise RuntimeError("garage still active; recording was not exported")
            if status.get("recordingErrors") or status.get("restoreErrors"):
                if failure is None:
                    failure = RuntimeError(f"garage cleanup failed: {status}")
            download_recording(send, status, output / "recording.jsonl", vehicle)
        except Exception as exc:
            cleanup_errors.append(str(exc))
            if failure is None:
                failure = exc
        (output / "finished.json").write_text(json.dumps({
            "status": status, "error": f"{type(failure).__name__}: {failure}" if failure is not None else None,
            "cleanupErrors": cleanup_errors,
        }, indent=2) + "\n")
    if failure is not None:
        raise failure
    return {"output": str(output), "recording": str(output / "recording.jsonl"), "status": status}


def recover(send, output, vehicle=None, stop_active=False):
    """Retrieve the matching session, including explicitly labelled aborted data."""
    output = Path(output).expanduser()
    expected = json.loads((output / "started.json").read_text())["output"]
    status = request(send, "status", vehicle, expected_output=expected)
    if status.get("active"):
        if not stop_active:
            raise ValueError("session still active; use --stop-active to stop and export")
        try:
            status = request(send, "stop", vehicle, expected_output=expected)
        except Exception as exc:
            status = request(send, "status", vehicle, expected_output=expected)
            status["stopRequestError"] = str(exc)
    destination = output / "recording.recovered.jsonl"
    download_recording(send, status, destination, vehicle)
    return {"output": str(destination), "status": status}


def cli(args, send):
    """Dispatch the explicit sim_control garage subcommands."""
    if args.command == "garage-inspect":
        inventory = request(send, "inspect", args.vehicle)
        path = Path(args.output).expanduser()
        if path.exists():
            raise FileExistsError(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(inventory, indent=2) + "\n")
        return {"output": str(path), "nodeCount": len(inventory["nodes"]), "gtControllers": inventory["gtControllers"]}
    if args.command == "garage-plan":
        inventory = request(send, "inspect", args.vehicle)
        if args.placements and args.preset:
            raise ValueError("choose placements YAML or a preset")
        profile = yaml.safe_load(Path(args.placements).expanduser().read_text()) if args.placements else None
        if args.placements and not isinstance(profile, dict):
            raise ValueError("placements profile must be a mapping")
        if args.preset:
            profile = utv_seat_profile(inventory)
        plan = make_plan(inventory, profile)
        path = Path(args.output).expanduser()
        if path.exists() or path.with_suffix(".inventory.json").exists():
            raise FileExistsError(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(plan, sort_keys=False))
        path.with_suffix(".inventory.json").write_text(json.dumps(inventory, indent=2) + "\n")
        return {"output": str(path), "nodes": plan["config"]["nodes"], "next": "Review height estimate and testPlacements, then run with --plan"}
    if args.command == "garage-recover":
        return recover(send, args.output, args.vehicle, args.stop_active)
    if args.command == "garage-run":
        plan = yaml.safe_load(Path(args.plan).expanduser().read_text())
        return run(send, plan, args.output, args.vehicle, args.max_wall_seconds)
    if args.command in ("garage-stop", "garage-status"):
        return request(send, "stop" if args.command == "garage-stop" else "status", args.vehicle)
    raise ValueError("unknown garage command: " + str(args.command))
