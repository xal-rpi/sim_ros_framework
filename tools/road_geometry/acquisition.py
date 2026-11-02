"""Live BeamNG acquisition and camera control.

Owns connections, scene replacement, Lua injection and native measurements.
Offline route selection, validation and publication live in their domain modules.
Only capture/survey with --fresh replace the loaded scene.
"""

import hashlib
import json
from pathlib import Path

import numpy as np
from geometry import HeightIndex, load_export, validate, write_export

ROOT = Path(__file__).resolve().parents[2]
LUA = ROOT / "luamod/lua/ge/extensions/xlab/roadGeometry.lua"


def lua_call(beamng, expression):
    return json.loads(
        beamng.control.queue_lua_command(
            "return jsonEncode(" + expression + ")", response=True
        )
    )


def inject(beamng):
    source = LUA.read_text()
    delimiter = "===="
    if "]" + delimiter + "]" in source:
        raise ValueError("Lua injection delimiter occurs in the source")
    chunk = (
        "xlabRoadGeometry = assert(load(["
        + delimiter
        + "["
        + source
        + "]"
        + delimiter
        + '], "xlab/roadGeometry"))(); return "loaded"'
    )
    beamng.control.queue_lua_command(chunk, response=True)


def connect(args):
    from beamngpy import BeamNGpy

    beamng = BeamNGpy(args.host, args.port, quit_on_close=False)
    beamng.open(launch=False)
    return beamng


def capture(args):
    from beamngpy import Scenario

    if Path(args.output).exists():
        raise FileExistsError(
            "Choose a new output folder; existing exports are immutable"
        )
    if args.samples < 1:
        raise ValueError("--samples must be positive")
    beamng = connect(args)
    try:
        if not args.fresh:
            raise ValueError(
                "This prototype requires --fresh: capture precedes builder cleanup; no duplicate injection into an active scene"
            )
        state = beamng.control.get_gamestate()
        if state["state"] != "menu":
            # A new inspection client has no Python Scenario object for the
            # already-loaded scene; use the standard GE stop request directly.
            beamng._send({"type": "StopScenario"}).ack("ScenarioStopped")
        # A generated test scenario in the user folder; zero vehicles, no LLC,
        # no edits to a vanilla level or the managed asset definitions.
        scene = Scenario("tech_ground", "xlab_geometry_probe")
        scene.make(beamng)
        print("Loading vehicle-free tech_ground test scene", flush=True)
        beamng.scenario.load(
            scene, connect_player_vehicle=False, connect_existing_vehicles=False
        )
        beamng.control.pause()
        inject(beamng)
        probe = lua_call(beamng, "xlabRoadGeometry.probe()")
        if not (not probe["classes"]["BeamNGVehicle"]):
            raise ValueError("Unexpected vehicle in geometry-only scene")
        manifest = json.loads(
            beamng.control.queue_lua_command(
                'return jsonEncode(assert(jsonReadFile("/art/xlab_environments/manifest.json")))',
                response=True,
            )
        )
        entry = manifest["assets"][args.asset]
        path = entry["path"]
        content = beamng.control.queue_lua_command(
            "return assert(readFile(" + json.dumps(path) + "))", response=True
        ).encode()
        digest = hashlib.sha256(content).hexdigest()
        if not (digest == entry["sha256"]):
            raise ValueError("Deployed asset does not match its manifest")
        print("Capturing evaluated mesh for", args.asset, flush=True)
        info = lua_call(beamng, "xlabRoadGeometry.capture(" + json.dumps(path) + ")")
        info.update(
            asset_name=args.asset,
            package=manifest["name"],
            package_version=manifest["version"],
        )
        info["route_defaults"] = entry.get("route_defaults", {})
        info["engine_version"] = lua_call(
            beamng,
            "{version=tostring(beamng_version or 'unknown'),build=tostring(beamng_versionb or 'unknown')}",
        )
        info["simulator_source_sha256"] = {}
        for name in [
            "splineTrack",
            "segmentToProceduralMesh",
            "basicCenters",
            "basicBorders",
        ]:
            source_path = "/lua/ge/extensions/util/trackBuilder/" + name + ".lua"
            text = beamng.control.queue_lua_command(
                "return assert(readFile(" + json.dumps(source_path) + "))",
                response=True,
            )
            info["simulator_source_sha256"][source_path] = hashlib.sha256(
                text.encode()
            ).hexdigest()
        segments = []
        for i in range(1, info["segmentCount"] + 1):
            segments.append(
                lua_call(
                    beamng,
                    "xlabRoadGeometry.segment(" + json.dumps(path) + "," + str(i) + ")",
                )
            )
        geometry, meta = write_export(
            args.output,
            info,
            segments,
            lua_hash=hashlib.sha256(LUA.read_bytes()).hexdigest(),
            asset_hash=digest,
            asset_bytes=content,
        )
        # Uniformly distributed by triangle area, rather than concentrating
        # probes only on dense spline nodes. Both mesh and rays use world ENU.
        tri = geometry["vertices_xyz"][geometry["triangles"]]
        area = (
            np.linalg.norm(
                np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1
            )
            / 2
        )
        candidates = np.flatnonzero(
            (geometry["face_normal"][:, 2] > 0.5) & (area > 1e-10)
        )
        rng = np.random.default_rng(17)
        ids = rng.choice(
            candidates, args.samples, p=area[candidates] / area[candidates].sum()
        )
        uv = rng.random((len(ids), 2))
        flip = uv.sum(axis=1) > 1
        uv[flip] = 1 - uv[flip]
        points = (
            tri[ids, 0]
            + uv[:, :1] * (tri[ids, 1] - tri[ids, 0])
            + uv[:, 1:] * (tri[ids, 2] - tri[ids, 0])
        )
        hits = []
        for start in range(0, len(points), 64):
            encoded = json.dumps(json.dumps(points[start : start + 64].tolist()))
            hits.extend(
                lua_call(
                    beamng, "xlabRoadGeometry.rays(jsonDecode(" + encoded + "),0.3,0.3)"
                )
            )
        heights = np.array([h.get("height", np.nan) for h in hits])
        valid = np.array([h["valid"] for h in hits], bool)
        errors = heights - points[:, 2]
        # Another overlapping piece may be above a sampled triangle. Distinguish
        # that real ambiguity from a mismatch between export and live collision.
        index = HeightIndex(geometry)
        envelope = [index.height(p, above=0.3, below=0.3) for p in points]
        envelope_error = np.array(
            [h[0] - z if h else np.nan for h, z in zip(envelope, heights)]
        )
        normals = np.array([h.get("engineNormal", [np.nan] * 3) for h in hits])
        normal_dot = (normals * geometry["face_normal"][ids]).sum(axis=1)
        angular = np.degrees(np.arccos(np.clip(normal_dot, -1, 1)))
        summary = dict(
            samples=len(hits),
            valid=int(valid.sum()),
            height_rmse_m=float(np.sqrt(np.mean(errors[valid] ** 2))),
            height_abs_p95_m=float(np.percentile(abs(errors[valid]), 95)),
            height_abs_max_m=float(np.max(abs(errors[valid]))),
            engine_normal_available=int(np.isfinite(normals).all(axis=1).sum()),
            normal_angle_median_p95_deg=np.nanpercentile(angular, [50, 95]).tolist(),
            sampled_triangle_mismatch_over_0_1mm=int((abs(errors[valid]) > 1e-4).sum()),
            upper_envelope_height_abs_max_m=float(np.nanmax(abs(envelope_error))),
            settings={
                "above_m": 0.3,
                "below_m": 0.3,
                "seed": 17,
                "sampling": "triangle-area weighted upper-facing candidates",
            },
            caveat="First static collision hit in a short height band; overlaps can differ from the sampled triangle.",
        )
        folder = Path(args.output)
        np.savez_compressed(
            folder / "collision_probes.npz",
            xyz=points,
            triangle_id=ids,
            valid=valid,
            ray_height=heights,
            engine_normal=normals,
        )
        (folder / "collision_validation.json").write_text(
            json.dumps(summary, indent=2) + "\n"
        )
        (folder / "capabilities.json").write_text(json.dumps(probe, indent=2) + "\n")
        print(json.dumps(summary, indent=2), flush=True)
        print(json.dumps(validate(folder), indent=2), flush=True)
        # Frame camera only; never teleport or engage a vehicle.
        center = (
            geometry["vertices_xyz"].min(axis=0) + geometry["vertices_xyz"].max(axis=0)
        ) / 2
        camera_at(
            beamng, center, float(np.ptp(geometry["vertices_xyz"][:, :2], axis=0).max())
        )
        from viewer import write_viewer

        print(write_viewer(folder), flush=True)
    finally:
        beamng.disconnect()


def camera_at(beamng, center, height):
    x, y, z = map(float, center)
    expression = f'''local p=vec3({x},{y-height*.6},{z+height});
local q=quatFromDir(vec3({x},{y},{z})-p,vec3(0,0,1));
commands.setFreeCamera(); core_camera.setPosRot(0,p.x,p.y,p.z,q.x,q.y,q.z,q.w); return "camera moved"'''
    beamng.control.queue_lua_command(expression, response=True)


def survey(args):
    """Observe native scene guides + static collision in an explicit ENU box."""
    from datetime import datetime, timezone
    from native_scene import grid_queries, save_survey
    import yaml

    if args.output.exists():
        raise FileExistsError(
            "Choose a new output folder; existing exports are immutable"
        )
    xy, shape = grid_queries(args.bounds, args.spacing)
    if min(shape) < 2:
        raise ValueError("Choose bounds/spacing with at least two rows and columns")
    preset = None
    if args.level:
        name = args.level.removesuffix(".yaml")
        if Path(name).name != name:
            raise ValueError(
                "--level is an existing level preset name, as in ROS launch"
            )
        preset_path = ROOT / "src/bng_xal/bng_bringup/config/levels" / (name + ".yaml")
        preset = yaml.safe_load(preset_path.read_text())
        # Loading custom objects needs the full scenario builder. Do not claim
        # equivalence to a ROS preset while silently skipping its additions.
        if args.fresh and (preset.get("extra_objects") or preset.get("environment")):
            raise ValueError(
                "For presets with additions, load via ROS first and omit --fresh to survey the actual scene"
            )
    if args.fresh and preset is None:
        raise ValueError(
            "--fresh requires --level, selecting an existing ROS level preset"
        )
    beamng = connect(args)
    try:
        if args.fresh:
            from beamngpy import Scenario

            if beamng.control.get_gamestate()["state"] != "menu":
                beamng._send({"type": "StopScenario"}).ack("ScenarioStopped")
            scene = Scenario(preset["level"], "xlab_geometry_probe")
            scene.make(beamng)
            print("Loading vehicle-free " + preset["level"] + " test scene", flush=True)
            beamng.scenario.load(
                scene, connect_player_vehicle=False, connect_existing_vehicles=False
            )
            beamng.control.pause()
        inject(beamng)
        probe = lua_call(beamng, "xlabRoadGeometry.probe()")
        if not probe.get("mission"):
            raise ValueError("Load a level/scenario before surveying")
        if preset and "/levels/" + preset["level"] + "/" not in probe["mission"]:
            raise ValueError("Loaded base level does not match the requested preset")
        if args.fresh and probe["classes"]["BeamNGVehicle"]:
            raise ValueError("Unexpected vehicle in geometry-only scene")
        roads = beamng.scenario.get_road_network(
            include_edges=True, drivable_only=False
        )
        roads = {str(k): v for k, v in roads.items()}

        def observe(points):
            hits = []
            for start in range(0, len(points), 128):
                encoded = json.dumps(json.dumps(points[start : start + 128].tolist()))
                hits.extend(
                    lua_call(
                        beamng,
                        f"xlabRoadGeometry.survey(jsonDecode({encoded}),{args.bounds[2]},{args.bounds[5]})",
                    )
                )
            return hits

        print(
            f"Sampling {len(xy)} static collision columns; {len(roads)} native road guides",
            flush=True,
        )
        hits = observe(xy)
        # Independent cell-centre probes quantify the approximation one would
        # introduce by fitting/interpolating this point grid downstream.
        ny, nx = shape
        rng = np.random.default_rng(17)
        cell_ids = rng.choice(
            (nx - 1) * (ny - 1), min(1000, (nx - 1) * (ny - 1)), replace=False
        )
        base = (cell_ids // (nx - 1)) * nx + cell_ids % (nx - 1)
        check_xy = xy[base] + args.spacing / 2
        check_hits = observe(check_xy)
        meta = dict(
            source={
                "mission": probe["mission"],
                "kind": "native_scene_survey",
                "level_preset": args.level,
                "level_preset_sha256": (
                    hashlib.sha256(preset_path.read_bytes()).hexdigest()
                    if preset
                    else None
                ),
                "scene_content_hash": None,
            },
            captured_utc=datetime.now(timezone.utc).isoformat(),
            capabilities=probe,
            capture_lua_sha256=hashlib.sha256(LUA.read_bytes()).hexdigest(),
            engine_version=lua_call(
                beamng,
                "{version=tostring(beamng_version or 'unknown'),build=tostring(beamng_versionb or 'unknown')}",
            ),
            settings={
                "bounds_enu_m": args.bounds,
                "spacing_m": args.spacing,
                "grid_order": "Y outer, X inner; fixed spacing, last row may precede maximum bound",
                "check_seed": 17,
                "fresh_vehicle_free_scene": args.fresh,
            },
        )
        report = save_survey(
            args.output, meta, roads, xy, shape, hits, check_xy, check_hits
        )
        print(json.dumps(report, indent=2), flush=True)
        if args.fresh:
            low, high = np.asarray(args.bounds[:3]), np.asarray(args.bounds[3:])
            camera_at(beamng, (low + high) / 2, float(np.max(high[:2] - low[:2])))
        print(args.output / "inspect.html", flush=True)
    finally:
        beamng.disconnect()


def prepare(args):
    """Add immutable native graph/whole-footprint context to a saved survey."""
    from native_scene import (
        load_survey,
        grid_queries,
        pack_observations,
        write_scene_viewer,
    )

    meta = load_survey(args.export)
    paths = [
        args.export / name
        for name in ["overview.json", "overview_samples.npz", "navigation.json"]
    ]
    if any(p.exists() for p in paths):
        raise FileExistsError(
            "Overview/navigation already exists; original samples are not overwritten"
        )
    b = connect(args)
    try:
        inject(b)
        footprint = lua_call(b, "xlabRoadGeometry.footprint()")
        if footprint["mission"] != meta["source"]["mission"]:
            raise ValueError("Load the source level before preparing the overview")
        band = args.height_band
        bounds = footprint["minXY"] + [band[0]] + footprint["maxXY"] + [band[1]]
        xy, shape = grid_queries(bounds, args.spacing, max_columns=1_000_000)
        navigation = lua_call(b, "xlabRoadGeometry.navigation()")
        navigation["nodes"] = navigation["nodes"] or []
        navigation["edges"] = navigation["edges"] or []
        navigation["source_files_sha256"] = meta["files_sha256"]
        print(
            f'Whole-footprint overview: {len(xy)} columns, bounds={bounds}; {len(navigation["nodes"])} navigation nodes',
            flush=True,
        )
        hits = []
        for start in range(0, len(xy), 128):
            encoded = json.dumps(json.dumps(xy[start : start + 128].tolist()))
            hits.extend(
                lua_call(
                    b,
                    f"xlabRoadGeometry.survey(jsonDecode({encoded}),{band[0]},{band[1]})",
                )
            )
        arrays, objects = pack_observations(xy, hits)
        from surface_properties import pack_surface_properties

        arrays["surface_index"], surfaces = pack_surface_properties(
            hits, arrays["engine_matches_static"]
        )
        arrays["grid_shape"] = shape
        with paths[1].open("xb") as f:
            np.savez_compressed(f, **arrays)
        overview = dict(
            schema_version="0.3-overview",
            source_files_sha256=meta["files_sha256"],
            mission=footprint["mission"],
            bounds_enu_m=bounds,
            footprint_source=footprint["source"],
            spacing_m=args.spacing,
            samples_sha256=hashlib.sha256(paths[1].read_bytes()).hexdigest(),
            columns=len(xy),
            valid_hits=int(arrays["surface_valid"].sum()),
            surface_objects=objects,
            surfaces=surfaces,
            capture_lua_sha256=hashlib.sha256(LUA.read_bytes()).hexdigest(),
            caveat="Coarse visual context. Small obstacles, stacked layers, surface continuity and drivability are not certified.",
        )
        with paths[0].open("x") as f:
            json.dump(overview, f, indent=2)
        with paths[2].open("x") as f:
            json.dump(navigation, f, indent=2)
        print(write_scene_viewer(args.export), flush=True)
    finally:
        b.disconnect()


def extract_path(args):
    from track_recipe import load_recipe, corridor_queries, save_track

    if args.output.exists():
        raise FileExistsError("Choose a new output folder")
    recipe, meta = load_recipe(args.export, args.recipe)
    queries, lateral, left_axis, folded = corridor_queries(recipe)
    b = connect(args)
    try:
        inject(b)
        probe = lua_call(b, "xlabRoadGeometry.probe()")
        if probe["mission"] != recipe["source_mission"]:
            raise ValueError("Load the source level before extracting the corridor")
        if recipe["mode"] == "network":
            current = lua_call(b, "xlabRoadGeometry.navigation()")
            saved = json.loads((args.export / "navigation.json").read_text())
            if (
                [n["key"] for n in current["nodes"]]
                != [n["key"] for n in saved["nodes"]]
                or not np.allclose(
                    [n["xyz"] for n in current["nodes"]],
                    [n["xyz"] for n in saved["nodes"]],
                    atol=1e-5,
                    rtol=0,
                )
                or [(e["from"], e["to"]) for e in current["edges"]]
                != [(e["from"], e["to"]) for e in saved["edges"]]
            ):
                raise ValueError(
                    "Live navigation geometry/directions changed; prepare a matching source snapshot"
                )
        points = queries.reshape(-1, 3)
        settings = recipe["settings"]
        print(
            f"Extracting {len(queries)} stations × {len(lateral)} lateral columns = {len(points)} surface observations",
            flush=True,
        )
        hits = []
        for start in range(0, len(points), 128):
            encoded = json.dumps(json.dumps(points[start : start + 128].tolist()))
            hits.extend(
                lua_call(
                    b,
                    f'xlabRoadGeometry.survey(jsonDecode({encoded}),{-settings["ray_below_m"]},{settings["ray_above_m"]},true)',
                )
            )
        info = dict(
            mission=probe["mission"],
            capture_lua_sha256=hashlib.sha256(LUA.read_bytes()).hexdigest(),
            scene_content_hash=None,
            note="Base level and, for network routes, navigation geometry/directions checked. Complete collision/mod state is not hashed.",
        )
        print(
            json.dumps(
                save_track(
                    args.output,
                    recipe,
                    meta,
                    queries,
                    lateral,
                    left_axis,
                    folded,
                    hits,
                    info,
                ),
                indent=2,
            )
        )
        print(args.output / "inspect.html")
    finally:
        b.disconnect()


def refine_overview(args):
    from native_scene import (
        load_survey,
        grid_queries,
        pack_observations,
        write_scene_viewer,
    )

    meta = load_survey(args.export)
    original = json.loads((args.export / "overview.json").read_text())
    bounds = original["bounds_enu_m"]
    xy, shape = grid_queries(bounds, args.spacing, max_columns=1_000_000)
    name = "spacing_" + format(args.spacing, "g").replace(".", "p") + "m"
    target = args.export / "overviews"
    target.mkdir(exist_ok=True)
    raw, info_path = target / (name + ".npz"), target / (name + ".json")
    if raw.exists() or info_path.exists():
        raise FileExistsError("This overview resolution is already saved")
    b = connect(args)
    try:
        inject(b)
        probe = lua_call(b, "xlabRoadGeometry.probe()")
        if probe["mission"] != meta["source"]["mission"]:
            raise ValueError("Load the source level first")
        hits = []
        print(
            f"Refining overview to {args.spacing:g} m: {len(xy):,} columns", flush=True
        )
        for start in range(0, len(xy), 256):
            encoded = json.dumps(json.dumps(xy[start : start + 256].tolist()))
            hits.extend(
                lua_call(
                    b,
                    f"xlabRoadGeometry.survey(jsonDecode({encoded}),{bounds[2]},{bounds[5]})",
                )
            )
            if start and start % (256 * 200) == 0:
                print(f"  {start:,}/{len(xy):,}", flush=True)
        arrays, objects = pack_observations(xy, hits)
        arrays["grid_shape"] = shape
        from surface_properties import pack_surface_properties

        arrays["surface_index"], surfaces = pack_surface_properties(
            hits, arrays["engine_matches_static"]
        )
        with raw.open("xb") as f:
            np.savez_compressed(f, **arrays)
        info = original | dict(
            spacing_m=args.spacing,
            samples_file=raw.relative_to(args.export).as_posix(),
            samples_sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),
            columns=len(xy),
            valid_hits=int(arrays["surface_valid"].sum()),
            surface_objects=objects,
            surfaces=surfaces,
            capture_lua_sha256=hashlib.sha256(LUA.read_bytes()).hexdigest(),
        )
        with info_path.open("x") as f:
            json.dump(info, f, indent=2)
        (args.export / "overview_current.json").write_text(
            json.dumps(info, indent=2) + "\n"
        )
        print(write_scene_viewer(args.export), flush=True)
    finally:
        b.disconnect()


def capture_context(args):
    """Explicit stationary acquisition; never runs when opening a saved HTML."""
    from beamngpy import Scenario
    from native_scene import grid_queries, pack_observations
    from route_context import save_context

    folder = args.export
    if not np.isfinite(args.radius) or args.radius <= 0:
        raise ValueError("Context radius must be positive")
    if not (folder / "geometry.npz").exists():
        raise ValueError("Fresh context acquisition requires a managed asset mesh")
    if (folder / "context.npz").exists() or (folder / "context.json").exists():
        raise FileExistsError("Context already exists")
    geometry, meta = load_export(folder)
    asset = meta["source"].get("asset_name")
    if not asset:
        raise ValueError("Fresh context acquisition requires a managed asset")
    lo = geometry["vertices_xyz"].min(axis=0)
    hi = geometry["vertices_xyz"].max(axis=0)
    bounds = [
        lo[0] - args.radius,
        lo[1] - args.radius,
        min(-5.0, lo[2] - 5),
        hi[0] + args.radius,
        hi[1] + args.radius,
        hi[2] + 20,
    ]
    xy, _ = grid_queries(bounds, args.spacing)
    b = connect(args)
    try:
        manifest = lua_call(
            b, 'assert(jsonReadFile("/art/xlab_environments/manifest.json"))'
        )
        entry = manifest["assets"][asset]
        if entry["sha256"] != meta["asset_sha256"]:
            raise ValueError("Deployed asset differs from exported geometry")
        if b.control.get_gamestate()["state"] != "menu":
            b._send({"type": "StopScenario"}).ack("ScenarioStopped")
        scene = Scenario(entry["base_level"], "xlab_geometry_context_probe")
        scene.make(b)
        b.scenario.load(
            scene, connect_player_vehicle=False, connect_existing_vehicles=False
        )
        b.control.pause()
        inject(b)
        probe = lua_call(b, "xlabRoadGeometry.probe()")
        if probe["classes"]["BeamNGVehicle"]:
            raise ValueError("Unexpected vehicle in context scene")
        lua_call(b, "xlabRoadGeometry.capture(" + json.dumps(entry["path"]) + ")")
        hits = []
        print(f"Sampling {len(xy):,} nearby columns; zero vehicles", flush=True)
        for start in range(0, len(xy), 256):
            encoded = json.dumps(json.dumps(xy[start : start + 256].tolist()))
            hits.extend(
                lua_call(
                    b,
                    f"xlabRoadGeometry.survey(jsonDecode({encoded}),{bounds[2]},{bounds[5]})",
                )
            )
        arrays, _ = pack_observations(xy, hits)
        info = save_context(
            folder,
            arrays["surface_xyz"][arrays["surface_valid"]],
            args.radius,
            dict(
                kind="fresh_managed_scene_static_survey",
                mission=probe["mission"],
                asset_sha256=meta["asset_sha256"],
                spacing_m=args.spacing,
                bounds_enu_m=bounds,
                capture_lua_sha256=hashlib.sha256(LUA.read_bytes()).hexdigest(),
            ),
        )
        print(json.dumps(info, indent=2), flush=True)
    finally:
        b.disconnect()
