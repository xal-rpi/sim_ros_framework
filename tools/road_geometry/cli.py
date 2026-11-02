#!/usr/bin/env python3
"""Stationary BeamNG geometry probes, evaluated mesh export and offline inspection."""

import argparse
import json
from pathlib import Path
import pickle

import numpy as np
from geometry import HeightIndex, load_export, select_recipe, validate
from acquisition import (
    capture,
    survey,
    prepare,
    extract_path,
    refine_overview,
    capture_context,
    camera_at,
    connect,
    inject,
    lua_call,
)


def compare_log(args):
    import yaml

    geometry, meta = load_export(args.export)
    index = HeightIndex(geometry)
    if args.stride < 1:
        raise ValueError("--stride must be positive")
    run = Path(args.log).parents[1]
    recorded = yaml.safe_load((run / "metadata.yaml").read_text())["sim"]
    hashes = {
        a["sha256"] for a in recorded.get("environment", {}).get("loaded_assets", [])
    }
    if meta["asset_sha256"] not in hashes:
        raise ValueError("Exported asset hash does not match this recording")
    config = recorded["vehicles"]["EGO"]["sensors"]["gtstate"]["surface"]
    radius = float(config["normal_radius"])
    ray_range = float(config["max_distance"])
    with Path(args.log).open("rb") as f:
        g = pickle.load(f)["data"][("EGO", "gtstate")]
    report = {
        "log": str(args.log),
        "asset_sha256": meta["asset_sha256"],
        "stride": args.stride,
        "normal_radius_m": radius,
        "ray_range_m": ray_range,
        "locations": {},
    }
    for loc in ["CoG", "FL", "FR", "RL", "RR"]:
        prefix = "surface" + loc
        ids = range(0, len(g["time"]), args.stride)
        errors = []
        normal_errors = []
        missing = 0
        count = 0
        for j in ids:
            if not g[prefix + "_heightValid"][j]:
                continue
            count += 1
            p = np.array([g[prefix + "_queryPos_" + a][j] for a in "xyz"])
            # Match the actual logged static ray's origin/range, not CoG Z.
            hit = index.height(p, above=0.5, below=ray_range - 0.5)
            if hit is None:
                missing += 1
                continue
            z, _ = hit
            errors.append(z - g[prefix + "_height"][j])
            heights = []
            for dx, dy in [
                (0, radius),
                (0.8660254037844386 * radius, -0.5 * radius),
                (-0.8660254037844386 * radius, -0.5 * radius),
            ]:
                top = 0.5 + 2 * radius
                neighbor = index.height(
                    [p[0] + dx, p[1] + dy, z], above=top, below=ray_range - top
                )
                heights.append(neighbor[0] if neighbor else np.nan)
            if np.isfinite(heights).all() and g[prefix + "_normalValid"][j]:
                z1, z2, z3 = heights
                a = np.array([2 * 0.8660254037844386 * radius, 0, z2 - z3])
                b = np.array([0.8660254037844386 * radius, 1.5 * radius, z1 - z3])
                n = np.cross(a, b)
                n /= np.linalg.norm(n)
                actual = np.array([g[prefix + "_normal_" + axis][j] for axis in "xyz"])
                normal_errors.append(
                    float(np.degrees(np.arccos(np.clip(n @ actual, -1, 1))))
                )
        report["locations"][loc] = dict(
            valid_logged_samples=count,
            missing_mesh_hits=missing,
            height_rmse_m=(
                float(np.sqrt(np.mean(np.square(errors)))) if errors else None
            ),
            height_abs_p95_m=(
                float(np.percentile(abs(np.array(errors)), 95)) if errors else None
            ),
            height_abs_max_m=float(np.max(abs(np.array(errors)))) if errors else None,
            normal_samples=len(normal_errors),
            normal_angle_median_p95_deg=(
                np.percentile(normal_errors, [50, 95]).tolist()
                if normal_errors
                else None
            ),
        )
    path = Path(args.export) / (
        "comparison_" + Path(args.log).parents[1].name + ".json"
    )
    path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=25252)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("probe")
    p = sub.add_parser("capture")
    p.add_argument("--asset", required=True)
    p.add_argument("--fresh", action="store_true")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--samples", type=int, default=400)
    p = sub.add_parser(
        "survey", help="Native road guides + bounded static collision point survey"
    )
    p.add_argument("--level", help="Existing ROS level preset; required with --fresh")
    p.add_argument(
        "--fresh",
        action="store_true",
        help="Replace scene with a paused vehicle-free base-level scene",
    )
    p.add_argument(
        "--bounds",
        type=float,
        nargs=6,
        required=True,
        metavar=("XMIN", "YMIN", "ZMIN", "XMAX", "YMAX", "ZMAX"),
    )
    p.add_argument("--spacing", type=float, default=1.0)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser(
        "prepare",
        help="Add coarse whole-level context and the native navigation graph to an existing survey",
    )
    p.add_argument("export", type=Path)
    p.add_argument("--spacing", type=float, default=1)
    p.add_argument(
        "--height-band", type=float, nargs=2, required=True, metavar=("ZMIN", "ZMAX")
    )
    p = sub.add_parser(
        "extract-path",
        help="Query measured cross-sections along a saved waypoint corridor",
    )
    p.add_argument("export", type=Path)
    p.add_argument("recipe", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser(
        "rebuild-reference",
        help="Replan an evaluated-guide recipe with bounded, recorded node transitions; offline, requires Node.js",
    )
    p.add_argument("export", type=Path)
    p.add_argument("recipe", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser(
        "refine-overview",
        help="Capture a denser whole-map preview without replacing the original",
    )
    p.add_argument("export", type=Path)
    p.add_argument("--spacing", type=float, required=True)
    p = sub.add_parser(
        "serve", help="Open the local editor with direct export-folder save/load"
    )
    p.add_argument("export", type=Path)
    p.add_argument("--http-port", type=int, default=8766)
    p = sub.add_parser(
        "import-recipe",
        help="Copy a browser-downloaded recipe into this export selections folder",
    )
    p.add_argument("export", type=Path)
    p.add_argument("recipe", type=Path)
    p.add_argument("--name", required=True)
    for name in ["inspect", "validate"]:
        p = sub.add_parser(name)
        p.add_argument("export", type=Path)
    p = sub.add_parser("compare-log")
    p.add_argument("export", type=Path)
    p.add_argument("log", type=Path)
    p.add_argument("--stride", type=int, default=10)
    p = sub.add_parser("camera")
    p.add_argument("recipe", type=Path)
    p.add_argument("--height", type=float, default=30)
    p = sub.add_parser("select")
    p.add_argument("export", type=Path)
    p.add_argument("recipe", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser(
        "select-route",
        help="Apply a named start/seam, direction and section choice to new arrays offline",
    )
    p.add_argument("export", type=Path)
    p.add_argument("recipe", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser(
        "add-context",
        help="Attach nearby visualization-only collision observations from a saved survey or fresh managed scene",
    )
    p.add_argument("export", type=Path)
    p.add_argument("--radius", type=float, default=25)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--survey", type=Path)
    mode.add_argument("--fresh", action="store_true")
    p.add_argument("--spacing", type=float, default=1)
    p = sub.add_parser(
        "export-json",
        help="Write measured route arrays and surface metadata as portable JSON, not a selection recipe",
    )
    p.add_argument("export", type=Path)
    p.add_argument("--route")
    p.add_argument(
        "--include-mesh",
        action="store_true",
        help="Include evaluated triangles and face normals for a managed asset",
    )
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser(
        "normalize-routes",
        help="Publish a normalized copy offline; source export stays unchanged",
    )
    p.add_argument("export", type=Path)
    p.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "capture":
        capture(args)
    elif args.command == "survey":
        survey(args)
    elif args.command == "prepare":
        prepare(args)
    elif args.command == "extract-path":
        extract_path(args)
    elif args.command == "rebuild-reference":
        from rebuild_reference import rebuild_reference

        print(
            json.dumps(
                rebuild_reference(args.export, args.recipe, args.output), indent=2
            )
        )
    elif args.command == "refine-overview":
        refine_overview(args)
    elif args.command == "serve":
        from local_server import serve

        serve(args.export, args.http_port)
    elif args.command == "select-route":
        from route_selection import materialize

        print(
            materialize(args.export, json.loads(args.recipe.read_text()), args.output)
        )
    elif args.command == "add-context":
        from route_context import from_survey

        if args.survey:
            print(
                json.dumps(from_survey(args.export, args.survey, args.radius), indent=2)
            )
        else:
            capture_context(args)
        from viewer import write_viewer

        print(write_viewer(args.export))
    elif args.command == "import-recipe":
        from local_server import save_document

        document = json.loads(args.recipe.read_text())
        kinds = {
            "0.1-route-selection": "routes",
            "0.2-native-selection": "areas",
            "0.3-track-recipe": "tracks",
            "0.4-track-recipe": "tracks",
            "0.5-track-recipe": "tracks",
        }
        if document.get("schema_version") not in kinds:
            raise ValueError("Unsupported selection schema")
        kind = kinds[document["schema_version"]]
        print(save_document(args.export, kind, args.name, document))
    elif args.command == "export-json":
        from route_json import export_route_json

        print(
            export_route_json(args.export, args.output, args.route, args.include_mesh)
        )
    elif args.command == "normalize-routes":
        from route_schema import migrate_export

        print(json.dumps(migrate_export(args.export, args.output), indent=2))
    elif args.command == "compare-log":
        compare_log(args)
    elif args.command == "validate":
        meta = json.loads((args.export / "metadata.json").read_text())
        if meta["schema_version"] == "0.5-road-routes":
            from route_schema import validate_routes

            print(json.dumps(validate_routes(args.export), indent=2))
        elif meta["schema_version"] == "0.4-selected-route":
            from route_selection import validate_selected

            print(json.dumps(validate_selected(args.export), indent=2))
        elif meta["schema_version"] == "0.2-native-survey":
            from native_scene import validate_survey

            print(json.dumps(validate_survey(args.export), indent=2))
        elif meta["schema_version"] == "0.3-track-samples":
            from track_recipe import validate_track

            print(json.dumps(validate_track(args.export), indent=2))
        elif meta["schema_version"] == "0.1-probe":
            print(json.dumps(validate(args.export), indent=2))
        else:
            raise ValueError("Unsupported export schema")
    elif args.command == "select":
        if (
            json.loads((args.export / "metadata.json").read_text())["schema_version"]
            == "0.2-native-survey"
        ):
            raise ValueError(
                "Native survey recipes are area/source bookmarks, not selected routes. Use camera or a new bounded survey."
            )
        print(
            json.dumps(
                select_recipe(
                    args.export, json.loads(args.recipe.read_text()), args.output
                ),
                indent=2,
            )
        )
    elif args.command == "inspect":
        meta = json.loads((args.export / "metadata.json").read_text())
        if meta["schema_version"] == "0.2-native-survey":
            from native_scene import write_scene_viewer

            print(write_scene_viewer(args.export))
        elif meta["schema_version"] == "0.3-track-samples":
            from track_recipe import write_track_viewer

            print(write_track_viewer(args.export))
        else:
            from viewer import write_viewer

            print(write_viewer(args.export))
    else:
        b = connect(args)
        try:
            if args.command == "probe":
                inject(b)
                print(json.dumps(lua_call(b, "xlabRoadGeometry.probe()"), indent=2))
            else:
                recipe = json.loads(args.recipe.read_text())
                if recipe.get("frame") != "local_ENU":
                    raise ValueError("Camera recipe must use local_ENU")
                if "source_mission" in recipe:
                    current = lua_call(b, "{mission=getMissionFilename()}")
                    if current["mission"] != recipe["source_mission"]:
                        raise ValueError(
                            "Camera bookmark belongs to a different loaded level"
                        )
                if (
                    not np.isfinite(recipe["focus_xyz"]).all()
                    or not np.isfinite(args.height)
                    or args.height <= 0
                ):
                    raise ValueError("Camera target must be finite and height positive")
                camera_at(b, recipe["focus_xyz"], args.height)
        finally:
            b.disconnect()


if __name__ == "__main__":
    main()
