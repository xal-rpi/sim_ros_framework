"""Recipe validation and corridor query arrays. No fitted surface or dynamics."""

import hashlib
import json
from pathlib import Path

import numpy as np
from native_scene import load_survey, pack_observations
from surface_properties import pack_surface_properties, validate_surface_indices


def load_recipe(folder, path):
    return validate_recipe(folder, json.loads(Path(path).read_text()))


def validate_recipe(folder, recipe):
    """Validate saved intent, source identities and extraction settings before use."""
    folder = Path(folder)
    meta = load_survey(folder)
    if not isinstance(recipe, dict):
        raise ValueError("Track recipe must be an object")
    required = {
        "source_files_sha256",
        "source_mission",
        "reference_xyz",
        "source_reference_xyz",
        "reference_distance_xyz_m",
        "topology",
        "mode",
        "settings",
    }
    if required - recipe.keys():
        raise ValueError(
            f"Missing track recipe fields: {sorted(required - recipe.keys())}"
        )
    if (
        recipe.get("schema_version")
        not in ["0.3-track-recipe", "0.4-track-recipe", "0.5-track-recipe"]
        or recipe.get("frame") != "local_ENU"
    ):
        raise ValueError("Expected an ENU track recipe from the waypoint editor")
    if (
        recipe["source_files_sha256"] != meta["files_sha256"]
        or recipe["source_mission"] != meta["source"]["mission"]
    ):
        raise ValueError("Recipe belongs to another source survey")
    points = np.asarray(recipe["reference_xyz"], float)
    source = np.asarray(recipe["source_reference_xyz"], float)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or not 2 <= len(points) <= 10001
        or not np.isfinite(points).all()
    ):
        raise ValueError("Invalid reference station coordinates")
    if (
        source.ndim != 2
        or source.shape[1] != 3
        or len(source) < 2
        or not np.isfinite(source).all()
    ):
        raise ValueError("Invalid reference polyline")
    reference = np.asarray(recipe.get("reference_polyline_xyz", source), float)
    if (
        reference.ndim != 2
        or reference.shape[1] != 3
        or len(reference) < 2
        or not np.isfinite(reference).all()
    ):
        raise ValueError("Invalid constructed reference polyline")
    ds = np.linalg.norm(np.diff(reference, axis=0), axis=1)
    keep = np.r_[True, ds > 1e-9]
    filtered = reference[keep]
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(filtered, axis=0), axis=1))]
    station = np.asarray(recipe["reference_distance_xyz_m"], float)
    if (
        station.shape != (len(points),)
        or not np.isfinite(station).all()
        or station[0] != 0
        or (np.diff(station) <= 0).any()
        or abs(station[-1] - arc[-1]) > 1e-6
    ):
        raise ValueError("Invalid travel-order reference distances")
    expected = np.column_stack(
        [np.interp(station, arc, filtered[:, i]) for i in range(3)]
    )
    if not np.allclose(points, expected, atol=1e-6, rtol=0):
        raise ValueError("Stations do not match the saved reference intent")
    if recipe["topology"] not in ["open", "closed"]:
        raise ValueError("Choose explicit open or closed topology")
    if recipe["topology"] == "closed" and np.linalg.norm(points[-1] - points[0]) > 1e-6:
        raise ValueError("Declared loop has a position gap; no gap repair is performed")
    guide_geometry = None
    if recipe["mode"] == "network":
        raw = folder / "navigation.json"
        if hashlib.sha256(raw.read_bytes()).hexdigest() != recipe["navigation_sha256"]:
            raise ValueError("Native navigation snapshot differs")
        nav = json.loads(raw.read_text())
        if recipe.get("reference_kind") == "native_evaluated_guides":
            geometry_path = folder / recipe.get(
                "navigation_geometry_file", "navigation_geometry.json"
            )
            if not geometry_path.resolve().is_relative_to(folder.resolve()):
                raise ValueError(
                    "Guide association snapshot must be inside the source survey"
                )
            if hashlib.sha256(geometry_path.read_bytes()).hexdigest() != recipe.get(
                "navigation_geometry_sha256"
            ):
                raise ValueError("Evaluated guide association snapshot differs")
            guide_geometry = json.loads(geometry_path.read_text())
        edge_ids = recipe["navigation_edge_ids"]
        if len(edge_ids) != len(source) - 1:
            raise ValueError("Missing source edge references")
        if any(type(i) is not int or not 0 <= i < len(nav["edges"]) for i in edge_ids):
            raise ValueError("Invalid navigation source edge")
        checked_roles = []
        if (
            recipe.get("schema_version") == "0.5-track-recipe"
            and len(recipe.get("source_reference_roles", [])) != len(source) - 1
        ):
            raise ValueError("Missing native reference roles")
        if recipe.get("schema_version") == "0.5-track-recipe":
            for previous, current in zip(edge_ids[:-1], edge_ids[1:]):
                if (
                    previous != current
                    and nav["edges"][previous]["to"] != nav["edges"][current]["from"]
                ):
                    raise ValueError(
                        "Reference crosses disconnected directed navigation edges"
                    )
        for a, b, edge_id in zip(source[:-1], source[1:], edge_ids):
            if not isinstance(edge_id, int) or not 0 <= edge_id < len(nav["edges"]):
                raise ValueError("Invalid navigation source edge")
            edge = nav["edges"][edge_id]
            if guide_geometry is not None:
                from guide_geometry import project

                line = np.asarray(guide_geometry["edges"][edge_id]["points"])
                ta, qa, da = project(line, a)
                tb, qb, db = project(line, b)
                if (
                    tb < ta - 1e-5
                    or max(np.linalg.norm(qa - a), np.linalg.norm(qb - b)) > 1e-5
                ):
                    raise ValueError(
                        "Reference leaves its directed evaluated-guide association"
                    )
                if recipe.get("schema_version") == "0.5-track-recipe":
                    index = min(int(project(line, (a + b) / 2)[0]), len(line) - 2)
                    role = guide_geometry["edges"][edge_id]["roles"][index]
                    source_index = len(checked_roles)
                    if recipe["source_reference_roles"][source_index] != role:
                        raise ValueError(
                            "Reference role differs from its native association"
                        )
                    checked_roles.append(role)
                continue
            x, y = (
                np.asarray(nav["nodes"][edge[k]]["xyz"], float) for k in ["from", "to"]
            )
            delta = y - x
            if delta @ delta < 1e-20:
                raise ValueError("Degenerate native navigation edge")
            ta, tb = (float((p - x) @ delta / (delta @ delta)) for p in [a, b])
            if (
                min(ta, tb) < -1e-6
                or max(ta, tb) > 1 + 1e-6
                or tb < ta - 1e-6
                or max(
                    np.linalg.norm(a - (x + ta * delta)),
                    np.linalg.norm(b - (x + tb * delta)),
                )
                > 1e-5
            ):
                raise ValueError("Reference segment leaves its directed native edge")
    elif recipe["mode"] != "free":
        raise ValueError("Unknown waypoint source mode")
    from reference_path import replay_reference

    replayed = replay_reference(recipe, guide_geometry)
    if replayed.shape != reference.shape or not np.allclose(
        reference, replayed, atol=1e-8, rtol=0
    ):
        raise ValueError("Constructed reference does not match the recorded operation")
    corridor_queries(recipe)
    return recipe, meta


def corridor_queries(recipe):
    points = np.asarray(recipe["reference_xyz"], float)
    s = recipe["settings"]
    keys = [
        "left_extent_m",
        "right_extent_m",
        "extra_margin_m",
        "station_spacing_m",
        "lateral_spacing_m",
        "ray_above_m",
        "ray_below_m",
    ]
    if not isinstance(s, dict) or set(s) != set(keys):
        raise ValueError("Extraction settings must contain exactly: " + ", ".join(keys))
    if any(type(s[k]) not in (int, float) for k in keys):
        raise ValueError("Extraction settings must be numeric")
    values = np.array([s[k] for k in keys], float)
    if (
        not np.isfinite(values).all()
        or (values[[0, 1, 3, 4, 5, 6]] <= 0).any()
        or values[2] < 0
    ):
        raise ValueError("Invalid extents, sampling spacing or height band")
    left, right = (
        s["left_extent_m"] + s["extra_margin_m"],
        s["right_extent_m"] + s["extra_margin_m"],
    )
    nr, nl = int(np.ceil(right / s["lateral_spacing_m"])), int(
        np.ceil(left / s["lateral_spacing_m"])
    )
    if len(points) * (nr + nl + 1) > 250000:
        raise ValueError(
            "Limit a corridor to 250,000 columns; increase spacing or select a shorter section"
        )
    lateral = np.r_[np.linspace(-right, 0, nr + 1)[:-1], np.linspace(0, left, nl + 1)]
    tangent = np.empty_like(points)
    tangent[1:-1] = points[2:] - points[:-2]
    tangent[0], tangent[-1] = points[1] - points[0], points[-1] - points[-2]
    if recipe["topology"] == "closed":
        tangent[0] = tangent[-1] = points[1] - points[-2]
    horizontal = np.linalg.norm(tangent[:, :2], axis=1)
    if (horizontal < 1e-9).any():
        raise ValueError(
            "Reference has an undefined horizontal tangent, e.g. a direct U-turn"
        )
    left_axis = np.column_stack(
        (-tangent[:, 1] / horizontal, tangent[:, 0] / horizontal, np.zeros(len(points)))
    )
    query = points[:, None, :] + lateral[None, :, None] * left_axis[:, None, :]
    # Detect folded sampling cells in XY. A wide offset strip through a tight
    # corner can fold even if the underlying reference path is continuous.
    forward = query[1:, :-1, :2] - query[:-1, :-1, :2]
    sideways = query[:-1, 1:, :2] - query[:-1, :-1, :2]
    jac = forward[:, :, 0] * sideways[:, :, 1] - forward[:, :, 1] * sideways[:, :, 0]
    return query, lateral, left_axis, jac <= 0


def save_track(
    folder, recipe, source_meta, queries, lateral, left_axis, folded, hits, capture_info
):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    n, k, _ = queries.shape
    observations, objects = pack_observations(queries.reshape(-1, 3)[:, :2], hits)
    surface_index, surfaces = pack_surface_properties(
        hits, observations["engine_matches_static"]
    )
    center_index = int(np.argmin(abs(lateral)))
    surface = observations["surface_xyz"].reshape(n, k, 3)
    center = surface[:, center_index]
    valid = observations["surface_valid"].reshape(n, k)
    center_valid = valid[:, center_index]
    arrays = dict(
        station_id=np.arange(n),
        reference_xyz=recipe["reference_xyz"],
        center_xyz=center,
        center_valid=center_valid,
        reference_distance_xyz_m=recipe["reference_distance_xyz_m"],
        section_xyz=observations["surface_xyz"],
        section_valid=observations["surface_valid"],
        section_normal=observations["surface_normal"],
        section_normal_valid=observations["normal_valid"],
        section_object_index=observations["surface_object_index"],
        section_surface_index=surface_index,
        center_surface_index=surface_index.reshape(n, k)[:, center_index],
        section_reference_xyz=queries.reshape(-1, 3),
        section_lateral_m=np.tile(lateral, n),
        section_offsets=np.arange(n + 1, dtype=np.int64) * k,
        horizontal_left_axis=left_axis,
        chosen_limits_m=np.tile(
            [
                -recipe["settings"]["right_extent_m"],
                recipe["settings"]["left_extent_m"],
            ],
            (n, 1),
        ),
        physical_limits_m=np.full((n, 2), np.nan),
        physical_limits_valid=np.zeros((n, 2), bool),
        folded_xy_cell=folded,
    )
    np.savez_compressed(folder / "route.npz", **arrays)
    (folder / "recipe.json").write_text(json.dumps(recipe, indent=2) + "\n")
    metadata = dict(
        schema_version="0.3-track-samples",
        frame="local_ENU",
        units="m",
        source=source_meta["source"],
        source_files_sha256=source_meta["files_sha256"],
        topology=recipe["topology"],
        repetitions=None,
        surface_objects=objects,
        surfaces=surfaces,
        query_construction="Horizontal sections perpendicular to the reference chord tangent; ENU up cross horizontal forward is left. Samples ordered right to left. Z bands are relative to each saved reference Z.",
        station_convention="Travel-order station_id; reference_distance_xyz_m is 3D chord distance on the designed/nav reference, not measured surface progress.",
        surface_rule="First static hit within explicit relative height band. Missing points remain NaN; no stitching, smoothing or triangulation.",
        limits_convention="chosen_limits_m are user extraction extents before extra margin; not certified physical edges or optimizer limits.",
        unavailable=[
            "physical pavement edges",
            "certified drivable area",
            "all stacked layers",
            "grip",
            "speed constraints",
        ],
        capture=capture_info,
        route_sha256=hashlib.sha256((folder / "route.npz").read_bytes()).hexdigest(),
    )
    metadata["reference_design"] = dict(
        recipe_file="recipe.json",
        recipe_sha256=hashlib.sha256((folder / "recipe.json").read_bytes()).hexdigest(),
        operations=recipe.get("reference_operations", []),
        transition_warnings=recipe.get("transition_warnings", []),
        navigation_geometry_file=recipe.get(
            "navigation_geometry_file", "navigation_geometry.json"
        ),
        navigation_geometry_sha256=recipe.get("navigation_geometry_sha256"),
    )
    (folder / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    from route_schema import normalize_export

    normalize_export(folder, preserve_recipe_hashes=False)
    report = validate_track(folder)
    write_track_viewer(folder)
    return report


def validate_track(folder):
    folder = Path(folder)
    meta = json.loads((folder / "metadata.json").read_text())
    from route_schema import VERSION, validate_routes

    if meta["schema_version"] == VERSION:
        return validate_routes(folder)["routes"][0]
    if (
        hashlib.sha256((folder / "route.npz").read_bytes()).hexdigest()
        != meta["route_sha256"]
    ):
        raise ValueError("Saved route arrays changed")
    with np.load(folder / "route.npz", allow_pickle=False) as z:
        valid, nv = z["section_valid"], z["section_normal_valid"]
        off = z["section_offsets"]
        ordered = all(
            (np.diff(z["section_lateral_m"][a:b]) > 0).all()
            for a, b in zip(off[:-1], off[1:])
        )
        checks = dict(
            reference_finite=bool(np.isfinite(z["reference_xyz"]).all()),
            valid_surface_finite=bool(np.isfinite(z["section_xyz"][valid]).all()),
            valid_normal_finite=bool(np.isfinite(z["section_normal"][nv]).all()),
            sections_ordered=bool(ordered),
            offsets_valid=bool(
                off[0] == 0 and off[-1] == len(valid) and (np.diff(off) > 0).all()
            ),
        )
        if "section_surface_index" in z:
            checks["surface_indices_valid"] = validate_surface_indices(
                z["section_surface_index"], meta["surfaces"], valid
            )
            checks["center_surface_indices_valid"] = validate_surface_indices(
                z["center_surface_index"], meta["surfaces"], z["center_valid"]
            )
        sections = z["section_xyz"].reshape(len(off) - 1, -1, 3)
        flags = np.abs(np.diff(sections[:, :, 2], axis=1)) > 0.3
        folded_rows = np.flatnonzero(z["folded_xy_cell"].any(axis=1))
        missing_rows = np.flatnonzero(~valid.reshape(len(off) - 1, -1).all(axis=1))
        defect_rows = np.unique(np.r_[folded_rows, folded_rows + 1, missing_rows])
        report = dict(
            array_checks_passed=all(checks.values()),
            checks=checks,
            stations=len(z["station_id"]),
            query_columns=len(valid),
            valid_surface_hits=int(valid.sum()),
            valid_normals=int(nv.sum()),
            missing_center_stations=int((~z["center_valid"]).sum()),
            folded_xy_cells=int(z["folded_xy_cell"].sum()),
            lateral_height_jumps_over_0_3m=int(flags.sum()),
            inspection_station_ids=defect_rows.tolist(),
            reference_to_measured_center_abs_p95_m=(
                float(
                    np.nanpercentile(
                        abs(z["center_xyz"][:, 2] - z["reference_xyz"][:, 2]), 95
                    )
                )
                if z["center_valid"].any()
                else None
            ),
            closed_reference_gap_m=(
                float(np.linalg.norm(z["reference_xyz"][-1] - z["reference_xyz"][0]))
                if meta["topology"] == "closed"
                else None
            ),
            caveat="Folds and height jumps are warnings for inspection, not a drivable-area classifier. No surface fit or connectivity repair.",
        )
    (folder / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def write_track_viewer(folder):
    """Use the same route inspector as managed track exports."""
    from viewer import write_viewer

    return write_viewer(folder)
