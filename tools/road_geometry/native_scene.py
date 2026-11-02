"""Native-scene observations, kept separate from managed-asset triangle meshes.

A decal is a reference guide, not evidence of pavement. A ray survey is a
height-band observation, not a mesh, a route, or a drivable-area classifier.
"""

import hashlib
import json
from pathlib import Path

import numpy as np
from surface_properties import pack_surface_properties, validate_surface_indices


def grid_queries(bounds, spacing, max_columns=250_000):
    bounds = np.asarray(bounds, float)
    if bounds.shape != (6,) or not np.isfinite(bounds).all():
        raise ValueError("Bounds must be finite Xmin Ymin Zmin Xmax Ymax Zmax")
    low, high = bounds[:3], bounds[3:]
    if (high <= low).any() or not np.isfinite(spacing) or spacing <= 0:
        raise ValueError("Bounds must increase; spacing must be positive")
    # Exact requested spacing; do not move the last row onto the far boundary.
    counts = (high[:2] - low[:2]) / spacing
    if not np.isfinite(counts).all() or (counts > max_columns).any():
        raise ValueError(
            f"Limit each survey to {max_columns:,} columns; use smaller explicit regions"
        )
    nx, ny = (int(np.floor(value)) + 1 for value in counts)
    if nx * ny > max_columns:
        raise ValueError(
            f"Limit each survey to {max_columns:,} columns; use smaller explicit regions"
        )
    x = low[0] + np.arange(nx) * spacing
    y = low[1] + np.arange(ny) * spacing
    xx, yy = np.meshgrid(x, y)
    return np.column_stack((xx.ravel(), yy.ravel())), (len(y), len(x))


def pack_observations(xy, hits):
    """Use static collision heights; attach native normals only on matching hits."""
    xy = np.asarray(xy, float)
    if len(xy) != len(hits):
        raise ValueError("One observation required for every query")
    valid = np.array([h["valid"] for h in hits], bool)
    z = np.array([h.get("height", np.nan) for h in hits], float)
    xyz = np.column_stack((xy, z))
    engine = np.array([h.get("enginePoint", [np.nan] * 3) for h in hits], float)
    normals = np.array([h.get("engineNormal", [np.nan] * 3) for h in hits], float)
    # The two ray APIs sometimes disagree/miss. Never attach a normal or an
    # object identity to a different collision hit just because XY is the same.
    same = (
        valid
        & np.isfinite(engine).all(axis=1)
        & (np.linalg.norm(engine - xyz, axis=1) <= 0.001)
    )
    normal_valid = (
        same
        & np.isfinite(normals).all(axis=1)
        & (np.linalg.norm(normals, axis=1) > 0.5)
    )
    normals[~normal_valid] = np.nan
    objects, object_index = [], {}
    object_ids = np.full(len(hits), -1, np.int64)
    for i, hit in enumerate(hits):
        if not same[i] or "engineObject" not in hit:
            continue
        entry = {
            k: hit[k]
            for k in (
                "engineObject",
                "engineObjectId",
                "engineObjectClass",
                "engineObjectPersistentId",
                "engineObjectShape",
                "engineObjectName",
            )
            if k in hit
        }
        key = json.dumps(entry, sort_keys=True)
        if key not in object_index:
            object_index[key] = len(objects)
            objects.append(entry)
        object_ids[i] = object_index[key]
    terrain = np.array([h.get("terrainHeight", np.nan) for h in hits], float)
    terrain_normals = np.array(
        [h.get("terrainNormal", [np.nan] * 3) for h in hits], float
    )
    return (
        dict(
            query_xy=xy,
            surface_xyz=xyz,
            surface_valid=valid,
            surface_normal=normals,
            normal_valid=normal_valid,
            surface_object_index=object_ids,
            engine_xyz=engine,
            engine_matches_static=same,
            terrain_height=terrain,
            terrain_normal=terrain_normals,
        ),
        objects,
    )


def pack_guides(roads):
    rows, offsets, sources = [], [0], []
    for runtime_id, road in sorted(
        roads.items(), key=lambda item: (item[1].get("persistentId", ""), str(item[0]))
    ):
        edges = road.get("edges", [])
        if not edges:
            continue
        points = np.asarray(
            [[e["right"], e["middle"], e["left"]] for e in edges], float
        )
        rows.append(points)
        offsets.append(offsets[-1] + len(points))
        sources.append(
            {
                "runtime_id": str(runtime_id),
                "persistent_id": road.get("persistentId") or None,
                "material": road.get("material"),
                "drivability": road.get("drivability"),
                "one_way": road.get("oneWay"),
                "flip_direction": road.get("flipDirection"),
                "declared_looped": road.get("looped"),
                "name": road.get("name") or None,
            }
        )
    return (
        dict(
            edge_xyz=np.concatenate(rows) if rows else np.empty((0, 3, 3)),
            road_offsets=np.asarray(offsets, np.int64),
        ),
        sources,
    )


def interpolation_diagnostics(grid, shape, check_xy, checks):
    """Measure a possible height-grid approximation; never output its triangles.

    Cell-centre collision queries are independent of the grid corner queries.
    Large errors remain visible, including barriers, roofs and height jumps.
    """
    ny, nx = shape
    if nx < 2 or ny < 2:
        return {"valid_checks": 0}, np.empty(0)
    x0, y0 = grid["query_xy"][0]
    dx = grid["query_xy"][1, 0] - x0
    dy = grid["query_xy"][nx, 1] - y0
    ix = np.rint((check_xy[:, 0] - x0) / dx - 0.5).astype(int)
    iy = np.rint((check_xy[:, 1] - y0) / dy - 0.5).astype(int)
    base = iy * nx + ix
    corners = np.column_stack((base, base + 1, base + nx, base + nx + 1))
    valid = grid["surface_valid"][corners].all(axis=1) & checks["surface_valid"]
    error = np.full(len(base), np.nan)
    error[valid] = (
        grid["surface_xyz"][corners[valid], 2].mean(axis=1)
        - checks["surface_xyz"][valid, 2]
    )

    def stats(values):
        if not len(values):
            return None
        return dict(
            abs_median_p95_max_m=np.percentile(abs(values), [50, 95, 100]).tolist(),
            rmse_m=float(np.sqrt(np.mean(values**2))),
        )

    # Matching native object IDs are useful diagnostics, not proof a cell is
    # continuous: two faces of the same terrain/mesh can still contain a jump.
    ids = grid["surface_object_index"][corners]
    same_object = valid & (ids[:, 0] >= 0) & (ids == ids[:, :1]).all(axis=1)
    return (
        dict(
            valid_checks=int(valid.sum()),
            total_checks=len(base),
            all_valid_cells=stats(error[valid]),
            same_corner_object_cells=stats(error[same_object]),
            caveat="Midpoint bilinear error only; not a bound on error within a cell or proof of connectivity.",
        ),
        error,
    )


def save_survey(folder, metadata, roads, xy, shape, hits, check_xy, check_hits):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    arrays, objects = pack_observations(xy, hits)
    checks, check_objects = pack_observations(check_xy, check_hits)
    arrays["surface_index"], surfaces = pack_surface_properties(
        hits, arrays["engine_matches_static"]
    )
    checks["surface_index"], check_surfaces = pack_surface_properties(
        check_hits, checks["engine_matches_static"]
    )
    guides, sources = pack_guides(roads)
    summary, error = interpolation_diagnostics(arrays, shape, check_xy, checks)
    arrays["grid_shape"] = np.asarray(shape, np.int64)
    np.savez_compressed(folder / "surface_samples.npz", **arrays)
    np.savez_compressed(folder / "road_guides.npz", **guides)
    np.savez_compressed(folder / "check_samples.npz", **checks, bilinear_error_m=error)
    # Retain the complete native road fields, including the semantic flags.
    (folder / "roads.json").write_text(json.dumps(roads, indent=2) + "\n")
    metadata.update(
        schema_version="0.2-native-survey",
        units="m",
        world_frame="local_ENU",
        axes=["project_east", "project_north", "up"],
        handedness="right",
        T_enu_from_native=np.eye(4).tolist(),
        geographic_datum=None,
        complete_scene=False,
        native_collision_mesh_available=False,
        roads=sources,
        surface_objects=objects,
        check_objects=check_objects,
        surfaces=surfaces,
        check_surfaces=check_surfaces,
        routes=[],
        physical_boundaries=None,
        connectivity=None,
        observation="First static vertical collision hit within explicit Z band, sampled at fixed XY; no triangulation.",
        guide_convention="Native getEdgesTable right/middle/left, in native station order. Direction flags retained; no route chosen or closed.",
        normal_convention="Native Engine.castRay normals, unflipped, only when hit agrees with static height within 1 mm; invalid is NaN + validity flag.",
        unavailable=[
            "exact native collision triangle connectivity",
            "certified drivable areas/edges/holes",
            "all stacked surface layers",
            "grip",
            "obstacle semantics",
            "route/corridor selection",
        ],
    )
    metadata["files_sha256"] = {
        name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
        for name in [
            "surface_samples.npz",
            "road_guides.npz",
            "check_samples.npz",
            "roads.json",
        ]
    }
    (folder / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    valid = arrays["surface_valid"]
    difference = arrays["surface_xyz"][:, 2] - arrays["terrain_height"]
    report = dict(
        columns=len(xy),
        static_hits=int(valid.sum()),
        normals_available=int(arrays["normal_valid"].sum()),
        native_roads=len(sources),
        navigation_candidate_roads=sum(float(r["drivability"]) >= 0 for r in sources),
        surface_z_min_max_m=(
            [
                float(np.nanmin(arrays["surface_xyz"][:, 2])),
                float(np.nanmax(arrays["surface_xyz"][:, 2])),
            ]
            if valid.any()
            else None
        ),
        collision_above_terrain_over_5cm=int((valid & (difference > 0.05)).sum()),
        collision_below_terrain_over_5cm=int((valid & (difference < -0.05)).sum()),
        midpoint_grid_approximation=summary,
    )
    (folder / "diagnostics.json").write_text(json.dumps(report, indent=2) + "\n")
    validate_survey(folder)
    write_scene_viewer(folder)
    return report


def load_survey(folder):
    folder = Path(folder)
    meta = json.loads((folder / "metadata.json").read_text())
    if meta.get("schema_version") != "0.2-native-survey":
        raise ValueError("Expected a native survey export")
    for name, digest in meta["files_sha256"].items():
        if not (folder / name).resolve().is_relative_to(folder.resolve()):
            raise ValueError("Survey file escapes the export")
        if hashlib.sha256((folder / name).read_bytes()).hexdigest() != digest:
            raise ValueError(f"Survey file changed: {name}")
    return meta


def validate_survey(folder):
    """Check saved observations without turning their grid into a surface."""
    folder = Path(folder)
    meta = load_survey(folder)
    with np.load(folder / "surface_samples.npz", allow_pickle=False) as z:
        n = len(z["query_xy"])
        valid, nv = z["surface_valid"], z["normal_valid"]
        low, high = np.asarray(meta["settings"]["bounds_enu_m"]).reshape(2, 3)
        checks = dict(
            query_xy_finite=bool(np.isfinite(z["query_xy"]).all()),
            grid_shape_valid=bool(np.prod(z["grid_shape"]) == n),
            hit_coordinates_finite=bool(np.isfinite(z["surface_xyz"][valid]).all()),
            hit_xy_matches_query=bool(
                np.array_equal(z["surface_xyz"][:, :2], z["query_xy"])
            ),
            hits_inside_z_band=bool(
                (
                    (z["surface_xyz"][valid, 2] >= low[2])
                    & (z["surface_xyz"][valid, 2] <= high[2])
                ).all()
            ),
            normals_match_static_hits=bool(
                (~nv | (valid & z["engine_matches_static"])).all()
            ),
            valid_normals_finite=bool(np.isfinite(z["surface_normal"][nv]).all()),
            valid_normals_unit=bool(
                np.allclose(
                    np.linalg.norm(z["surface_normal"][nv], axis=1), 1, atol=1e-4
                )
            ),
        )
        downward_normal_count = int((z["surface_normal"][nv, 2] < 0).sum())
        if "surface_index" in z:
            checks["surface_indices_valid"] = validate_surface_indices(
                z["surface_index"], meta["surfaces"], valid
            )
    with np.load(folder / "road_guides.npz", allow_pickle=False) as z:
        off, edges = z["road_offsets"], z["edge_xyz"]
        checks["guide_offsets_valid"] = bool(
            len(off) == len(meta["roads"]) + 1
            and off[0] == 0
            and off[-1] == len(edges)
            and (np.diff(off) > 0).all()
        )
        checks["guide_coordinates_finite"] = bool(np.isfinite(edges).all())
    report = dict(
        passed=all(checks.values()),
        checks=checks,
        downward_native_normals=downward_normal_count,
        caveat="Array/observation consistency only; does not certify pavement, connectivity, or layer coverage.",
    )
    (folder / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def write_scene_viewer(folder):
    import base64

    folder = Path(folder)
    meta = load_survey(folder)
    with np.load(folder / "surface_samples.npz", allow_pickle=False) as f:
        mask = f["surface_valid"]
        points = f["surface_xyz"][mask]
        object_ids = f["surface_object_index"][mask]
    with np.load(folder / "road_guides.npz", allow_pickle=False) as f:
        guides = [
            f["edge_xyz"][a:b].tolist()
            for a, b in zip(f["road_offsets"][:-1], f["road_offsets"][1:])
        ]
    data = dict(
        metadata=meta,
        points=base64.b64encode(np.asarray(points, "<f4").tobytes()).decode(),
        object_ids=object_ids.tolist(),
        guides=guides,
        validation=json.loads((folder / "diagnostics.json").read_text()),
    )
    overview_path = folder / "overview_current.json"
    if not overview_path.exists():
        overview_path = folder / "overview.json"
    if overview_path.exists():
        overview = json.loads(overview_path.read_text())
        if overview["source_files_sha256"] != meta["files_sha256"]:
            raise ValueError("Overview sidecar belongs to a different survey")
        raw = folder / overview.get("samples_file", "overview_samples.npz")
        if not raw.resolve().is_relative_to(folder.resolve()):
            raise ValueError("Overview file must be inside the export")
        if hashlib.sha256(raw.read_bytes()).hexdigest() != overview["samples_sha256"]:
            raise ValueError("Overview sample file changed")
        with np.load(raw, allow_pickle=False) as f:
            mask = f["surface_valid"]
            data["overview_points"] = base64.b64encode(
                np.asarray(f["surface_xyz"][mask], "<f4").tobytes()
            ).decode()
            data["overview_normals"] = base64.b64encode(
                np.asarray(f["surface_normal"][mask], "<f4").tobytes()
            ).decode()
        data["overview"] = overview
    navigation_path = folder / "navigation.json"
    if navigation_path.exists():
        navigation = json.loads(navigation_path.read_text())
        if navigation["source_files_sha256"] != meta["files_sha256"]:
            raise ValueError("Navigation sidecar belongs to a different survey")
        data["navigation"] = navigation
        data["navigation_sha256"] = hashlib.sha256(
            navigation_path.read_bytes()
        ).hexdigest()
        from guide_geometry import write_guide_geometry

        guide_path = write_guide_geometry(folder)
        data["navigation_geometry"] = json.loads(guide_path.read_text())
        data["navigation_geometry_sha256"] = hashlib.sha256(
            guide_path.read_bytes()
        ).hexdigest()
        data["navigation_geometry_file"] = guide_path.relative_to(folder).as_posix()
    template = (
        Path(__file__)
        .with_name("scene_viewer.html")
        .read_text()
        .replace("__CSS__", Path(__file__).with_name("workspace.css").read_text())
    )
    planner = Path(__file__).with_name("route_planner.js").read_text()
    editor = Path(__file__).with_name("route_editor.js").read_text()
    template = template.replace("__PLANNER__", planner).replace("__EDITOR__", editor)
    path = folder / "inspect.html"
    path.write_text(
        template.replace(
            "__DATA__", json.dumps(data, separators=(",", ":")).replace("<", "\\u003c")
        )
    )
    return path
