"""One measured-route contract for mesh rows, collision corridors and selections.

Normalization changes notation and derives explicit coordinates only. It never
moves, fits, closes or resamples the observed XYZ geometry. Source-specific
quantities stay under source_* names and their conventions stay in acquisition.
"""

import hashlib
import json
import shutil
from tempfile import TemporaryDirectory
from pathlib import Path

import numpy as np

VERSION = "0.5-road-routes"
STATION_FIELDS = {
    "station_id",
    "source_station_id",
    "source_row",
    "center_xyz",
    "center_valid",
    "reference_xyz",
    "reference_valid",
    "horizontal_left_axis",
    "horizontal_left_valid",
    "center_distance_xy_m",
    "center_distance_xyz_m",
    "reference_distance_xy_m",
    "reference_distance_xyz_m",
    "center_surface_index",
    "source_ref",
    "source_forward_axis",
    "source_up_axis",
    "source_profile_limits_m",
    "source_profile_right_xyz",
    "source_profile_left_xyz",
    "source_reference_distance_xyz_m",
    "source_distance_xy_m",
    "source_distance_xyz_m",
} | {
    f"{kind}_limits{suffix}"
    for kind in ["sampled", "requested", "physical", "optimization"]
    for suffix in ["_xy_m", "_valid"]
}
SECTION_FIELDS = {
    "section_xyz",
    "section_offsets",
    "section_valid",
    "section_reference_xyz",
    "section_lateral_xy_m",
    "section_lateral_valid",
    "section_normal",
    "section_normal_valid",
    "section_surface_index",
    "section_object_index",
    "section_vertex_id",
    "source_section_lateral_m",
} - {"section_offsets"}

LATERAL = (
    "section_lateral_xy_m = dot((section_xyz - center_xyz).xy, "
    "horizontal_left_axis.xy); left positive, right negative, metres. "
    "Horizontal projection, not distance along a banked profile. "
    "Sections retain source XYZ and right-to-left sample order."
)
DISTANCE = (
    "station_id is a source identity; row order is travel order. "
    "center_distance_xy_m/center_distance_xyz_m and "
    "reference_distance_xy_m/reference_distance_xyz_m are separate cumulative "
    "horizontal/3D polyline chord lengths, starting at zero. Duplicate joins "
    "are retained. Missing coordinates propagate NaN. None is fitted chart progress."
)
LIMITS = (
    "All *_limits_xy_m have columns [right, left] in horizontal metres. "
    "sampled = valid observed sample envelope, not guaranteed continuous coverage; "
    "requested = extraction extents excluding extra margin; physical = known usable "
    "pavement edges; optimization = explicit chosen optimization domain. "
    "Unknown limits are NaN with corresponding *_limits_valid false."
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def distance(xyz, dimensions):
    return np.r_[
        0.0, np.cumsum(np.linalg.norm(np.diff(xyz[:, :dimensions], axis=0), axis=1))
    ]


def chord_left(reference, closed):
    """Fallback only: skip duplicate neighbors, never alter the stored points."""
    n = len(reference)
    body = n - 1 if closed and n > 1 else n
    axes = np.full((n, 3), np.nan)
    for i in range(body):
        p = reference[i, :2]
        neighbors = []
        for step in [-1, 1]:
            delta = None
            for count in range(1, body):
                j = i + step * count
                if closed:
                    j %= body
                elif not 0 <= j < body:
                    break
                v = step * (reference[j, :2] - p)
                length = np.linalg.norm(v)
                if np.isfinite(length) and length > 1e-8:
                    delta = v / length
                    break
            if delta is not None:
                neighbors.append(delta)
        if neighbors:
            forward = np.sum(neighbors, axis=0)
            length = np.linalg.norm(forward)
            if length > 1e-8:
                axes[i] = [-forward[1] / length, forward[0] / length, 0.0]
    if closed and n > body:
        axes[-1] = axes[0]
    return axes


def normalize_arrays(arrays, route):
    """Convert old acquisition arrays or recompute a common selected route."""
    a = {k: np.asarray(v).copy() for k, v in arrays.items()}
    canonical = "section_lateral_xy_m" in a
    center = np.asarray(a["center_xyz"], float)
    reference = np.asarray(
        a.get(
            "reference_xyz", a.get("design_reference_xyz", np.full_like(center, np.nan))
        ),
        float,
    )
    n, q = len(center), len(a["section_xyz"])
    out = {
        k: v
        for k, v in a.items()
        if k in STATION_FIELDS | SECTION_FIELDS or k == "folded_xy_cell"
    }
    out.update(
        center_xyz=center, reference_xyz=reference, section_offsets=a["section_offsets"]
    )
    out["center_valid"] = a.get("center_valid", np.isfinite(center).all(axis=1)).astype(
        bool
    )
    out["reference_valid"] = np.isfinite(reference).all(axis=1)
    out["section_valid"] = a.get(
        "section_valid", np.isfinite(a["section_xyz"]).all(axis=1)
    ).astype(bool)
    out.setdefault("source_station_id", a["station_id"].copy())
    out.setdefault("source_row", np.arange(n, dtype=np.int64))
    # Preserve acquisition axes when available. Collision queries and builder
    # profiles have different constructions; a shared projection does not
    # pretend they were sampled on identical planes.
    if "horizontal_left_axis" in a:
        left = np.asarray(a["horizontal_left_axis"], float).copy()
    elif "design_forward_axis" in a:
        f = np.asarray(a["design_forward_axis"], float)
        length = np.linalg.norm(f[:, :2], axis=1)
        left = np.full((n, 3), np.nan)
        valid = np.isfinite(length) & (length > 1e-8)
        left[valid] = np.c_[
            -f[valid, 1] / length[valid],
            f[valid, 0] / length[valid],
            np.zeros(valid.sum()),
        ]
        fallback = chord_left(reference, route["topology"] == "closed")
        left[~valid] = fallback[~valid]
    else:
        left = chord_left(reference, route["topology"] == "closed")
    out["horizontal_left_axis"] = left
    out["horizontal_left_valid"] = np.isfinite(left).all(axis=1)
    rows = np.repeat(np.arange(n), np.diff(out["section_offsets"]))
    lateral = np.einsum(
        "ij,ij->i", a["section_xyz"][:, :2] - center[rows, :2], left[rows, :2]
    )
    out["section_lateral_xy_m"] = lateral
    out["section_lateral_valid"] = np.isfinite(lateral)
    out.setdefault("section_reference_xyz", np.full((q, 3), np.nan))
    out.setdefault("section_normal", np.full((q, 3), np.nan))
    out.setdefault("section_normal_valid", np.zeros(q, bool))
    for name, size in [
        ("center_surface_index", n),
        ("section_surface_index", q),
        ("section_object_index", q),
        ("section_vertex_id", q),
    ]:
        out.setdefault(name, np.full(size, -1, np.int64))
    out.setdefault("source_ref", np.full((n, 3), -1, np.int64))
    for role, points in [("center", center), ("reference", reference)]:
        for dimensions, suffix in [(2, "xy"), (3, "xyz")]:
            out[f"{role}_distance_{suffix}_m"] = distance(points, dimensions)
    if not canonical:
        renames = {
            "section_lateral_m": "source_section_lateral_m",
            "profile_limits_m": "source_profile_limits_m",
            "profile_right_xyz": "source_profile_right_xyz",
            "profile_left_xyz": "source_profile_left_xyz",
            "design_forward_axis": "source_forward_axis",
            "design_up_axis": "source_up_axis",
            "reference_distance_xyz_m": "source_reference_distance_xyz_m",
            "distance_xy_m": "source_distance_xy_m",
            "distance_xyz_m": "source_distance_xyz_m",
        }
        for old, new in renames.items():
            if old in a:
                out[new] = a[old]
    for kind in ["sampled", "requested", "physical", "optimization"]:
        key = f"{kind}_limits_xy_m"
        limits = np.asarray(a.get(key, np.full((n, 2), np.nan)), float).copy()
        valid = np.asarray(
            a.get(f"{kind}_limits_valid", np.zeros((n, 2), bool)), bool
        ).copy()
        if kind == "sampled":
            limits[:] = np.nan
            for row, (start, end) in enumerate(
                zip(out["section_offsets"][:-1], out["section_offsets"][1:])
            ):
                mask = (
                    out["section_valid"][start:end]
                    & out["section_lateral_valid"][start:end]
                )
                e = lateral[start:end][mask]
                if len(e):
                    limits[row] = [e.min(), e.max()]
            valid = np.isfinite(limits)
        elif not canonical and kind == "requested" and "chosen_limits_m" in a:
            # This legacy field exists only on horizontal collision queries.
            limits = np.asarray(a["chosen_limits_m"], float)
            valid = np.isfinite(limits)
        elif not canonical and kind == "physical" and "physical_limits_m" in a:
            limits = np.asarray(a["physical_limits_m"], float)
            valid = a.get("physical_limits_valid", np.isfinite(limits))
        limits[~valid] = np.nan
        out[key], out[f"{kind}_limits_valid"] = limits, valid
    return out


def normalize_definition(route, arrays, file, acquisition):
    """Every route has the same definition keys; differences are provenance."""
    closed = route["topology"] == "closed"
    return dict(
        name=route["name"],
        file=file,
        stations=len(arrays["station_id"]),
        topology=route["topology"],
        direction="array_row_order",
        start_station_id=int(arrays["station_id"][0]),
        end_station_id=int(arrays["station_id"][-1]),
        start_xyz=arrays["center_xyz"][0].tolist(),
        end_xyz=arrays["center_xyz"][-1].tolist(),
        seam_station_id=int(arrays["station_id"][0]) if closed else None,
        end_meaning=(
            "Repeated starting seam after one traversal."
            if closed
            else "Last stored sample; no implicit closure."
        ),
        cross_sections=acquisition.get("cross_sections"),
        lateral=LATERAL,
        limits=LIMITS,
        station_convention=DISTANCE,
        acquisition=acquisition,
    )


def normalize_export(folder, *, preserve_recipe_hashes=True):
    """Finish an unpublished acquisition; use migrate_export for saved bundles.

    Only actual previous hash pairs are accepted for saved row-selection recipes.
    They remain valid because normalization preserves station and section XYZ.
    Geometry, recipes, collision evidence and context files are untouched.
    """
    folder = Path(folder)
    meta = json.loads((folder / "metadata.json").read_text())
    if meta["schema_version"] == VERSION:
        return meta
    if meta["schema_version"] not in [
        "0.1-probe",
        "0.3-track-samples",
        "0.4-selected-route",
    ]:
        raise ValueError(
            "Expected a route export, not a whole-map survey or selection recipe"
        )
    if (folder / "geometry.npz").exists() and digest(folder / "geometry.npz") != meta[
        "geometry_sha256"
    ]:
        raise ValueError("Source mesh changed before notation migration")
    old_hash = digest(folder / "metadata.json")
    routes = meta.get("routes") or [
        meta.get("route_definition")
        or dict(
            name=folder.name,
            topology=meta["topology"],
            cross_sections=meta.get("query_construction"),
            direction="Saved recipe travel order",
            limits=meta.get("limits_convention"),
        )
    ]
    definitions, aliases = [], list(meta.get("notation_previous_hashes", []))
    prepared = {}
    for route in routes:
        file = route.get("file", "route.npz")
        path = folder / file
        if not path.resolve().is_relative_to(folder.resolve()):
            raise ValueError("Route file escapes the export")
        old_route_hash = digest(path)
        if file == "route.npz" and meta.get("route_sha256") != old_route_hash:
            raise ValueError("Route arrays changed before notation migration")
        with np.load(path, allow_pickle=False) as z:
            a = dict(z)
        acquisition = route.get("acquisition") or dict(
            schema_version=meta["schema_version"],
            definition_source=route.get("definition_source", "waypoint_recipe"),
            direction=route.get("direction"),
            cross_sections=route.get("cross_sections", meta.get("query_construction")),
            lateral=route.get(
                "lateral", "Horizontal query offsets, left positive; metres"
            ),
            limits=route.get("limits", meta.get("limits_convention")),
            reference_role=(
                "native_builder_design_curve"
                if "design_reference_xyz" in a
                else (
                    "authored_or_navigation_reference"
                    if "reference_xyz" in a
                    else "unknown"
                )
            ),
            source_ref_columns=(
                ["subtrack", "piece", "evaluated_point"] if "source_ref" in a else None
            ),
            reference_design=meta.get("reference_design"),
            source_definition=route,
        )
        b = normalize_arrays(a, route)
        definition = normalize_definition(route, b, file, acquisition)
        # Serialize every array and metadata before replacing acquisition files.
        # Saved exports are migrated in a separate directory by migrate_export.
        from io import BytesIO

        buffer = BytesIO()
        np.savez_compressed(buffer, **b)
        prepared[path] = buffer.getvalue()
        definition["sha256"] = hashlib.sha256(prepared[path]).hexdigest()
        definitions.append(definition)
        if preserve_recipe_hashes:
            aliases.append(
                dict(
                    metadata_sha256=old_hash,
                    route_sha256=old_route_hash,
                    route_name=route["name"],
                )
            )
    meta.update(
        schema_version=VERSION,
        routes=definitions,
        frame="local_ENU",
        units="m",
        axes=["project_east", "project_north", "up"],
        handedness="right",
        geographic_datum=meta.get("geographic_datum"),
        T_enu_from_native=meta.get("T_enu_from_native", np.eye(4).tolist()),
        station_convention=DISTANCE,
        lateral_convention=LATERAL,
        limits_convention=LIMITS,
        surface_objects=meta.get("surface_objects", []),
        sources=meta.get("sources", []),
        surfaces=meta.get("surfaces", []),
        notation_previous_hashes=aliases,
    )
    meta.setdefault("scope", "Selected route observations; not a complete map")
    meta.setdefault("complete_scene", False)
    meta["normal_convention"] = dict(
        section="World ENU unit normals, unchanged from the matching native Engine hit; unknown NaN/false. No facet averaging.",
        face="Native mesh geometric face normals when available; see geometry.npz. Source winding retained.",
    )
    # All consumers use routes, including single-route measured corridors.
    for key in ["route_definition", "topology", "query_construction", "route_sha256"]:
        meta.pop(key, None)
    from route_json import json_values

    prepared[folder / "metadata.json"] = (
        json.dumps(json_values(meta), indent=2, allow_nan=False) + "\n"
    ).encode()
    for path, content in prepared.items():
        staged = path.with_name(path.name + ".notation.tmp")
        staged.write_bytes(content)
        staged.replace(path)
    return meta


def validate_routes(folder, *, write_report=True):
    """Common geometric checks; does not certify a downstream surface chart."""
    from route_selection import load_route

    folder = Path(folder)
    meta = json.loads((folder / "metadata.json").read_text())
    if meta.get("schema_version") != VERSION or not meta.get("routes"):
        raise ValueError(
            "Expected canonical routes; normalize legacy exports explicitly"
        )
    reports = []
    for definition in meta["routes"]:
        a, _, route, _ = load_route(folder, definition["name"])
        validate_array_shapes(a)
        off = a["section_offsets"]
        xyz = a["section_xyz"]
        n = len(a["station_id"])
        valid = a["section_valid"]
        nv = a["section_normal_valid"]
        offsets_ok = (
            off.shape == (n + 1,)
            and off[0] == 0
            and off[-1] == len(xyz)
            and (np.diff(off) > 0).all()
        )
        ordered = all(
            np.all(
                np.diff(
                    a["section_lateral_xy_m"][start:end][
                        a["section_lateral_valid"][start:end]
                    ]
                )
                > 0
            )
            for start, end in zip(off[:-1], off[1:])
        )
        checks = dict(
            offsets_valid=bool(offsets_ok),
            sections_ordered=bool(ordered),
            valid_surface_finite=bool(np.isfinite(xyz[valid]).all()),
            valid_center_finite=bool(
                np.isfinite(a["center_xyz"][a["center_valid"]]).all()
            ),
            valid_normal_finite=bool(np.isfinite(a["section_normal"][nv]).all()),
            valid_normals_unit=bool(
                np.allclose(
                    np.linalg.norm(a["section_normal"][nv], axis=1),
                    1.0,
                    atol=1e-5,
                    rtol=0,
                )
            ),
        )
        checks["horizontal_left_axes_valid"] = bool(
            np.allclose(
                np.linalg.norm(
                    a["horizontal_left_axis"][a["horizontal_left_valid"]], axis=1
                ),
                1.0,
                atol=1e-6,
            )
            and np.allclose(
                a["horizontal_left_axis"][a["horizontal_left_valid"], 2], 0.0, atol=1e-8
            )
        )
        for kind in ["sampled", "requested", "physical", "optimization"]:
            limits, lv = a[f"{kind}_limits_xy_m"], a[f"{kind}_limits_valid"]
            checks[f"{kind}_limits_valid"] = bool(
                limits.shape == (n, 2)
                and lv.shape == (n, 2)
                and np.isfinite(limits[lv]).all()
                and np.isnan(limits[~lv]).all()
                and (limits[lv.all(axis=1), 1] >= limits[lv.all(axis=1), 0]).all()
            )
        from surface_properties import validate_surface_indices

        checks["surface_indices_valid"] = validate_surface_indices(
            a["section_surface_index"], meta["surfaces"], valid
        )
        checks["center_surface_indices_valid"] = validate_surface_indices(
            a["center_surface_index"], meta["surfaces"], a["center_valid"]
        )
        gap = np.linalg.norm(a["center_xyz"][0] - a["center_xyz"][-1])
        checks["closed_seam_valid"] = bool(
            route["topology"] != "closed" or (np.isfinite(gap) and gap <= 0.001)
        )
        ds = np.linalg.norm(np.diff(a["center_xyz"], axis=0), axis=1)
        jumps = sum(
            int((np.abs(np.diff(xyz[start:end, 2])) > 0.3).sum())
            for start, end in zip(off[:-1], off[1:])
        )
        widths = np.diff(a["sampled_limits_xy_m"], axis=1).ravel()
        center_mask = a["center_valid"] & a["reference_valid"]
        missing_rows = np.array(
            [
                i
                for i, (start, end) in enumerate(zip(off[:-1], off[1:]))
                if not valid[start:end].all()
            ],
            int,
        )
        folded_rows = (
            np.flatnonzero(a["folded_xy_cell"].any(axis=1))
            if "folded_xy_cell" in a
            else np.array([], int)
        )
        report = dict(
            name=route["name"],
            array_checks_passed=all(checks.values()),
            checks=checks,
            stations=n,
            section_samples=len(xyz),
            valid_surface_hits=int(valid.sum()),
            valid_normals=int(nv.sum()),
            missing_center_stations=int((~a["center_valid"]).sum()),
            duplicate_adjacent_stations=int((ds < 1e-8).sum()),
            max_spacing_m=float(np.nanmax(ds)) if np.isfinite(ds).any() else None,
            sampled_horizontal_width_min_m=(
                float(np.nanmin(widths)) if np.isfinite(widths).any() else None
            ),
            lateral_height_jumps_over_0_3m=jumps,
            reference_to_measured_center_abs_p95_m=(
                float(
                    np.percentile(
                        abs(
                            a["center_xyz"][center_mask, 2]
                            - a["reference_xyz"][center_mask, 2]
                        ),
                        95,
                    )
                )
                if center_mask.any()
                else None
            ),
            downward_section_normals=int((a["section_normal"][nv, 2] < 0).sum()),
            inspection_station_rows=np.unique(
                np.r_[missing_rows, folded_rows, folded_rows + 1]
            ).tolist(),
            endpoint_gap_m=float(gap),
            topology=route["topology"],
            folded_xy_cells=(
                int(a["folded_xy_cell"].sum()) if "folded_xy_cell" in a else None
            ),
            caveat="Observed-polyline checks only. A smooth fitted chart can still fold or overshoot; validate that in trajgen.",
        )
        reports.append(report)
        if not report["array_checks_passed"]:
            raise ValueError(f"Invalid common route: {report}")
    result = dict(
        schema_version=VERSION, routes=reports, unavailable=meta["unavailable"]
    )
    if (folder / "geometry.npz").exists():
        with np.load(folder / "geometry.npz", allow_pickle=False) as g:
            v, f, normal = g["vertices_xyz"], g["triangles"], g["face_normal"]
            from geometry import validate_mesh

            result["mesh"] = validate_mesh(dict(g))
    recipe_path = folder / "recipe.json"
    if recipe_path.exists():
        recipe = json.loads(recipe_path.read_text())
        roles = recipe.get("source_reference_roles", [])
        points = np.asarray(recipe.get("source_reference_xyz", []), float)
        if len(points) == len(roles) + 1:
            lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
            arc = np.r_[0.0, np.cumsum(lengths)]
            edits = [
                edit
                for operation in recipe.get("reference_operations", [])
                for edit in operation["transitions"]
            ]
            replaced = {i for edit in edits for i in edit["connector_segments"]}
            result["reference_transitions"] = edits
            result["reference_warnings"] = [
                dict(
                    source_segment=i,
                    role=role,
                    source_reference_interval_xyz_m=arc[i : i + 2].tolist(),
                    length_xyz_m=float(lengths[i]),
                    warning="Guide/graph connector: inspect heading continuity before fitting. Not a measured physical road turn; no repair applied.",
                )
                for i, role in enumerate(roles)
                if role == "native_graph_endpoint_connector" and i not in replaced
            ]
    from route_json import json_values

    if write_report:
        (folder / "validation.json").write_text(
            json.dumps(json_values(result), indent=2, allow_nan=False) + "\n"
        )
    return result


def migrate_export(folder, output):
    """Publish a validated, normalized copy; never rewrite the source or manifest.

    output must be a new directory outside the source. A failed conversion leaves
    neither a partial destination nor changes to the saved acquisition/recipes.
    """
    folder, output = Path(folder).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError("Choose a new migration destination")
    if output.is_relative_to(folder) or folder.is_relative_to(output):
        raise ValueError("Migration destination must be separate from the source")
    if any(path.is_symlink() for path in folder.rglob("*")):
        raise ValueError("Migration does not copy symlinks")
    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix=".normalize-", dir=output.parent) as temporary:
        staging = Path(temporary) / folder.name
        shutil.copytree(folder, staging)
        road = staging / "road.json"
        options = None
        if road.exists():
            previous = json.loads(road.read_text())
            options = (previous["route"]["name"], "geometry_arrays" in previous)
            road.unlink()
        normalize_export(staging)
        report = validate_routes(staging)
        if options:
            from route_json import export_route_json

            export_route_json(staging, road, *options)
        from viewer import write_viewer

        write_viewer(staging)
        staging.rename(output)
    return report


def validate_array_shapes(arrays):
    """Check canonical numeric shapes and mask/index types before geometric use."""
    required = STATION_FIELDS - {
        "source_forward_axis",
        "source_up_axis",
        "source_profile_limits_m",
        "source_profile_right_xyz",
        "source_profile_left_xyz",
        "source_reference_distance_xyz_m",
        "source_distance_xy_m",
        "source_distance_xyz_m",
    }
    required |= SECTION_FIELDS - {"source_section_lateral_m"}
    required.add("section_offsets")
    if required - arrays.keys():
        raise ValueError(f"Missing route arrays: {sorted(required - arrays.keys())}")
    stations, points = arrays["station_id"], arrays["section_xyz"]
    if (
        stations.ndim != 1
        or len(stations) < 2
        or points.ndim != 2
        or points.shape[1] != 3
    ):
        raise ValueError("Route needs station IDs (N,) and section XYZ (Q,3)")
    n, q = len(stations), len(points)
    vectors = {
        "center_xyz",
        "reference_xyz",
        "horizontal_left_axis",
        "source_ref",
        "section_xyz",
        "section_reference_xyz",
        "section_normal",
    }
    integers = {
        "station_id",
        "source_station_id",
        "source_row",
        "source_ref",
        "section_surface_index",
        "center_surface_index",
        "section_object_index",
        "section_vertex_id",
    }
    for key in required - {"section_offsets"}:
        a = arrays[key]
        size = n if key in STATION_FIELDS else q
        shape = (
            (size, 3) if key in vectors else (size, 2) if "_limits" in key else (size,)
        )
        if a.shape != shape:
            raise ValueError(f"Invalid route array shape for {key}: expected {shape}")
        kind = "b" if key.endswith("_valid") else "iu" if key in integers else "fiu"
        if a.dtype.kind not in kind:
            raise ValueError(f"Invalid route array type for {key}")
    off = arrays["section_offsets"]
    if (
        off.dtype.kind not in "iu"
        or off.shape != (n + 1,)
        or off[0] != 0
        or off[-1] != q
        or (off[1:] <= off[:-1]).any()
    ):
        raise ValueError("Invalid section offsets")
