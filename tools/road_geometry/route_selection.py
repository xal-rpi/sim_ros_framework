"""Explicit offline route seam/direction/interval choices, preserving source data."""

import json
import re
from pathlib import Path
import shutil

import numpy as np


from route_schema import digest


def load_route(folder, name=None):
    folder = Path(folder)
    meta = json.loads((folder / "metadata.json").read_text())
    from route_schema import VERSION

    if meta["schema_version"] == VERSION:
        if meta.get("frame") != "local_ENU" or meta.get("units") != "m":
            raise ValueError("Canonical routes must use local_ENU metres")
        routes = [r for r in meta["routes"] if name is None or r["name"] == name]
        if len(routes) != 1:
            raise ValueError("Select exactly one source route")
        route = routes[0]
        path = folder / route["file"]
        if not path.resolve().is_relative_to(folder.resolve()):
            raise ValueError("Route file escapes the export")
        if digest(path) != route["sha256"]:
            raise ValueError("Route arrays changed")
        if (folder / "geometry.npz").exists() and digest(
            folder / "geometry.npz"
        ) != meta["geometry_sha256"]:
            raise ValueError("Source mesh changed")
    elif meta["schema_version"] in ["0.3-track-samples", "0.4-selected-route"]:
        path = folder / "route.npz"
        route = meta.get("route_definition") or dict(
            name=folder.name,
            topology=meta["topology"],
            direction="Saved recipe travel order",
            cross_sections=meta["query_construction"],
            limits=meta["limits_convention"],
        )
        if name is not None and name != route["name"]:
            raise ValueError("Route name does not belong to this export")
        expected = meta["route_sha256"]
        if digest(path) != expected:
            raise ValueError("Route arrays changed")
    elif meta["schema_version"] == "0.1-probe":
        from geometry import load_export

        load_export(folder)
        routes = [r for r in meta["routes"] if name is None or r["name"] == name]
        if len(routes) != 1:
            raise ValueError("Select exactly one source route")
        route = routes[0]
        path = folder / route["file"]
    else:
        raise ValueError(f"Unsupported route schema: {meta['schema_version']}")
    if not path.resolve().is_relative_to(folder.resolve()):
        raise ValueError("Route file escapes the export")
    if route.get("topology") not in ["unknown", "open", "closed"]:
        raise ValueError("Unsupported route topology")
    with np.load(path, allow_pickle=False) as z:
        arrays = dict(z)
    return arrays, meta, route, path


def validate_recipe(folder, recipe):
    if (
        recipe.get("schema_version") != "0.1-route-selection"
        or recipe.get("frame") != "local_ENU"
    ):
        raise ValueError("Expected an ENU route selection")
    if (
        not isinstance(recipe.get("name"), str)
        or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", recipe["name"])
        or recipe["name"].endswith(".")
    ):
        raise ValueError("Use a simple route export name")
    arrays, meta, route, path = load_route(folder, recipe.get("route_name"))
    pair = (recipe.get("source_metadata_sha256"), recipe.get("source_route_sha256"))
    accepted = {(digest(Path(folder) / "metadata.json"), digest(path))}
    accepted.update(
        (p["metadata_sha256"], p["route_sha256"])
        for p in meta.get("notation_previous_hashes", [])
        if p["route_name"] == route["name"]
    )
    if pair not in accepted:
        raise ValueError("Selection belongs to different source geometry")
    selection_rows(arrays, route, recipe)
    return arrays, meta, route


def selection_rows(arrays, route, recipe):
    if type(recipe.get("full_loop")) is not bool:
        raise ValueError("Choose explicit full-loop or open-section mode")
    n = len(arrays["station_id"])
    closed = route["topology"] == "closed"
    body = n - 1 if closed else n
    if closed and (
        not np.isfinite(arrays["center_xyz"][[0, -1]]).all()
        or np.linalg.norm(arrays["center_xyz"][0] - arrays["center_xyz"][-1]) > 0.001
    ):
        raise ValueError("Closed source has a seam gap; no repair performed")
    start, end = recipe.get("start_row"), recipe.get("end_row")
    direction = recipe.get("direction")
    if (
        type(start) is not int
        or not 0 <= start < body
        or direction not in ["forward", "reverse"]
    ):
        raise ValueError("Invalid start row or direction")
    step = 1 if direction == "forward" else -1
    if recipe.get("full_loop") is True:
        if not closed:
            raise ValueError("Only an explicitly closed source can select a full loop")
        rows = (start + step * np.arange(body)) % body
        return np.r_[rows, rows[0]].astype(np.int64)
    if type(end) is not int or not 0 <= end < body:
        raise ValueError("Invalid end row")
    count = ((end - start) * step) % body if closed else (end - start) * step
    if count < 1:
        raise ValueError("Choose a distinct reachable end in the selected direction")
    rows = start + step * np.arange(count + 1)
    return (rows % body if closed else rows).astype(np.int64)


STATION_FIELDS = {
    "station_id",
    "center_xyz",
    "center_valid",
    "reference_xyz",
    "design_reference_xyz",
    "source_ref",
    "design_forward_axis",
    "design_up_axis",
    "horizontal_left_axis",
    "profile_limits_m",
    "chosen_limits_m",
    "physical_limits_m",
    "physical_limits_valid",
    "optimization_limits_valid",
    "profile_right_xyz",
    "profile_left_xyz",
    "center_surface_index",
    "source_station_id",
    "source_row",
}
SECTION_FIELDS = {
    "section_xyz",
    "section_valid",
    "section_normal",
    "section_normal_valid",
    "section_object_index",
    "section_surface_index",
    "section_reference_xyz",
    "section_lateral_m",
    "section_vertex_id",
}


def reorder(arrays, route, recipe):
    from route_schema import (
        STATION_FIELDS as COMMON_STATION,
        SECTION_FIELDS as COMMON_SECTION,
    )

    rows = selection_rows(arrays, route, recipe)
    reverse = recipe["direction"] == "reverse"
    off = arrays["section_offsets"]
    samples = [
        (
            np.arange(off[i], off[i + 1])[::-1]
            if reverse
            else np.arange(off[i], off[i + 1])
        )
        for i in rows
    ]
    selected = np.concatenate(samples)
    out = {
        k: v[rows].copy()
        for k, v in arrays.items()
        if k in STATION_FIELDS | COMMON_STATION
    }
    out.update(
        {
            k: v[selected].copy()
            for k, v in arrays.items()
            if k in SECTION_FIELDS | COMMON_SECTION
        }
    )
    out["source_station_id"] = arrays.get("source_station_id", arrays["station_id"])[
        rows
    ]
    out["source_row"] = rows
    out["section_offsets"] = np.r_[0, np.cumsum([len(s) for s in samples])]
    if "folded_xy_cell" in arrays:
        cells = []
        for a, b in zip(rows[:-1], rows[1:]):
            index = int(b if reverse else a)
            cell = arrays["folded_xy_cell"][index]
            cells.append(cell[::-1] if reverse else cell)
        out["folded_xy_cell"] = np.asarray(cells, bool)
    for name in ["distance_xy_m", "distance_xyz_m", "reference_distance_xyz_m"]:
        if name in arrays and "section_lateral_xy_m" not in arrays:
            out["source_" + name] = arrays[name][rows]
    if reverse:
        for name in ["section_lateral_m", "section_lateral_xy_m"]:
            if name in out:
                out[name] *= -1
        for name in ["profile_limits_m", "chosen_limits_m", "physical_limits_m"] + [
            f"{k}_limits_xy_m"
            for k in ["sampled", "requested", "physical", "optimization"]
        ]:
            if name in out:
                out[name] = -out[name][:, ::-1]
        for name in [
            "sampled_limits_valid",
            "requested_limits_valid",
            "physical_limits_valid",
            "optimization_limits_valid",
        ]:
            if name in out:
                out[name] = out[name][:, ::-1]
        for name in ["design_forward_axis", "horizontal_left_axis"]:
            if name in out:
                out[name] *= -1
        if "profile_left_xyz" in out:
            out["profile_left_xyz"], out["profile_right_xyz"] = (
                out["profile_right_xyz"],
                out["profile_left_xyz"],
            )
    delta = np.diff(out["center_xyz"], axis=0)
    out["distance_xy_m"] = np.r_[0, np.cumsum(np.linalg.norm(delta[:, :2], axis=1))]
    out["distance_xyz_m"] = np.r_[0, np.cumsum(np.linalg.norm(delta, axis=1))]
    if "section_lateral_xy_m" in arrays:
        from route_schema import normalize_arrays

        out = normalize_arrays(
            out, route | dict(topology="closed" if recipe.get("full_loop") else "open")
        )
    return out


def materialize(folder, recipe, output):
    folder, output = Path(folder), Path(output)
    arrays, meta, route = validate_recipe(folder, recipe)
    result = reorder(arrays, route, recipe)
    output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output / "route.npz", **result)
    definition = route | dict(
        name=recipe["name"],
        topology="closed" if recipe.get("full_loop") else "open",
        definition_source="route_selection_recipe",
        direction=recipe["direction"] + " relative to source travel order",
        start_station_id=int(result["station_id"][0]),
        end_station_id=int(result["station_id"][-1]),
        start_xyz=result["center_xyz"][0].tolist(),
        end_xyz=result["center_xyz"][-1].tolist(),
        seam_station_id=(
            int(result["station_id"][0]) if recipe.get("full_loop") else None
        ),
        end_meaning=(
            "Repeated selected seam after one traversal."
            if recipe.get("full_loop")
            else "Explicit selected terminal endpoint."
        ),
    )
    if "acquisition" in route:
        definition["acquisition"] = dict(
            definition_source="route_selection_recipe",
            direction=recipe["direction"] + " relative to immediate source row order",
            cross_sections=route["cross_sections"],
            reference_role=route["acquisition"].get("reference_role"),
            source_ref_columns=route["acquisition"].get("source_ref_columns"),
            source_acquisition=route["acquisition"],
            selection=recipe,
        )
    # Old endpoint/frame residuals describe the source seam, not this selection.
    for key in [
        "stations",
        "file",
        "seam_position_gap_m",
        "seam_forward_angle_deg",
        "seam_up_angle_deg",
    ]:
        definition.pop(key, None)
    for name in ["geometry.npz", "context.npz", "context.json"]:
        if (folder / name).exists():
            shutil.copy2(folder / name, output / name)
    selected_meta = meta | dict(
        schema_version="0.4-selected-route",
        topology=definition["topology"],
        route_definition=definition,
        route_sha256=digest(output / "route.npz"),
        routes=[
            definition | dict(file="route.npz", stations=len(result["station_id"]))
        ],
        query_construction=route.get("cross_sections", meta.get("query_construction")),
        limits_convention=route.get("limits", meta.get("limits_convention")),
        station_convention="station_id and source_station_id retain source identities in chosen travel order. source_row indexes the immediate source. distance_xy_m/distance_xyz_m are recomputed horizontal/3D measured-center chord lengths; missing coordinates propagate NaN. source_*distance arrays retain parent coordinates.",
        selection=dict(
            source_metadata_sha256=recipe["source_metadata_sha256"],
            source_route_sha256=recipe["source_route_sha256"],
            operation=recipe,
            terminal_seam_copy_replaced=route["topology"] == "closed",
            context_scope="Copied parent context, not clipped to the selected interval; local viewer filtering is independent.",
            note="Closed-loop rotation replaces the source terminal seam copy with the selected first row. Interior duplicate joins are preserved. World normals stay unchanged; reverse swaps section order and negates lateral coordinates.",
        ),
    )
    # Old notation aliases apply only to the parent arrays, never this selection.
    selected_meta.pop("notation_previous_hashes", None)
    if meta.get("reference_design"):
        source_recipe = folder / meta["reference_design"]["recipe_file"]
        if digest(source_recipe) != meta["reference_design"]["recipe_sha256"]:
            raise ValueError("Acquisition reference recipe changed")
        shutil.copy2(source_recipe, output / "source_reference_recipe.json")
        selected_meta["reference_design"] = meta["reference_design"] | dict(
            recipe_file="source_reference_recipe.json"
        )
    from route_json import json_values

    (output / "metadata.json").write_text(
        json.dumps(json_values(selected_meta), indent=2, allow_nan=False) + "\n"
    )
    (output / "recipe.json").write_text(json.dumps(recipe, indent=2) + "\n")
    from route_schema import normalize_export

    normalize_export(output, preserve_recipe_hashes=False)
    validate_selected(output)
    from viewer import write_viewer

    write_viewer(output)
    return output


def validate_selected(folder):
    from route_schema import VERSION, validate_routes

    if (
        json.loads((Path(folder) / "metadata.json").read_text())["schema_version"]
        == VERSION
    ):
        return validate_routes(folder)
    arrays, meta, route, _ = load_route(folder)
    off = arrays["section_offsets"]
    ordered = all(
        np.all(np.diff(arrays["section_lateral_m"][a:b]) > 0)
        for a, b in zip(off[:-1], off[1:])
    )
    report = dict(
        stations=len(arrays["station_id"]),
        section_samples=len(arrays["section_xyz"]),
        offsets_valid=bool(
            off[0] == 0
            and off[-1] == len(arrays["section_xyz"])
            and (np.diff(off) > 0).all()
        ),
        right_to_left_ordered=bool(ordered),
        topology=route["topology"],
        endpoint_gap_m=float(
            np.linalg.norm(arrays["center_xyz"][0] - arrays["center_xyz"][-1])
        ),
        valid_surface_samples=int(
            arrays.get(
                "section_valid", np.isfinite(arrays["section_xyz"]).all(axis=1)
            ).sum()
        ),
        folded_xy_cells=(
            int(arrays["folded_xy_cell"].sum()) if "folded_xy_cell" in arrays else None
        ),
        caveat="Ordering/provenance checks, not certification of drivable boundaries or a smooth-surface fit.",
    )
    if not report["offsets_valid"] or not ordered:
        raise ValueError("Invalid selected cross-sections")
    from route_json import json_values

    (Path(folder) / "validation.json").write_text(
        json.dumps(json_values(report), indent=2, allow_nan=False) + "\n"
    )
    return report
