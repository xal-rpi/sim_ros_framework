"""Portable measured-route JSON. No fitting, smoothing, closure or unit changes."""

import hashlib
import json
import math
from pathlib import Path

import numpy as np


def json_values(value):
    if isinstance(value, list):
        return [json_values(v) for v in value]
    if isinstance(value, dict):
        return {k: json_values(v) for k, v in value.items()}
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def export_route_json(folder, output, route_name=None, include_mesh=False):
    folder, output = Path(folder), Path(output)
    if output.exists():
        raise FileExistsError("Choose a new JSON file; original exports are immutable")
    meta = json.loads((folder / "metadata.json").read_text())
    from route_selection import load_route
    from route_schema import VERSION, validate_routes

    if meta["schema_version"] == VERSION:
        validate_routes(folder, write_report=False)
    arrays, meta, route, path = load_route(folder, route_name)
    geometry = None
    if include_mesh:
        if not (folder / "geometry.npz").exists():
            raise ValueError("Native corridor exports have no verified triangle mesh")
        if (
            hashlib.sha256((folder / "geometry.npz").read_bytes()).hexdigest()
            != meta["geometry_sha256"]
        ):
            raise ValueError("Source mesh changed")
        with np.load(folder / "geometry.npz", allow_pickle=False) as saved:
            geometry = dict(saved)
        from geometry import validate_mesh

        validate_mesh(geometry)
    shapes = {name: list(value.shape) for name, value in arrays.items()}
    arrays = {name: value.tolist() for name, value in arrays.items()}
    if "start_station_id" not in route:
        route = route | dict(
            start_station_id=arrays["station_id"][0],
            end_station_id=arrays["station_id"][-1],
            start_xyz=arrays["center_xyz"][0],
            end_xyz=arrays["center_xyz"][-1],
            seam_station_id=(
                arrays["station_id"][0] if route["topology"] == "closed" else None
            ),
            end_meaning=(
                "Last stored sample repeats the chosen starting seam; one traversal."
                if route["topology"] == "closed"
                else "Last stored route sample; no implicit closure."
            ),
        )
    document = dict(
        schema_version="0.1-road-route-json",
        route_schema_version=meta["schema_version"],
        frame="local_ENU",
        units="m",
        array_units="XYZ, lateral, limits and distances: metres; normals: unitless; IDs, offsets and indices: integers; validity: boolean.",
        source=dict(
            asset_sha256=meta.get("asset_sha256"),
            geometry_sha256=meta.get("geometry_sha256"),
            simulator_source=meta.get("source"),
            source_files_sha256=meta.get("source_files_sha256"),
            route_file=path.relative_to(folder).as_posix(),
            route_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            metadata_sha256=hashlib.sha256(
                (folder / "metadata.json").read_bytes()
            ).hexdigest(),
        ),
        route=route,
        arrays=arrays,
        array_shapes=shapes,
        surfaces=meta.get("surfaces", []),
        source_geometry=meta.get("sources"),
        normal_convention=meta.get("normal_convention")
        or (
            "Native Engine.castRay normals, unflipped; attached only when the Engine hit matches the static height within 1 mm."
            if "surface_rule" in meta
            else None
        ),
        unavailable=meta["unavailable"],
        caveat="Geometry observations, not a fitted optimizer surface. Null means unavailable; -1 means unresolved surface. Preserve duplicate seams and explicit topology. No automatic closure, gap bridging or smoothing.",
    )
    if include_mesh:
        document["geometry_arrays"] = {
            name: value.tolist() for name, value in geometry.items()
        }
        document["geometry_array_shapes"] = {
            name: list(value.shape) for name, value in geometry.items()
        }
    # Face normals stay in geometry.npz with triangle connectivity. Sections do
    # not fabricate a unique normal at a mesh edge or interpolate across seams.
    content = json.dumps(json_values(document), indent=2, allow_nan=False) + "\n"
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        stream.write(content)
    return output
