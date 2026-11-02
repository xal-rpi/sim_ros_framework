"""One saved-route inspector for evaluated meshes and measured native corridors."""

import base64
import json
from pathlib import Path
import numpy as np
from route_selection import digest, load_route
from route_json import json_values


def write_viewer(folder):
    folder = Path(folder)
    meta = json.loads((folder / "metadata.json").read_text())
    names = (
        [r["name"] for r in meta["routes"]]
        if not (folder / "route.npz").exists()
        else [None]
    )
    routes = []
    extent = []
    for name in names:
        a, _, route, path = load_route(folder, name)
        valid = a.get("section_valid", np.isfinite(a["section_xyz"]).all(axis=1))
        center_valid = a.get("center_valid", np.isfinite(a["center_xyz"]).all(axis=1))
        reference = a.get(
            "reference_xyz", a.get("design_reference_xyz", a["center_xyz"])
        )
        points = a["section_xyz"][valid]
        extent.extend(points.tolist() if len(points) else reference.tolist())
        routes.append(
            dict(
                definition=route,
                sha256=digest(path),
                station=a["station_id"].tolist(),
                center=np.where(
                    center_valid[:, None], a["center_xyz"], np.nan
                ).tolist(),
                reference=reference.tolist(),
                sections=a["section_xyz"].tolist(),
                offsets=a["section_offsets"].tolist(),
                lateral=a.get(
                    "section_lateral_xy_m", a.get("section_lateral_m")
                ).tolist(),
                valid=valid.tolist(),
                normal=a.get(
                    "section_normal", np.full_like(a["section_xyz"], np.nan)
                ).tolist(),
                normal_valid=a.get(
                    "section_normal_valid", np.zeros(len(valid), bool)
                ).tolist(),
                surface=a.get(
                    "center_surface_index", np.full(len(a["station_id"]), -1)
                ).tolist(),
            )
        )

    def binary(a, dtype="<f4"):
        return base64.b64encode(np.asarray(a, dtype).tobytes()).decode()

    mesh = None
    if (folder / "geometry.npz").exists():
        if digest(folder / "geometry.npz") != meta["geometry_sha256"]:
            raise ValueError("Mesh hash changed")
        with np.load(folder / "geometry.npz", allow_pickle=False) as z:
            mesh = dict(
                vertices=binary(z["vertices_xyz"]),
                triangles=binary(z["triangles"], "<u4"),
                normals=binary(z["face_normal"]),
            )
    context = None
    context_info = None
    if (folder / "context.npz").exists():
        context_info = json.loads((folder / "context.json").read_text())
        if digest(folder / "context.npz") != context_info["sha256"]:
            raise ValueError("Context hash changed")
        with np.load(folder / "context.npz", allow_pickle=False) as z:
            context = binary(z["xyz"])
    extent = np.asarray(extent, float)
    extent = extent[np.isfinite(extent).all(axis=1)]
    if not len(extent):
        raise ValueError("Route has no finite reference or measured extent")
    data = dict(
        schema_version="0.1-route-viewer",
        metadata_sha256=digest(folder / "metadata.json"),
        notation_previous_hashes=meta.get("notation_previous_hashes", []),
        title=(
            routes[0]["definition"]["name"]
            if len(routes) == 1 and (folder / "route.npz").exists()
            else meta["source"].get("asset_name", folder.name)
        ),
        routes=routes,
        mesh=mesh,
        context=context,
        context_info=context_info,
        low=extent.min(axis=0).tolist(),
        high=extent.max(axis=0).tolist(),
        surfaces=meta.get("surfaces", []),
        report=(
            json.loads((folder / "validation.json").read_text())
            if (folder / "validation.json").exists()
            else {}
        ),
    )
    template = Path(__file__).with_name("route_viewer.html").read_text()
    for tag, file in [
        ("__CSS__", "workspace.css"),
        ("__SELECTION__", "route_selection.js"),
        ("__VIEWER__", "route_viewer.js"),
    ]:
        template = template.replace(tag, Path(__file__).with_name(file).read_text())
    path = folder / "inspect.html"
    path.write_text(
        template.replace(
            "__DATA__",
            json.dumps(
                json_values(data), separators=(",", ":"), allow_nan=False
            ).replace("<", "\\u003c"),
        )
    )
    return path
