"""Array-only packing and inspection of evaluated BeamNG mesh captures."""

import hashlib
import json
from pathlib import Path

import numpy as np
from surface_properties import intern_surface, surface_record, validate_surface_indices


def pack(segments):
    vertices, triangles, normals, sources, table = [], [], [], [], []
    vertex_count = 0
    for segment in segments:
        for mesh in segment["meshes"]:
            v = np.asarray(mesh["vertices"], dtype=float).reshape(-1, 3)
            f = np.asarray(mesh["triangles"], dtype=np.int64).reshape(-1, 3)
            if not (len(f) and f.min() >= 0 and (f.max() < len(v))):
                raise ValueError("Native triangles must contain valid vertex indices")
            raw = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
            length = np.linalg.norm(raw, axis=1)
            n = raw / np.maximum(length[:, None], 1e-30)
            # Preserve triangles/winding. Separately orient geometric normals
            # to the mesher's normal convention, which can differ from winding.
            sn = np.asarray(mesh["normals"], float).reshape(-1, 3)
            corners = np.asarray(mesh["normalCorners"], np.int64).reshape(-1, 3)
            if not len(corners) or corners.min() < 0 or corners.max() >= len(sn):
                raise ValueError("Native normal corners contain invalid normal indices")
            shading = sn[corners].mean(axis=1)
            n[(n * shading).sum(axis=1) < 0] *= -1
            sid = len(table)
            table.append(
                {
                    key: segment[key]
                    for key in [
                        "piece",
                        "subtrack",
                        "quality",
                        "centerMesh",
                        "objectId",
                        "objectName",
                    ]
                }
                | {
                    "submesh": mesh["index"],
                    "tag": mesh.get("tag"),
                    "material": mesh.get("material"),
                    "vertex_start": vertex_count,
                    "surface_properties": mesh.get("surfaceProperties"),
                    "vertex_count": len(v),
                    "triangle_count": len(f),
                }
            )
            vertices.append(v)
            triangles.append(f + vertex_count)
            normals.append(n)
            sources.append(np.full(len(f), sid, dtype=np.int64))
            vertex_count += len(v)
    return (
        dict(
            vertices_xyz=np.vstack(vertices),
            triangles=np.vstack(triangles),
            face_normal=np.vstack(normals),
            triangle_source_id=np.concatenate(sources),
        ),
        table,
    )


class HeightIndex:
    """Uniform XY bins for vertical triangle intersections, retaining all layers."""

    def __init__(self, geometry, cell=5.0):
        self.tri = geometry["vertices_xyz"][geometry["triangles"]]
        self.normals = geometry["face_normal"]
        self.cell = cell
        self.bins = {}
        for i, tri in enumerate(self.tri):
            low = np.floor(tri[:, :2].min(axis=0) / cell).astype(int)
            high = np.floor(tri[:, :2].max(axis=0) / cell).astype(int)
            for x in range(low[0], high[0] + 1):
                for y in range(low[1], high[1] + 1):
                    self.bins.setdefault((x, y), []).append(i)

    def hits(self, x, y, *, upper=True):
        ids = np.asarray(
            self.bins.get(
                (int(np.floor(x / self.cell)), int(np.floor(y / self.cell))), []
            ),
            int,
        )
        if not len(ids):
            return ids, np.empty(0)
        a, b, c = self.tri[ids].transpose(1, 0, 2)
        u, v = b - a, c - a
        det = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
        good = abs(det) > 1e-12
        if upper:
            good &= self.normals[ids, 2] > 0.25
        ids, a, u, v, det = ids[good], a[good], u[good], v[good], det[good]
        dx, dy = x - a[:, 0], y - a[:, 1]
        s = (dx * v[:, 1] - dy * v[:, 0]) / det
        t = (u[:, 0] * dy - u[:, 1] * dx) / det
        inside = (s >= -1e-8) & (t >= -1e-8) & (s + t <= 1 + 1e-8)
        return ids[inside], (a[:, 2] + s * u[:, 2] + t * v[:, 2])[inside]

    def height(self, point, *, above=0.3, below=0.3):
        ids, z = self.hits(point[0], point[1])
        mask = (z <= point[2] + above) & (z >= point[2] - below)
        ids, z = ids[mask], z[mask]
        if not len(z):
            return None
        selected = np.argmax(z)
        return float(z[selected]), int(ids[selected])


def route_endpoints(center, forward, up, defaults, info):
    """An explicit owned-asset route choice; never infer closure from proximity.

    Native generator axes check seam orientation independently of the coarse
    first/last mesh chords. Chords on a curved loop need not point identically.
    """
    topology = defaults.get("topology", "unknown")
    if topology not in ["unknown", "open", "closed"]:
        raise ValueError("Unknown route-default topology")
    if defaults and (
        defaults.get("start") != "first_evaluated_station"
        or defaults.get("direction") != "native_builder_order"
    ):
        raise ValueError(
            "Unsupported managed route start/direction; use an explicit selection recipe"
        )
    gap = float(np.linalg.norm(center[-1] - center[0]))

    def angle(a, b):
        if (
            not np.isfinite([a, b]).all()
            or min(np.linalg.norm(a), np.linalg.norm(b)) < 0.5
        ):
            return None
        return float(
            np.degrees(
                np.arccos(
                    np.clip(
                        np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1
                    )
                )
            )
        )

    forward_gap, up_gap = angle(forward[0], forward[-1]), angle(up[0], up[-1])
    if topology == "closed":
        if not (info.get("declaredConnected") and info.get("evaluatedClosed")):
            raise ValueError(
                "Explicit closed route requires declared and evaluated builder closure"
            )
        if (
            gap > 0.001
            or forward_gap is None
            or up_gap is None
            or max(forward_gap, up_gap) > 0.1
        ):
            raise ValueError(
                "Explicit closed route has a position/orientation seam mismatch; no repair performed"
            )
    return dict(
        topology=topology,
        definition_source="environment_manifest.route_defaults" if defaults else None,
        start_station_id=0,
        end_station_id=len(center) - 1,
        start_xyz=center[0].tolist(),
        end_xyz=center[-1].tolist(),
        seam_station_id=0 if topology == "closed" else None,
        end_meaning=(
            "Last stored sample repeats the starting seam; one traversal, not a terminal destination."
            if topology == "closed"
            else "Last stored route sample; unknown topology is not permission to join endpoints."
        ),
        seam_position_gap_m=gap,
        seam_forward_angle_deg=forward_gap,
        seam_up_angle_deg=up_gap,
    )


def write_export(output, info, segments, *, lua_hash, asset_hash, asset_bytes):
    output = Path(output)
    # Never overwrite a previous export with another scene silently.
    output.mkdir(parents=True, exist_ok=False)
    geometry, table = pack(segments)
    surfaces, surface_lookup = [], {}
    for entry in table:
        entry["surface_index"] = intern_surface(
            surfaces,
            surface_lookup,
            surface_record(
                entry["surface_properties"]
                or dict(
                    kind="managed_trackbuilder",
                    material_name=entry["material"],
                    resolution_source="mesh_material_label_only",
                ),
                dict(
                    engineObject=entry["objectName"],
                    engineObjectId=entry["objectId"],
                    engineObjectClass="ProceduralMesh",
                ),
            ),
        )
    geometry["triangle_surface_index"] = np.array(
        [table[i]["surface_index"] for i in geometry["triangle_source_id"]], np.int64
    )
    np.savez_compressed(output / "geometry.npz", **geometry)
    (output / "source_asset.json").write_bytes(asset_bytes)
    routes = []
    (output / "routes").mkdir()
    for subtrack in sorted({s["subtrack"] for s in segments}):
        center, ref, source, sections, lateral, offsets = [], [], [], [], [], [0]
        widths = []
        section_vertex_ids = []
        right = []
        left_edge = []
        center_surface_ids = []
        section_surface_ids = []
        forward_axes = []
        up_axes = []
        for seg in segments:
            if seg["subtrack"] != subtrack:
                continue
            center_mesh = next(m for m in seg["meshes"] if m.get("tag") == "center")
            points = seg["points"]
            mesh_vertices = np.asarray(center_mesh["vertices"], float)
            if not points or len(mesh_vertices) % len(points) != 0:
                raise ValueError(
                    "Native center mesh does not contain uniform profile rows"
                )
            row_count = len(mesh_vertices) // len(points)
            sid = next(
                i
                for i, s in enumerate(table)
                if s["piece"] == seg["piece"]
                and s["subtrack"] == subtrack
                and s["submesh"] == center_mesh["index"]
            )
            mask = (geometry["triangle_source_id"] == sid) & (
                geometry["face_normal"][:, 2] > 0.25
            )
            top_vertices = set(
                (geometry["triangles"][mask] - table[sid]["vertex_start"])
                .ravel()
                .tolist()
            )
            design = np.asarray([p["xyz"] for p in points], float)
            for row, p in enumerate(points):
                tangent = (
                    design[min(row + 1, len(design) - 1)] - design[max(row - 1, 0)]
                )
                tangent /= np.linalg.norm(tangent)
                up = np.asarray(p["up"], float)
                if up[2] < 0:
                    up = -up
                left = np.cross(up, tangent)
                left /= np.linalg.norm(left)
                row_ids = [
                    j
                    for j in range(row * row_count, (row + 1) * row_count)
                    if j in top_vertices
                ]
                profile = mesh_vertices[row_ids]
                e = (profile - design[row]) @ left
                order = np.argsort(e)
                e, profile = e[order], profile[order]
                if not (len(e) >= 2 and np.all(np.diff(e) > 1e-08)):
                    raise ValueError("Ambiguous/nonmonotone section")
                # Physical center is sampled from the actual upper profile,
                # not the elevated generator design curve.
                c = np.array([np.interp(0, e, profile[:, axis]) for axis in range(3)])
                e = (profile - c) @ left
                center.append(c)
                ref.append(design[row])
                source.append([subtrack, seg["piece"], p["index"]])
                sections.extend(profile)
                lateral.extend(e)
                offsets.append(len(sections))
                section_vertex_ids.extend(
                    (np.asarray(row_ids)[order] + table[sid]["vertex_start"]).tolist()
                )
                right.append(profile[0])
                left_edge.append(profile[-1])
                widths.append([e[0], e[-1]])
                forward_axes.append(p.get("forward", [np.nan] * 3))
                up_axes.append(p["up"])
                center_surface_ids.append(table[sid]["surface_index"])
                section_surface_ids.extend([table[sid]["surface_index"]] * len(profile))
        center = np.asarray(center)
        n = len(center)
        delta = np.diff(center, axis=0)
        arrays = dict(
            station_id=np.arange(n, dtype=np.int64),
            center_xyz=center,
            design_reference_xyz=np.asarray(ref),
            source_ref=np.asarray(source, np.int64),
            design_forward_axis=np.asarray(forward_axes),
            design_up_axis=np.asarray(up_axes),
            distance_xy_m=np.r_[0, np.cumsum(np.linalg.norm(delta[:, :2], axis=1))],
            distance_xyz_m=np.r_[0, np.cumsum(np.linalg.norm(delta, axis=1))],
            section_xyz=np.asarray(sections),
            section_offsets=np.asarray(offsets, np.int64),
            section_lateral_m=np.asarray(lateral),
            profile_limits_m=np.asarray(widths),
            section_vertex_id=np.asarray(section_vertex_ids, np.int64),
            center_surface_index=np.asarray(center_surface_ids, np.int64),
            section_surface_index=np.asarray(section_surface_ids, np.int64),
            profile_right_xyz=np.asarray(right),
            profile_left_xyz=np.asarray(left_edge),
            physical_limits_valid=np.zeros((n, 2), bool),
            optimization_limits_valid=np.zeros((n, 2), bool),
        )
        route = f"subtrack_{subtrack}"
        endpoints = route_endpoints(
            center,
            np.asarray(forward_axes),
            np.asarray(up_axes),
            info.get("route_defaults", {}),
            info,
        )
        np.savez_compressed(output / "routes" / f"{route}.npz", **arrays)
        routes.append(
            {
                "name": route,
                "file": f"routes/{route}.npz",
                "stations": n,
                **endpoints,
                "declared_connected": info.get("declaredConnected"),
                "evaluated_closed": info.get("evaluatedClosed"),
                "direction": "Track Builder evaluated point order",
                "cross_sections": "Native mesher station rows; upper center-profile vertices, ordered right to left",
                "lateral": "Signed Euclidean projection on normalize(up cross travel_tangent), positive left; metres",
                "design_axes": "Native generator forward/up axes in world ENU, not collision normals. Used to check seam orientation.",
                "limits": "profile_limits are the generated center strip, not certified usable pavement",
                "duplicate_seam_stations": "Preserved; no welding, smoothing, endpoint snapping or closure",
            }
        )
    metadata = dict(
        schema_version="0.1-probe",
        frame="local_ENU",
        units="m",
        axes=["project_east", "project_north", "up"],
        handedness="right",
        geographic_datum=None,
        T_enu_from_native=np.eye(4).tolist(),
        scope="single managed Track Builder asset",
        complete_scene=False,
        source=info,
        asset_sha256=asset_hash,
        capture_lua_sha256=lua_hash,
        geometry_sha256=hashlib.sha256(
            (output / "geometry.npz").read_bytes()
        ).hexdigest(),
        sources=table,
        routes=routes,
        surfaces=surfaces,
        normal_convention="Geometric face normal oriented to native shading normals; raw triangle winding preserved",
        unavailable=[
            "certified usable edges/holes",
            "grip",
            "speed restrictions",
            "obstacle semantics",
            "native scene mesh readback",
            "Track Builder intersections",
            "general native-map export",
        ],
        settings={
            "upper_candidate_min_normal_z": 0.25,
            "capture": "evaluated compiler payload, final revision per piece",
        },
    )
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    from route_schema import normalize_export

    metadata = normalize_export(output, preserve_recipe_hashes=False)
    return geometry, metadata


def load_export(folder):
    folder = Path(folder)
    metadata = json.loads((folder / "metadata.json").read_text())
    from route_schema import VERSION

    if metadata.get("schema_version") not in ["0.1-probe", VERSION]:
        raise ValueError("Unsupported mesh export schema")
    if not (
        hashlib.sha256((folder / "geometry.npz").read_bytes()).hexdigest()
        == metadata["geometry_sha256"]
    ):
        raise ValueError("Source mesh changed")
    with np.load(folder / "geometry.npz", allow_pickle=False) as z:
        geometry = dict(z)
    return geometry, metadata


def validate(folder):
    from route_schema import VERSION, validate_routes

    if (
        json.loads((Path(folder) / "metadata.json").read_text())["schema_version"]
        == VERSION
    ):
        return validate_routes(folder)
    geometry, meta = load_export(folder)
    validate_mesh(geometry)
    v, f, n = geometry["vertices_xyz"], geometry["triangles"], geometry["face_normal"]
    area = (
        np.linalg.norm(
            np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]]), axis=1
        )
        / 2
    )
    report = {
        "vertices": len(v),
        "triangles": len(f),
        "finite": bool(np.isfinite(v).all() and np.isfinite(n).all()),
        "index_range_valid": bool(f.min() >= 0 and f.max() < len(v)),
        "degenerate_triangles": int((area < 1e-12).sum()),
        "face_normal_unit_max_error": float(np.max(abs(np.linalg.norm(n, axis=1) - 1))),
        "routes": [],
        "unavailable": meta["unavailable"],
    }
    if "triangle_surface_index" in geometry:
        report["surface_indices_valid"] = validate_surface_indices(
            geometry["triangle_surface_index"], meta["surfaces"]
        )
    for route in meta["routes"]:
        with np.load(Path(folder) / route["file"], allow_pickle=False) as z:
            ds = np.linalg.norm(np.diff(z["center_xyz"], axis=0), axis=1)
            off = z["section_offsets"]
            ordered = True
            for a, b in zip(off[:-1], off[1:]):
                lateral_key = (
                    "section_lateral_xy_m"
                    if "section_lateral_xy_m" in z
                    else "section_lateral_m"
                )
                ordered &= bool(np.all(np.diff(z[lateral_key][a:b]) > 0))
            first = z["center_xyz"][1] - z["center_xyz"][0]
            last = z["center_xyz"][-1] - z["center_xyz"][-2]
            cosine = np.dot(first, last) / max(
                np.linalg.norm(first) * np.linalg.norm(last), 1e-30
            )
            report["routes"].append(
                {
                    "name": route["name"],
                    "stations": len(z["station_id"]),
                    "cross_sections_ordered": ordered,
                    "cross_section_offsets_valid": bool(
                        off[0] == 0
                        and off[-1] == len(z["section_xyz"])
                        and np.all(np.diff(off) >= 2)
                    ),
                    "sampled_horizontal_width_min_m": float(
                        np.diff(
                            (
                                z["sampled_limits_xy_m"]
                                if "sampled_limits_xy_m" in z
                                else z["profile_limits_m"]
                            ),
                            axis=1,
                        ).min()
                    ),
                    "duplicate_adjacent_stations": int((ds < 1e-8).sum()),
                    "max_spacing_m": float(ds.max()),
                    "min_xyz": z["center_xyz"].min(axis=0).tolist(),
                    "max_xyz": z["center_xyz"].max(axis=0).tolist(),
                    "endpoint_gap_m": float(
                        np.linalg.norm(z["center_xyz"][0] - z["center_xyz"][-1])
                    ),
                    "endpoint_chord_heading_difference_deg": float(
                        np.degrees(np.arccos(np.clip(cosine, -1, 1)))
                    ),
                    "native_seam_forward_angle_deg": route.get(
                        "seam_forward_angle_deg"
                    ),
                    "native_seam_up_angle_deg": route.get("seam_up_angle_deg"),
                    "topology": route["topology"],
                }
            )
            if "section_surface_index" in z:
                report["routes"][-1]["surface_indices_valid"] = bool(
                    validate_surface_indices(
                        z["section_surface_index"], meta["surfaces"]
                    )
                    and validate_surface_indices(
                        z["center_surface_index"], meta["surfaces"]
                    )
                )
    (Path(folder) / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def select_recipe(folder, recipe, output):
    """Materialize selected route rows; keep gaps as separate parts, no clipping.

    Area selection tests center XYZ. Original cross-sections are retained whole;
    this deliberately does not invent new pavement boundaries on the ROI box.
    """
    _, meta = load_export(folder)
    if recipe["geometry_sha256"] != meta["geometry_sha256"]:
        raise ValueError("Recipe belongs to a different immutable geometry export")
    if recipe.get("asset_sha256", meta["asset_sha256"]) != meta["asset_sha256"]:
        raise ValueError("Recipe belongs to a different source asset")
    if recipe.get("frame") != "local_ENU":
        raise ValueError("Recipe must use local_ENU")
    route = None
    arrays = None
    mask = None
    for operation in recipe["operations"]:
        op = operation["op"]
        if op == "select_route":
            if route is not None:
                raise ValueError("Select one route per recipe")
            route = next(r for r in meta["routes"] if r["name"] == operation["name"])
            with np.load(Path(folder) / route["file"], allow_pickle=False) as z:
                arrays = dict(z)
            mask = np.ones(len(arrays["station_id"]), bool)
        elif route is None:
            raise ValueError("Select a route before applying operations")
        elif op == "slice_stations":
            first, last = operation["first"], operation["last"]
            ids = arrays["station_id"]
            if first not in ids or last not in ids or first > last:
                raise ValueError("Invalid explicit station interval")
            mask &= (ids >= first) & (ids <= last)
        elif op == "spatial_box":
            low, high = np.asarray(operation["min_xyz"], float), np.asarray(
                operation["max_xyz"], float
            )
            if (
                low.shape != (3,)
                or high.shape != (3,)
                or not np.isfinite([low, high]).all()
                or (low > high).any()
            ):
                raise ValueError("Invalid ENU box")
            mask &= (
                (arrays["center_xyz"] >= low) & (arrays["center_xyz"] <= high)
            ).all(axis=1)
        else:
            raise ValueError(f"Unsupported recipe operation: {op}")
    if route is None:
        raise ValueError("No route selected")
    rows = np.flatnonzero(mask)
    if not len(rows):
        raise ValueError(
            "Selection has no source stations; enlarge the area or change route"
        )
    offsets = arrays["section_offsets"]
    sample_rows = np.concatenate([np.arange(offsets[i], offsets[i + 1]) for i in rows])
    count = np.array([offsets[i + 1] - offsets[i] for i in rows])
    selected = dict(
        station_id=arrays["station_id"][rows],
        original_row=rows,
        part_id=np.r_[0, np.cumsum(np.diff(rows) != 1)],
        center_xyz=arrays["center_xyz"][rows],
        source_ref=arrays["source_ref"][rows],
        original_distance_xyz_m=arrays.get(
            "center_distance_xyz_m", arrays.get("distance_xyz_m")
        )[rows],
        section_xyz=arrays["section_xyz"][sample_rows],
        section_lateral_xy_m=arrays.get(
            "section_lateral_xy_m", arrays.get("section_lateral_m")
        )[sample_rows],
        section_offsets=np.r_[0, np.cumsum(count)],
    )
    for name in ["center_surface_index"]:
        if name in arrays:
            selected[name] = arrays[name][rows]
    for name in ["section_surface_index", "section_vertex_id"]:
        if name in arrays:
            selected[name] = arrays[name][sample_rows]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output / "selection.npz", **selected)
    (output / "recipe.json").write_text(json.dumps(recipe, indent=2) + "\n")
    summary = {
        "source_geometry_sha256": meta["geometry_sha256"],
        "route": route["name"],
        "surfaces": meta.get("surfaces", []),
        "stations": len(rows),
        "parts": int(selected["part_id"][-1]) + 1,
        "semantics": "Source stations selected by center XYZ; whole cross-sections retained. Gaps remain separate parts. No geometry clipping or smoothing.",
    }
    (output / "selection.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def validate_mesh(geometry):
    """Reject malformed mesh arrays before indexing; return geometric diagnostics."""
    required = {"vertices_xyz", "triangles", "face_normal"}
    if required - geometry.keys():
        raise ValueError(
            "Missing mesh arrays: " + ", ".join(sorted(required - geometry.keys()))
        )
    v, f, normal = (
        geometry[key] for key in ("vertices_xyz", "triangles", "face_normal")
    )
    if (
        v.ndim != 2
        or v.shape[1] != 3
        or len(v) == 0
        or f.ndim != 2
        or f.shape[1] != 3
        or len(f) == 0
        or f.dtype.kind not in "iu"
        or normal.shape != (len(f), 3)
    ):
        raise ValueError("Invalid mesh array shapes or triangle index type")
    if not np.isfinite(v).all() or not np.isfinite(normal).all():
        raise ValueError("Mesh positions and normals must be finite")
    if (f < 0).any() or (f >= len(v)).any():
        raise ValueError("Mesh triangle indices are out of range")
    degenerate = int(
        (
            np.linalg.norm(
                np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]]), axis=1
            )
            < 2e-12
        ).sum()
    )
    error = float(np.max(abs(np.linalg.norm(normal, axis=1) - 1)))
    if degenerate or error > 1e-4:
        raise ValueError("Mesh contains degenerate triangles or nonunit normals")
    return dict(
        vertices=len(v),
        triangles=len(f),
        finite=True,
        index_range_valid=True,
        degenerate_triangles=degenerate,
        face_normal_unit_max_error=error,
    )
