"""Associate native graph links with evaluated decal guide curves.

This is a derived reference-path association, never a change to map geometry.
Graph endpoint offsets and unsupported junctions remain explicit source roles.
"""

import hashlib
import json
from pathlib import Path
import numpy as np
from native_scene import load_survey


def project(line, point):
    a, b = line[:-1], line[1:]
    delta = b[:, :2] - a[:, :2]
    t = np.clip(
        ((point[:2] - a[:, :2]) * delta).sum(1)
        / np.maximum((delta * delta).sum(1), 1e-20),
        0,
        1,
    )
    q = a + (b - a) * t[:, None]
    d = np.linalg.norm(q[:, :2] - point[:2], axis=1)
    j = int(np.argmin(d))
    return j + float(t[j]), q[j], float(d[j])


def local_projections(line, point):
    """Keep locally plausible branches until a directed interval is checked.

    The nearest point alone is insufficient at overlapping open-guide ends:
    it can select the far end of the parameter range, even though another
    projection follows the native link without traversing the entire road.
    This never wraps/closes a guide or connects separate guide objects.
    """
    delta = line[1:, :2] - line[:-1, :2]
    t = np.clip(
        ((point[:2] - line[:-1, :2]) * delta).sum(1)
        / np.maximum((delta * delta).sum(1), 1e-20),
        0,
        1,
    )
    q = line[:-1] + (line[1:] - line[:-1]) * t[:, None]
    d = np.linalg.norm(q[:, :2] - point[:2], axis=1)
    result = []
    for j in np.flatnonzero((d <= 0.9) & (abs(q[:, 2] - point[2]) <= 1)):
        station = j + float(t[j])
        if not any(abs(station - p[0]) < 1e-8 for p in result):
            result.append((station, q[j], float(d[j])))
    return result


def trim(line, start, end):
    def at(s):
        i = min(int(s), len(line) - 2)
        return line[i] + (line[i + 1] - line[i]) * (s - i)

    low, high = sorted([start, end])
    points = np.vstack(
        [at(low), line[int(np.floor(low)) + 1 : int(np.ceil(high))], at(high)]
    )
    if end < start:
        points = points[::-1]
    return points


def build_guide_geometry(folder):
    folder = Path(folder)
    meta = load_survey(folder)
    nav_path = folder / "navigation.json"
    nav = json.loads(nav_path.read_text())
    with np.load(folder / "road_guides.npz", allow_pickle=False) as z:
        lines = [
            (i, z["edge_xyz"][a:b, 1])
            for i, (a, b) in enumerate(
                zip(z["road_offsets"][:-1], z["road_offsets"][1:])
            )
            if float(meta["roads"][i]["drivability"]) > 0 and b - a >= 2
        ]
    result = []
    for edge in nav["edges"]:
        a, b = [np.array(nav["nodes"][edge[k]]["xyz"], float) for k in ["from", "to"]]
        chord = np.linalg.norm(b - a)
        candidates = []
        for guide_id, line in lines:
            for s, qa, da in local_projections(line, a):
                for t, qb, db in local_projections(line, b):
                    if abs(t - s) < 1e-8:
                        continue
                    points = trim(line, s, t)
                    length = np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
                    if length > 1.8 * chord + 0.1:
                        continue  # do not implicitly wrap an open guide's seam
                    candidates.append(
                        dict(
                            score=da + db + 0.01 * length,
                            guide_id=guide_id,
                            points=points,
                            native_station_interval=[s, t],
                            endpoint_offset_m=[
                                float(np.linalg.norm(qa - a)),
                                float(np.linalg.norm(qb - b)),
                            ],
                        )
                    )
        candidates.sort(key=lambda c: c["score"])
        if not candidates:
            result.append(
                dict(
                    points=[a.tolist(), b.tolist()],
                    roles=["native_navigation_chord"],
                    guide_id=None,
                    reason="No reliable local guide association; native connectivity retained, no gap repair",
                )
            )
            continue
        best = candidates[0]
        # Different source objects may overlap. Refuse materially different
        # equal-score curves; identical geometry retains all candidate identities.
        ambiguous = False
        for c in candidates[1:]:
            if c["score"] - best["score"] > 0.02:
                break
            distance = max(project(c["points"], p)[2] for p in best["points"])
            if distance > 0.1:
                ambiguous = True
                break
        if ambiguous:
            result.append(
                dict(
                    points=[a.tolist(), b.tolist()],
                    roles=["native_navigation_chord"],
                    guide_id=None,
                    reason="Ambiguous guide curves; native link retained for explicit review",
                )
            )
            continue
        points = [a]
        roles = []
        for j, p in enumerate(best["points"]):
            if np.linalg.norm(p - points[-1]) <= 1e-8:
                continue
            points.append(p)
            roles.append(
                "native_graph_endpoint_connector" if j == 0 else "evaluated_guide"
            )
        if np.linalg.norm(b - points[-1]) > 1e-8:
            points.append(b)
            roles.append("native_graph_endpoint_connector")
        result.append(
            dict(
                points=np.asarray(points).tolist(),
                roles=roles,
                guide_id=best["guide_id"],
                guide_persistent_id=meta["roads"][best["guide_id"]]["persistent_id"],
                native_station_interval=best["native_station_interval"],
                endpoint_offset_m=best["endpoint_offset_m"],
                candidate_guide_ids=[
                    c["guide_id"]
                    for c in candidates
                    if c["score"] - best["score"] <= 0.02
                ],
            )
        )
    return dict(
        schema_version="0.5-guide-associations",
        source_files_sha256=meta["files_sha256"],
        navigation_sha256=hashlib.sha256(nav_path.read_bytes()).hexdigest(),
        edges=result,
        construction="Derived route references: trimmed evaluated guide middle points plus explicit native-graph endpoint connectors; unsupported links keep their native chord.",
        settings=dict(
            max_endpoint_xy_offset_m=0.9,
            max_endpoint_z_offset_m=1,
            max_guide_length_to_chord_ratio=1.8,
            endpoint_projection="all local branches; directed interval length checked before ranking",
        ),
        caveat="Guides are not certified pavement centerlines or boundaries. Connectors and fallback chords are explicitly labelled.",
    )


def write_guide_geometry(folder):
    result = build_guide_geometry(folder)
    content = (json.dumps(result, separators=(",", ":")) + "\n").encode()
    path = (
        Path(folder)
        / "guide_associations"
        / (hashlib.sha256(content).hexdigest() + ".json")
    )
    # Content-addressed derived snapshots keep older saved recipes reproducible.
    # In particular, never replace the original navigation_geometry.json.
    path.parent.mkdir(exist_ok=True)
    if not path.exists():
        path.write_bytes(content)
    return path
