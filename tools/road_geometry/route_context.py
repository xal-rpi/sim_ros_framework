"""Optional nearby collision observations for visualization, not route geometry."""

import json
from pathlib import Path

import numpy as np

from route_selection import digest, load_route


def nearby(points, centers, radius):
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError("Radius must be positive")
    centers = centers[np.isfinite(centers).all(axis=1)]
    buckets = {}
    for p in centers:
        key = tuple(np.floor(p[:2] / radius).astype(int))
        buckets.setdefault(key, []).append(p[:2])
    mask = np.zeros(len(points), bool)
    groups = {}
    for i, p in enumerate(points):
        if np.isfinite(p).all():
            groups.setdefault(tuple(np.floor(p[:2] / radius).astype(int)), []).append(i)
    for (x, y), indices in groups.items():
        refs = [
            p
            for a in range(x - 1, x + 2)
            for b in range(y - 1, y + 2)
            for p in buckets.get((a, b), [])
        ]
        if refs:
            for chunk in range(0, len(indices), 1024):
                ids = indices[chunk : chunk + 1024]
                mask[ids] = (
                    np.sum(
                        (points[ids, None, :2] - np.asarray(refs)[None, :, :]) ** 2,
                        axis=2,
                    ).min(axis=1)
                    <= radius**2
                )
    return mask


def save_context(folder, points, radius, source):
    folder = Path(folder)
    if (folder / "context.npz").exists() or (folder / "context.json").exists():
        raise FileExistsError(
            "Context already exists; original observations are preserved"
        )
    meta = json.loads((folder / "metadata.json").read_text())
    if "routes" in meta and not (folder / "route.npz").exists():
        centers = np.concatenate(
            [load_route(folder, r["name"])[0]["center_xyz"] for r in meta["routes"]]
        )
    else:
        centers = load_route(folder)[0]["center_xyz"]
    points = np.asarray(points, float).reshape(-1, 3)
    points = points[nearby(points, centers, radius)]
    np.savez_compressed(folder / "context.npz", xyz=points)
    info = dict(
        schema_version="0.1-route-context",
        frame="local_ENU",
        units="m",
        points=len(points),
        radius_m=radius,
        source=source,
        sha256=digest(folder / "context.npz"),
        definition="Saved static collision observations within horizontal radius of sampled route centers. All observed Z layers retained; first hit per source height band only. Visualization context, not drivable area, route membership or mesh connectivity.",
    )
    (folder / "context.json").write_text(json.dumps(info, indent=2) + "\n")
    return info


def from_survey(folder, survey, radius):
    from native_scene import load_survey

    folder, survey = Path(folder), Path(survey)
    meta = load_survey(survey)
    destination = json.loads((folder / "metadata.json").read_text())
    if destination["source"]["mission"] != meta["source"]["mission"]:
        raise ValueError("Context survey belongs to another level")
    pointer = survey / "overview_current.json"
    if pointer.exists():
        info = json.loads(pointer.read_text())
        path = (survey / info["samples_file"]).resolve()
        if not path.is_relative_to(survey.resolve()):
            raise ValueError("Context source escapes survey")
        if digest(path) != info["samples_sha256"]:
            raise ValueError("Context source hash changed")
    else:
        path = survey / "surface_samples.npz"
    with np.load(path, allow_pickle=False) as z:
        points = z["surface_xyz"][z["surface_valid"]]
    return save_context(
        folder,
        points,
        radius,
        dict(
            kind="saved_native_survey",
            mission=meta["source"]["mission"],
            samples_sha256=digest(path),
        ),
    )
