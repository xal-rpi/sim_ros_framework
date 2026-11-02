"""Explicit offline conversion of a saved network recipe to current references."""

import hashlib
import json
from pathlib import Path
import subprocess


def rebuild_reference(folder, recipe_path, output):
    from guide_geometry import write_guide_geometry
    from track_recipe import load_recipe

    folder, recipe_path, output = Path(folder), Path(recipe_path), Path(output)
    if output.exists():
        raise FileExistsError("Choose a new recipe name")
    recipe, _ = load_recipe(folder, recipe_path)
    if (
        recipe["mode"] != "network"
        or recipe["reference_kind"] != "native_evaluated_guides"
    ):
        raise ValueError("Expected a network recipe using evaluated guides")
    association_path = write_guide_geometry(folder)
    payload = dict(
        recipe=recipe,
        navigation=json.loads((folder / "navigation.json").read_text()),
        associations=json.loads(association_path.read_text()),
        parent_recipe_sha256=hashlib.sha256(recipe_path.read_bytes()).hexdigest(),
    )
    try:
        result = subprocess.run(
            ["node", str(Path(__file__).with_suffix(".js"))],
            input=json.dumps(payload),
            text=True,
            capture_output=True,
            check=True,
        )
    except FileNotFoundError as e:
        raise RuntimeError(
            "Rebuilding a reference offline requires Node.js; extraction of saved recipes does not."
        ) from e
    except subprocess.CalledProcessError as e:
        raise ValueError(e.stderr.strip()) from e
    rebuilt = json.loads(result.stdout)
    rebuilt.update(
        navigation_geometry_file=association_path.relative_to(folder).as_posix(),
        navigation_geometry_sha256=hashlib.sha256(
            association_path.read_bytes()
        ).hexdigest(),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    # Validate through the same Python replay path used before live extraction.
    staged = output.with_suffix(".rebuild.json")
    created = False
    try:
        with staged.open("x") as stream:
            created = True
            stream.write(json.dumps(rebuilt, indent=2, allow_nan=False) + "\n")
        load_recipe(folder, staged)
        staged.replace(output)
    finally:
        if created and staged.exists():
            staged.unlink()
    return dict(
        recipe=str(output),
        transitions=len(rebuilt["reference_operations"][0]["transitions"]),
        unresolved_transitions=rebuilt["transition_warnings"],
        stations=len(rebuilt["reference_xyz"]),
        max_waypoint_reprojection_m=max(
            p["displacement_xyz_m"]
            for p in rebuilt["reference_rebuild"]["waypoint_reprojections"]
        ),
    )
