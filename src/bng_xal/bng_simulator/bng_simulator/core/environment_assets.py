"""Load configured experiment assets after the base scene, before simulation starts.

Assets live in xlab_environments, not in the vanilla level or integration mod.
The server returns actual file bytes for checksum verification and run metadata.
"""

import hashlib
from pathlib import PurePosixPath


def load_environment_assets(beamng, config: dict, logger) -> None:
    environment = config.get("environment", {})
    if not isinstance(environment, dict):
        raise ValueError("environment must be a mapping")
    paths = environment.get("trackbuilder_tracks", [])
    if not isinstance(paths, list):
        raise ValueError("environment.trackbuilder_tracks must be a list of asset paths")
    if not paths:
        environment.pop("loaded_assets", None)
        return
    for path in paths:
        if (not isinstance(path, str) or not path.startswith("/trackEditor/xlab_environments/")
                or ".." in PurePosixPath(path).parts or not path.endswith(".json")):
            raise ValueError(f"Invalid xlab_environments Track Builder path: {path!r}")
    if len(set(paths)) != len(paths):
        raise ValueError("Duplicate Track Builder assets would spawn duplicate geometry")

    response = beamng._send({"type": "GetEnvironmentAssets", "paths": paths}).recv(
        "EnvironmentAssets"
    )
    manifest = response["manifest"]
    if manifest.get("name") != "xlab_environments":
        raise ValueError("Unexpected environment mod manifest")
    assets = {entry["path"]: entry for entry in manifest["assets"].values()}
    resolved = []
    # Verify the entire batch before spawning anything, so a broken deployment
    # fails during initialization rather than silently producing a partial scene.
    for path in paths:
        entry = assets.get(path)
        if entry is None or entry.get("type") != "trackbuilder":
            raise ValueError(f"Track Builder asset is missing from the mod manifest: {path}")
        content = response["contents"][path]
        digest = hashlib.sha256(content.encode("utf-8")).hexdigest()
        if digest != entry["sha256"]:
            raise ValueError(f"Environment asset checksum mismatch; rebuild the mod: {path}")
        if entry.get("base_level") != config["scenario"]["level"]:
            raise ValueError(f"Asset {path} requires level {entry.get('base_level')}")
        resolved.append({"path": path, "sha256": digest,
                         "package": manifest["name"], "version": manifest["version"]})

    for entry in resolved:
        beamng.scenario.load_trackbuilder_track(entry["path"])
        logger.info(f"Loaded environment asset {entry['path']} sha256={entry['sha256']}")
    # SimulationRequestHandler.get_sim_config() includes these deployed hashes
    # in the existing metadata workflow, without introducing a separate logger.
    environment["loaded_assets"] = resolved
