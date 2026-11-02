"""Save controller identity, local reference files and tracking configuration."""

from pathlib import Path

import yaml

DEFAULT_INFO_SERVICE = "/drive_high_level_ros/get_ref_traj_info"


def snapshot_ref_traj_info(response, run_folder, warn):
    """Snapshot a locally available reference; retain provenance and explicit failures.

    The reference must match the controller's hashes. No ROS/SSH transfer occurs.
    Sensor recording can continue when files are unavailable, but metadata marks
    the reference incomplete and replay cannot silently select another trajectory.
    """
    from drive_trajopt.traj.artifact import snapshot_reference

    if not response.success:
        raise ValueError(response.message or "GetRefTrajInfo returned success=false")
    descriptor = response.info
    info = {}
    for field in descriptor.get_fields_and_field_types():
        value = getattr(descriptor, field)
        info[field] = value if isinstance(value, (str, bool, int, float)) else list(value)
    run = Path(run_folder)
    info["reference_dir"] = "reference"
    try:
        snapshot_reference(descriptor, run)
    except (OSError, ValueError, AttributeError) as exc:
        info["reference_snapshot_error"] = str(exc)
        warn(f"REFERENCE SNAPSHOT FAILED: {exc}")
    source = Path(descriptor.tracking_config).expanduser()
    try:
        if not descriptor.tracking_config or not source.is_file():
            raise FileNotFoundError(f"Tracking YAML is not accessible locally: {source}")
        config = yaml.safe_load(source.read_text())
        if not isinstance(config, dict):
            raise ValueError(f"Tracking YAML must be a mapping: {source}")
        if "reference_snapshot_error" not in info:
            config["artifact_dir"] = "reference"
        destination = run / "tracking.yaml"
        destination.write_text(yaml.safe_dump(config, sort_keys=False))
        info["tracking_yaml"] = destination.name
    except (OSError, ValueError, yaml.YAMLError) as exc:
        info["tracking_snapshot_error"] = str(exc)
        warn(f"TRACKING SNAPSHOT FAILED: {exc}")
    return info
