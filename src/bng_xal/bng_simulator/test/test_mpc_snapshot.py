"""Tracking snapshots stay portable and failures do not prevent sensor logging."""

from array import array
from pathlib import Path
import sys
from threading import Event
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

from bng_simulator.utils.mpc_snapshot import snapshot_ref_traj_info


def response(path, **overrides):
    from drive_trajopt.traj.artifact import reference_hashes, reference_identity

    root = Path(overrides.pop('local_reference', Path(path).parent / 'reference-source'))
    root.mkdir(parents=True, exist_ok=True)
    for name in ('meta.yaml', 'traj.csv', 'surface.npz'):
        (root / name).write_text(name)
    hashes = reference_hashes(root)
    success, message = overrides.pop("success", True), overrides.pop("message", "")
    values = dict(tracking_config=str(path),
                  artifact_name="donut", artifact_dir=str(root),
                  reference_id=reference_identity(hashes), reference_files=list(hashes),
                  reference_sha256=list(hashes.values()),
                  x_labels=["s", "e"], u_labels=["steer"], session_id=42, contract_version=2,
                  delta_s=array("d", [0.5, 0.5]), surface_planar=False)
    values.update(overrides)
    result = SimpleNamespace(**values)
    result.get_fields_and_field_types = lambda: {key: "unused" for key in values}
    return SimpleNamespace(success=success, message=message, info=result)


def test_snapshot_copies_loaded_yaml_and_captures_identity(tmp_path):
    source = tmp_path / "original.yaml"
    source.write_text("mpc:\n  n_stages: 10\n")
    run = tmp_path / "run_001"
    run.mkdir()
    warn = Mock()
    captured = snapshot_ref_traj_info(response(source), run, warn)
    source.write_text("modified after recording starts\n")
    assert yaml.safe_load((run / "tracking.yaml").read_text()) == {"mpc": {"n_stages": 10}, "artifact_dir": "reference"}
    assert captured["reference_dir"] == "reference"
    assert (run / "reference" / "traj.csv").read_text() == "traj.csv"
    assert captured["tracking_yaml"] == "tracking.yaml"
    assert captured["x_labels"] == ["s", "e"]
    assert captured["delta_s"] == [0.5, 0.5]
    assert yaml.safe_load(yaml.safe_dump(captured)) == captured
    warn.assert_not_called()


@pytest.mark.parametrize("path", ["", "/nonexistent/controller/tracking.yaml"])
def test_unavailable_yaml_still_preserves_controller_identity(tmp_path, path):
    warn = Mock()
    captured = snapshot_ref_traj_info(response(path, local_reference=tmp_path / "source"), tmp_path, warn)
    assert captured["artifact_name"] == "donut"
    assert "tracking_yaml" not in captured
    warn.assert_called_once()


def test_unsuccessful_service_response_is_not_captured(tmp_path):
    with pytest.raises(ValueError, match="controller not ready"):
        snapshot_ref_traj_info(response("", success=False, message="controller not ready", local_reference=tmp_path / "source"),
                               tmp_path, Mock())
    assert not (tmp_path / "tracking.yaml").exists()


def test_run_metadata_exposes_tracking_yaml_for_replay(tmp_path):
    from bng_simulator.utils.log_session import LoggedRun, build_metadata

    run = LoggedRun(str(tmp_path), str(tmp_path / "data"), {}, 1)
    captured = {"tracking_yaml": "tracking.yaml", "x_labels": ["s", "e"]}
    metadata = build_metadata(run, mpc_config=captured)
    assert metadata["tracking_yaml"] == "tracking.yaml"
    assert metadata["mpc_config"]["x_labels"] == ["s", "e"]


def test_background_capture_handles_late_service_and_custom_namespace(tmp_path, monkeypatch):
    from bng_simulator.scripts import start_logs
    from rclpy import executors, node

    assert "/drive_high_level_ros/controller_info" in start_logs.DEFAULT_TOPICS
    source = tmp_path / "loaded.yaml"
    source.write_text("mpc: {}\n")
    future = Mock()
    future.done.return_value = True
    future.cancelled.return_value = False
    future.result.return_value = response(source)
    service = Mock()
    service.wait_for_service.side_effect = [False, True]
    service.call_async.return_value = future
    ros_node = Mock()
    ros_node.create_client.return_value = service
    executor = Mock()
    msg_type = SimpleNamespace(Request=Mock())
    monkeypatch.setitem(sys.modules, "xal_drive_msgs.srv", SimpleNamespace(GetRefTrajInfo=msg_type))
    monkeypatch.setattr(node, "Node", Mock(return_value=ros_node))
    monkeypatch.setattr(executors, "SingleThreadedExecutor", Mock(return_value=executor))
    monkeypatch.setattr(start_logs.rclpy, "ok", lambda: True)
    stop = Event()
    monkeypatch.setattr(stop, "wait", Mock(return_value=False))
    monkeypatch.setattr(start_logs, "mpc_config_stop_event", stop)
    monkeypatch.setattr(start_logs, "mpc_info_service", "/custom/get_ref_traj_info")
    monkeypatch.setattr(start_logs, "run_folder", str(tmp_path))
    monkeypatch.setattr(start_logs, "mpc_config", None)
    monkeypatch.setattr(start_logs, "mpc_config_received", False)
    start_logs.check_mpc_config()
    assert start_logs.mpc_config_received is True
    assert start_logs.mpc_config["service"] == "/custom/get_ref_traj_info"
    assert start_logs.mpc_config["tracking_yaml"] == "tracking.yaml"
    ros_node.create_client.assert_called_once_with(msg_type, "/custom/get_ref_traj_info")
    ros_node.destroy_node.assert_called_once()
    executor.shutdown.assert_called_once()


def test_changed_reference_is_marked_incomplete_without_stopping_sensor_logging(tmp_path):
    source = tmp_path / 'tracking.yaml'
    source.write_text('artifact_dir: original\n')
    result = response(source)
    (Path(result.info.artifact_dir) / 'traj.csv').write_text('regenerated')
    run = tmp_path / 'run'
    run.mkdir()
    warn = Mock()
    captured = snapshot_ref_traj_info(result, run, warn)
    assert captured['reference_dir'] == 'reference'
    assert 'traj.csv' in captured['reference_snapshot_error']
    assert not (run / 'reference').exists()
    assert yaml.safe_load((run / 'tracking.yaml').read_text())['artifact_dir'] == 'original'
    assert 'REFERENCE SNAPSHOT FAILED' in warn.call_args.args[0]



def test_logger_stop_drains_records_already_accepted_by_the_queue(tmp_path):
    import multiprocessing as mp
    import pickle
    from bng_simulator.logger_process import LoggerProcess
    from bng_simulator.utils.log_session import run_number_from_folder

    assert run_number_from_folder(tmp_path / 'recording') is None
    assert run_number_from_folder(tmp_path / 'run_001') == 1
    queue = mp.Queue()
    process = LoggerProcess(queue, str(tmp_path), flush_interval=60)
    process.start()
    try:
        for index in range(250):
            queue.put(dict(vehicle_name='EGO', sensor_name='gtstate', data=[dict(time=index)]))
        process.stop()
        process.join(5)
        assert not process.is_alive() and process.exitcode == 0
        records = []
        for path in sorted(tmp_path.glob('*.pkl')):
            records.extend(pickle.loads(path.read_bytes())[('EGO','gtstate')]['time'])
        assert records == list(range(250))
    finally:
        if process.is_alive():
            process.terminate()
            process.join(2)
        queue.close()
        queue.join_thread()



def test_batch_recording_does_not_read_or_end_on_closed_stdin(monkeypatch):
    from bng_simulator.scripts import start_logs
    monkeypatch.setattr('builtins.input', Mock(side_effect=AssertionError('Batch must not read stdin')))
    ticks = []
    def stop_after_ticks(seconds):
        ticks.append(seconds)
        if len(ticks) == 3:
            raise KeyboardInterrupt
    monkeypatch.setattr(start_logs.time, 'sleep', stop_after_ticks)
    start_logs.wait_for_stop(True)
    assert len(ticks) == 3
