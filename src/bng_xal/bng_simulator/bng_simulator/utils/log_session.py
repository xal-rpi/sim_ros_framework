"""
Programmatic BeamNG log sessions (start_logger / stop_logger + metadata + consolidate).

Interactive CLI: ``ros2 run bng_simulator start_logs``
Batch / replay callers::

    from bng_simulator.utils.log_session import LoggerClient, begin_run, end_run

    client = LoggerClient()
    run = begin_run(client, "~/beamng_log_data/my_batch")
    # ... experiment ...
    end_run(client, run, {"additional_info": "px4 seg 0", "px4_segment_id": 0})
"""

from __future__ import annotations

import os
import pickle
import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import yaml
from bng_msgs.srv import MPConfig, StartLogger, StopLogger
from rclpy.node import Node
from tqdm.auto import tqdm

from bng_simulator.utils.services_utils import send_request

MSG_VERSION = "1.0"


def get_next_run_folder(root_dir: str | os.PathLike) -> str:
    """Create and return the next ``run_XXX`` folder under *root_dir*."""
    root_dir = os.path.expanduser(str(root_dir))
    os.makedirs(root_dir, exist_ok=True)
    runs = [d for d in os.listdir(root_dir) if d.startswith("run_")]
    if runs:
        nums = [int(d.split("_")[1]) for d in runs if d.split("_")[1].isdigit()]
        next_num = max(nums) + 1 if nums else 1
    else:
        next_num = 1
    run_folder = os.path.join(root_dir, f"run_{next_num:03d}")
    os.makedirs(run_folder)
    return run_folder


def run_number_from_folder(run_folder: str | os.PathLike) -> int:
    return int(os.path.basename(str(run_folder)).split("_")[1])


def consolidate_logger_files(data_dir: str | os.PathLike) -> Path:
    """
    Merge ``data_*.pkl`` shards under *data_dir* into ``data.pkl``.

    *data_dir* is the logger prefix passed to ``start_logger`` (e.g. ``run_001/data``).
    """
    data_dir = str(data_dir)
    consolidated: dict = {}
    temp_files = []
    for fname in os.listdir(data_dir):
        if fname.startswith("data_") and fname.endswith(".pkl"):
            try:
                num = int(fname[len("data_") : -len(".pkl")])
                temp_files.append((num, os.path.join(data_dir, fname)))
            except ValueError:
                continue
    temp_files.sort(key=lambda x: x[0])

    for _, file_path in tqdm(temp_files, desc="Consolidating data files", unit="file"):
        with open(file_path, "rb") as f:
            data = pickle.load(f)
        for key, sensor_data in data.items():
            if key not in consolidated:
                consolidated[key] = {}
            for field, values in sensor_data.items():
                consolidated[key].setdefault(field, []).extend(values)

    final_data = {
        "version": MSG_VERSION,
        "format": "sensor_timeseries",
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "data": consolidated,
    }
    out = Path(data_dir) / "data.pkl"
    with open(out, "wb") as f:
        pickle.dump(final_data, f)
    for _, file_path in temp_files:
        os.remove(file_path)
    return out


class LoggerClient(Node):
    """ROS client for ``start_logger`` / ``stop_logger`` services."""

    def __init__(self, node_name: str = "logger_client"):
        super().__init__(node_name)
        self.start_client = self.create_client(StartLogger, "start_logger")
        self.stop_client = self.create_client(StopLogger, "stop_logger")
        self.mpc_config_client = self.create_client(MPConfig, "/mpc_high_level/get_config")
        while not self.start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for start_logger service...")
        while not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for stop_logger service...")

    def start_logging(self, file_path: str, max_queue_size: int, flush_interval: float):
        req = StartLogger.Request()
        req.save_location = file_path
        req.max_queue_size = int(max_queue_size)
        req.flush_interval = float(flush_interval)
        future = self.start_client.call_async(req)
        import rclpy

        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def stop_logging(self):
        req = StopLogger.Request()
        future = self.stop_client.call_async(req)
        import rclpy

        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def get_mpc_config(self):
        if not self.mpc_config_client.wait_for_service(timeout_sec=0.1):
            return None
        req = MPConfig.Request()
        future = self.mpc_config_client.call_async(req)
        import rclpy

        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.done():
                return future.result()
        except Exception as exc:
            self.get_logger().warning(f"Error getting MPC config: {exc}")
        return None


@dataclass
class LoggedRun:
    """An active or completed log session."""

    run_folder: str
    data_path: str
    scenario_infos: Dict[str, Any]
    run_number: int

    @property
    def data_pkl(self) -> Path:
        return Path(self.data_path) / "data.pkl"


def build_metadata(
    run: LoggedRun,
    *,
    map_name: str = "",
    additional_info: str = "",
    mpc_config: Optional[Dict[str, Any]] = None,
    rosbag: Optional[Dict[str, Any]] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Build metadata dict the same way ``start_logs`` does."""
    scenario_infos = run.scenario_infos or {}
    vehicles_part = scenario_infos.get("vehicles_part", {})
    metadata: Dict[str, Any] = {
        "map_name": map_name,
        "additional_info": additional_info,
        "run_folder": os.path.basename(run.run_folder),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "msg_version": MSG_VERSION,
        **vehicles_part,
        "sim": scenario_infos,
    }
    if mpc_config:
        metadata["mpc_config"] = mpc_config
    if rosbag is not None:
        metadata["rosbag"] = rosbag
    if extra:
        metadata.update(extra)
    return metadata


def begin_run(
    client: LoggerClient,
    root_dir: str | os.PathLike = "~/beamng_log_data",
    *,
    run_folder: Optional[str | os.PathLike] = None,
    max_queue_size: int = 5000,
    flush_interval: float = 5.0,
) -> LoggedRun:
    """
    Allocate ``run_XXX`` (unless *run_folder* given), start ``start_logger``.

    Raises ``RuntimeError`` if the logger service fails to start (folder removed).
    """
    if run_folder is None:
        run_folder = get_next_run_folder(root_dir)
    else:
        run_folder = str(run_folder)
        os.makedirs(run_folder, exist_ok=True)
    data_path = os.path.join(run_folder, "data")
    scenario_infos = send_request("get_sim_config", node_ros=client) or {}
    start_resp = client.start_logging(data_path, max_queue_size, flush_interval)
    if not start_resp or not start_resp.success:
        shutil.rmtree(run_folder, ignore_errors=True)
        raise RuntimeError("start_logger failed")
    return LoggedRun(
        run_folder=run_folder,
        data_path=data_path,
        scenario_infos=scenario_infos,
        run_number=run_number_from_folder(run_folder),
    )


def end_run(
    client: LoggerClient,
    run: LoggedRun,
    metadata_extra: Optional[Dict[str, Any]] = None,
    *,
    map_name: str = "",
    additional_info: str = "",
    mpc_config: Optional[Dict[str, Any]] = None,
    rosbag: Optional[Dict[str, Any]] = None,
    consolidate: bool = True,
) -> Path:
    """
    Stop logger, write ``metadata.yaml``, optionally consolidate shards.

    Returns path to ``data.pkl`` when *consolidate* is true.
    """
    stop_resp = client.stop_logging()
    if not stop_resp or not stop_resp.success:
        raise RuntimeError("stop_logger failed")

    extra = dict(metadata_extra or {})
    metadata = build_metadata(
        run,
        map_name=extra.get("map_name", map_name),
        additional_info=extra.get("additional_info", additional_info),
        mpc_config=mpc_config,
        rosbag=rosbag,
        extra=extra,
    )
    metadata_path = os.path.join(run.run_folder, "metadata.yaml")
    with open(metadata_path, "w") as f:
        yaml.dump(metadata, f, sort_keys=False)

    if consolidate:
        return consolidate_logger_files(run.data_path)
    return Path(run.data_path)
