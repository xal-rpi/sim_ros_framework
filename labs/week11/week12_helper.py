"""Week 12 helper utilities for the BeamNG closed-loop tracking assignment.

Students should focus on the control logic in their own starter file rather
than on thread bookkeeping, ROS topic boilerplate, or live-visualizer process
management.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Optional

import numpy as np
from builtin_interfaces.msg import Time
from rclpy.node import Node
from std_msgs.msg import Float64

from bng_msgs.msg import FrenetStateMsg, MPCSolution, PathSampleMsg
from mpc_visualizer import (
    MPCVisualizerConfig,
    build_snapshot_payload,
    launch_visualizer_process,
    publish_latest_snapshot,
    stop_visualizer_process,
)
from tracking_helper import TrackingReference


class ControllerRuntime:
    """Minimal shared state for the actuation and MPC workers."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._state_lock = threading.Lock()
        self._plan_lock = threading.Lock()
        self._control_lock = threading.Lock()
        self._error_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._latest_raw: tuple[int, dict[str, float]] | None = None
        self._latest_projected: dict[str, Any] | None = None
        self._raw_seq = 0
        self._plan: dict[str, Any] | None = None
        self._applied_control = {
            "throttle": 0.0,
            "steering": 0.0,
            "roadwheel_angle": 0.0,
            "rear_wheel_torque": 0.0,
            "rear_wheelspeed_target": 0.0,
        }
        self._errors: list[tuple[str, Exception]] = []

    def publish_raw_state(self, state: dict[str, float]) -> int:
        with self._condition:
            self._raw_seq += 1
            self._latest_raw = (self._raw_seq, dict(state))
            self._condition.notify_all()
            return self._raw_seq

    def wait_for_raw_state(self, last_seq: int, timeout: float = 0.1) -> tuple[int, dict[str, float]] | None:
        with self._condition:
            while not self._stop_event.is_set():
                if self._latest_raw is not None and self._latest_raw[0] > last_seq:
                    seq, state = self._latest_raw
                    return seq, dict(state)
                self._condition.wait(timeout=timeout)
            return None

    def publish_projected_state(self, projected: dict[str, Any]) -> None:
        with self._condition:
            self._latest_projected = dict(projected)
            self._condition.notify_all()

    def wait_for_projected_state(self, last_seq: int, timeout: float = 0.1) -> dict[str, Any] | None:
        with self._condition:
            while not self._stop_event.is_set():
                projected = self._latest_projected
                if projected is not None and int(projected["seq"]) > last_seq:
                    return dict(projected)
                self._condition.wait(timeout=timeout)
            return None

    def set_plan(self, plan: dict[str, Any]) -> None:
        with self._plan_lock:
            self._plan = dict(plan)

    def get_plan(self) -> dict[str, Any] | None:
        with self._plan_lock:
            if self._plan is None:
                return None
            return dict(self._plan)

    def set_applied_control(self, control: dict[str, float]) -> None:
        with self._control_lock:
            self._applied_control = dict(control)

    def get_applied_control(self) -> dict[str, float]:
        with self._control_lock:
            return dict(self._applied_control)

    def record_error(self, name: str, exc: Exception) -> None:
        with self._error_lock:
            self._errors.append((name, exc))
        self.stop()

    def get_first_error(self) -> tuple[str, Exception] | None:
        with self._error_lock:
            if not self._errors:
                return None
            return self._errors[0]

    def stop(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()

    def should_stop(self) -> bool:
        return self._stop_event.is_set()


class Week12Logger(Node):
    """ROS publisher helper for PlotJuggler and rosbag logging."""

    MPC_TRAJ_FIELDS = (
        "t",
        "r",
        "V",
        "beta",
        "wr",
        "e",
        "dphi",
        "roadwheel_angle",
        "rear_wheel_torque",
    )

    def __init__(self, vehicle_name: str = "EGO") -> None:
        super().__init__(f"week12_logger_{vehicle_name.lower()}")
        self.vehicle_name = str(vehicle_name)
        self._ref_topic = f"/{self.vehicle_name}/week12/ref/sample"
        self._frenet_topic = f"/{self.vehicle_name}/week12/frenet/state"
        self._mpc_topic = f"/{self.vehicle_name}/week12/mpc/solution"
        self._control_prefix = f"/{self.vehicle_name}/week12/control"

        self._pub_ref = self.create_publisher(PathSampleMsg, self._ref_topic, 10)
        self._pub_frenet = self.create_publisher(FrenetStateMsg, self._frenet_topic, 10)
        self._pub_mpc = self.create_publisher(MPCSolution, self._mpc_topic, 10)
        self._pub_control = {
            "throttle": self.create_publisher(Float64, f"{self._control_prefix}/throttle", 10),
            "steering": self.create_publisher(Float64, f"{self._control_prefix}/steering", 10),
            "roadwheel_angle": self.create_publisher(Float64, f"{self._control_prefix}/roadwheel_angle", 10),
            "rear_wheel_torque": self.create_publisher(Float64, f"{self._control_prefix}/rear_wheel_torque", 10),
            "rear_wheelspeed_target": self.create_publisher(Float64, f"{self._control_prefix}/rear_wheelspeed_target", 10),
            "solve_time": self.create_publisher(Float64, f"{self._control_prefix}/solve_time", 10),
        }

    def _stamp_from_time(self, time_sec: float | None) -> Time:
        if time_sec is None or not np.isfinite(float(time_sec)):
            return self.get_clock().now().to_msg()
        time_f = float(time_sec)
        stamp = Time()
        stamp.sec = int(time_f)
        stamp.nanosec = int((time_f - stamp.sec) * 1e9)
        return stamp

    def publish_reference_path_points(
        self,
        reference: TrackingReference,
        *,
        sample_period: float = 0.01,
        repeat: int = 1,
        path_name: str = "week12_reference",
    ) -> None:
        kappa_values = np.asarray(reference.kappa, dtype=float)
        for _ in range(int(repeat)):
            for idx in range(len(reference.s)):
                msg = PathSampleMsg()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.path_name = path_name
                msg.sample_index = int(idx)
                msg.s = float(reference.s[idx])
                msg.x = float(reference.x[idx])
                msg.y = float(reference.y[idx])
                msg.yaw = float(reference.yaw[idx])
                msg.kappa = float(kappa_values[idx])
                self._pub_ref.publish(msg)
                if sample_period > 0.0:
                    time.sleep(sample_period)

    def publish_frenet_state(
        self,
        *,
        sim_time: float,
        s: float,
        e: float,
        dphi: float,
        x_vehicle: float,
        y_vehicle: float,
        yaw_vehicle: float,
        x_ref: float,
        y_ref: float,
        yaw_ref: float,
        proj_dist: float,
        path_name: str = "week12_reference",
    ) -> None:
        msg = FrenetStateMsg()
        msg.header.stamp = self._stamp_from_time(sim_time)
        msg.header.frame_id = "map"
        msg.path_name = path_name
        msg.s = float(s)
        msg.e = float(e)
        msg.dphi = float(dphi)
        msg.proj_dist = float(proj_dist)
        msg.x_vehicle = float(x_vehicle)
        msg.y_vehicle = float(y_vehicle)
        msg.yaw_vehicle = float(yaw_vehicle)
        msg.x_ref = float(x_ref)
        msg.y_ref = float(y_ref)
        msg.yaw_ref = float(yaw_ref)
        self._pub_frenet.publish(msg)

    def publish_control(self, **signals: float) -> None:
        for key, value in signals.items():
            if key in self._pub_control:
                self._pub_control[key].publish(Float64(data=float(value)))

    def publish_mpc_solution(self, solution: dict[str, Any], solve_time: float, current_time: float) -> None:
        x_pred = solution["x_pred"]
        u_pred = solution["u_pred"]
        horizon_steps = len(np.asarray(u_pred["roadwheel_angle"], dtype=float))

        traj: list[float] = []
        for stage in range(horizon_steps):
            traj.extend(
                [
                    float(np.asarray(x_pred["t"], dtype=float)[stage]),
                    float(np.asarray(x_pred["r"], dtype=float)[stage]),
                    float(np.asarray(x_pred["V"], dtype=float)[stage]),
                    float(np.asarray(x_pred["beta"], dtype=float)[stage]),
                    float(np.asarray(x_pred["wr"], dtype=float)[stage]),
                    float(np.asarray(x_pred["e"], dtype=float)[stage]),
                    float(np.asarray(x_pred["dphi"], dtype=float)[stage]),
                    float(np.asarray(u_pred["roadwheel_angle"], dtype=float)[stage]),
                    float(np.asarray(u_pred["rear_wheel_torque"], dtype=float)[stage]),
                ]
            )

        msg = MPCSolution()
        msg.traj = traj
        msg.total_cost = float(solution.get("cost", float("nan")))
        msg.solver_status = 1
        msg.solve_time = float(solve_time)
        msg.curr_time = float(current_time)
        self._pub_mpc.publish(msg)


def sample_plan(plan: dict[str, Any], s_query: float) -> dict[str, float]:
    s_state = float(np.clip(s_query, plan["state_s"][0], plan["state_s"][-1]))
    s_control = float(np.clip(s_query, plan["control_s"][0], plan["control_s"][-1]))
    return {
        "roadwheel_angle": float(np.interp(s_control, plan["control_s"], plan["roadwheel_angle"])),
        "rear_wheel_torque": float(np.interp(s_control, plan["control_s"], plan["rear_wheel_torque"])),
        "rear_wheelspeed_ms": float(np.interp(s_state, plan["state_s"], plan["rear_wheelspeed_ms"])),
    }


def build_plan(
    reference: TrackingReference,
    projected: dict[str, Any],
    solution: dict[str, Any],
    applied_control: dict[str, float],
    solve_time: float,
) -> dict[str, Any]:
    x_pred = solution["x_pred"]
    u_pred = solution["u_pred"]
    state_s = np.array(x_pred["s"], copy=True)
    control_s = np.array(state_s[:-1], copy=True)
    x_roll, y_roll, chi_roll = reference.frenet_to_cartesian(x_pred["s"], x_pred["e"], x_pred["dphi"])
    psi_roll = chi_roll - np.asarray(x_pred["beta"], dtype=float)

    record = {
        "time": projected["current_state"]["time"],
        "solve_time": float(solve_time),
        "measured_state": dict(projected["raw_state"]),
        "frenet": {
            "s": projected["s"],
            "e": projected["e"],
            "dphi": projected["dphi"],
        },
        "control": dict(applied_control),
        "ref_traj": {key: np.array(value, copy=True) for key, value in solution["ref_traj"].items()},
        "predicted_control": {
            "s": np.array(control_s, copy=True),
            **{key: np.array(value, copy=True) for key, value in u_pred.items()},
        },
        "predicted_cartesian": {
            "x": np.array(x_roll, copy=True),
            "y": np.array(y_roll, copy=True),
            "psi": np.array(psi_roll, copy=True),
        },
        "predicted_state": {key: np.array(value, copy=True) for key, value in x_pred.items()},
        "vehicle_pose": {
            "x": float(projected["raw_state"]["x"]),
            "y": float(projected["raw_state"]["y"]),
            "psi": float(projected["raw_state"]["yaw"]),
            "delta": float(projected["raw_state"]["delta"]),
        },
    }

    return {
        "source_seq": projected["seq"],
        "source_sim_time": projected["sim_time"],
        "state_s": state_s,
        "control_s": control_s,
        "roadwheel_angle": np.array(u_pred["roadwheel_angle"], copy=True),
        "rear_wheel_torque": np.array(u_pred["rear_wheel_torque"], copy=True),
        "rear_wheelspeed_ms": np.array(x_pred["rear_wheelspeed_ms"], copy=True),
        "record": record,
    }


def start_live_visualizer(
    reference: TrackingReference,
    *,
    cfg: Optional[MPCVisualizerConfig] = None,
) -> dict[str, Any]:
    vis_cfg = cfg or MPCVisualizerConfig(update_interval_ms=50, max_history=1500)
    process, snapshot_queue = launch_visualizer_process(reference, cfg=vis_cfg)
    return {
        "process": process,
        "queue": snapshot_queue,
        "build_snapshot": build_snapshot_payload,
        "publish": publish_latest_snapshot,
        "stop": stop_visualizer_process,
    }


def publish_live_visualizer(visualizer: dict[str, Any] | None, record: dict[str, Any]) -> None:
    if visualizer is None:
        return
    visualizer["publish"](visualizer["queue"], visualizer["build_snapshot"](record))


def stop_live_visualizer(visualizer: dict[str, Any] | None) -> None:
    if visualizer is None:
        return
    visualizer["stop"](visualizer["process"], visualizer["queue"])
