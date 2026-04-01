"""Week 9 Frenet helper utilities.

This module keeps the live Frenet validation scaffold in one place.

It provides:
- curvature-profile sampling and Frenet-path integration for Parts 1 and 2
- a reference path built directly from the arrays produced in Parts 1 and 2
- projection of (x, y) to the closest reference state
- extraction of a planar pose from /<vehicle>/reduced_state
- a ROS 2 helper node for PlotJuggler publishing and live state access

"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from typing import Iterable, Optional
import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

try:
    from bng_msgs.msg import FrenetStateMsg, PathSampleMsg, ReducedGtStateMsg
except Exception as exc:  # pragma: no cover
    raise RuntimeError(
        "Week 9 messages are not importable; source the workspace and build bng_msgs first."
    ) from exc


def _wrap_angle_pi(angle_rad: np.ndarray | float) -> np.ndarray | float:
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def _unwrap_angle(angle_rad: Iterable[float]) -> np.ndarray:
    return np.unwrap(np.asarray(list(angle_rad), dtype=float))


def _arc_length_from_xy(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    ds = np.hypot(np.diff(x), np.diff(y))
    return np.concatenate(([0.0], np.cumsum(ds)))


def _yaw_from_xy(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(x)
    if n < 2:
        return np.zeros(n, dtype=float)

    dx = np.zeros(n, dtype=float)
    dy = np.zeros(n, dtype=float)
    dx[1:-1] = x[2:] - x[:-2]
    dy[1:-1] = y[2:] - y[:-2]
    dx[0] = x[1] - x[0]
    dy[0] = y[1] - y[0]
    dx[-1] = x[-1] - x[-2]
    dy[-1] = y[-1] - y[-2]
    return _unwrap_angle(np.arctan2(dy, dx))


def _dedupe_mask(x: np.ndarray, y: np.ndarray, min_dist: float) -> np.ndarray:
    if len(x) < 2:
        return np.ones(len(x), dtype=bool)
    ds = np.hypot(np.diff(x), np.diff(y))
    keep = np.ones(len(x), dtype=bool)
    keep[1:] = ds > float(min_dist)
    return keep


def wrap_angle_pi(angle_rad: np.ndarray | float) -> np.ndarray | float:
    return _wrap_angle_pi(angle_rad)


def _time_to_stamp_msg(time_seconds: float):
    time_seconds = float(time_seconds)
    sec = int(math.floor(time_seconds))
    nanosec = int(round((time_seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000

    stamp = ReducedGtStateMsg().header.stamp
    stamp.sec = sec
    stamp.nanosec = nanosec
    return stamp


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    psi: float
    t: float = 0.0


@dataclass(frozen=True)
class ClosestReferenceState:
    s: float
    x_ref: float
    y_ref: float
    psi_ref: float
    proj_dist: float


@dataclass(frozen=True)
class RefPoint:
    s: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class FrenetPath:
    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    kappa: np.ndarray


def make_piecewise_curvature_profile(
    segment_lengths: Iterable[float],
    curvatures: Iterable[float],
    *,
    ds: float,
) -> tuple[np.ndarray, np.ndarray]:
    segment_lengths_arr = np.asarray(list(segment_lengths), dtype=float)
    curvatures_arr = np.asarray(list(curvatures), dtype=float)
    if len(segment_lengths_arr) != len(curvatures_arr):
        raise ValueError("segment_lengths and curvatures must have the same length")
    if len(segment_lengths_arr) == 0:
        raise ValueError("At least one segment is required")
    if ds <= 0.0:
        raise ValueError("ds must be > 0")
    if np.any(segment_lengths_arr <= 0.0):
        raise ValueError("Every segment length must be > 0")

    total_length = float(np.sum(segment_lengths_arr))
    num_steps = max(1, int(math.ceil(total_length / ds)))
    s = np.linspace(0.0, total_length, num_steps + 1)
    kappa = np.zeros_like(s)

    edges = np.concatenate(([0.0], np.cumsum(segment_lengths_arr)))
    for idx, curvature in enumerate(curvatures_arr):
        s_lo = edges[idx]
        s_hi = edges[idx + 1]
        if idx == len(curvatures_arr) - 1:
            mask = (s >= s_lo) & (s <= s_hi + 1e-12)
        else:
            mask = (s >= s_lo) & (s < s_hi)
        kappa[mask] = curvature

    return s, kappa


def sample_curvature_function(
    s: Iterable[float],
    kappa_of_s,
) -> np.ndarray:
    s_arr = np.asarray(list(s), dtype=float)
    if s_arr.ndim != 1:
        raise ValueError("s must be one-dimensional")
    return np.asarray([float(kappa_of_s(float(si))) for si in s_arr], dtype=float)


def integrate_curvature_profile(
    s: Iterable[float],
    kappa: Iterable[float],
    *,
    x0: float = 0.0,
    y0: float = 0.0,
    yaw0: float = math.pi / 2.0,
) -> FrenetPath:
    s_arr = np.asarray(list(s), dtype=float)
    kappa_arr = np.asarray(list(kappa), dtype=float)

    if s_arr.ndim != 1 or kappa_arr.ndim != 1:
        raise ValueError("s and kappa must be one-dimensional")
    if len(s_arr) != len(kappa_arr):
        raise ValueError("s and kappa must have the same length")
    if len(s_arr) < 2:
        raise ValueError("At least two samples are required")
    if np.any(np.diff(s_arr) <= 0.0):
        raise ValueError("s must be strictly increasing")

    x = np.empty_like(s_arr)
    y = np.empty_like(s_arr)
    yaw = np.empty_like(s_arr)

    x[0] = float(x0)
    y[0] = float(y0)
    yaw[0] = float(yaw0)

    for idx in range(1, len(s_arr)):
        ds = float(s_arr[idx] - s_arr[idx - 1])
        yaw_rate = float(kappa_arr[idx - 1])
        yaw_mid = yaw[idx - 1] + 0.5 * ds * yaw_rate
        yaw[idx] = yaw[idx - 1] + ds * yaw_rate
        x[idx] = x[idx - 1] + ds * math.cos(yaw_mid)
        y[idx] = y[idx - 1] + ds * math.sin(yaw_mid)

    return FrenetPath(
        s=s_arr,
        x=x,
        y=y,
        yaw=np.unwrap(yaw),
        kappa=kappa_arr,
    )


def integrate_frenet_path(
    *,
    s: Iterable[float],
    kappa: Iterable[float],
    x0: float = 0.0,
    y0: float = 0.0,
    psi0: float = math.pi / 2.0,
) -> FrenetPath:
    return integrate_curvature_profile(s=s, kappa=kappa, x0=x0, y0=y0, yaw0=psi0)


def write_path_csv(path: FrenetPath, csv_path: str | Path) -> Path:
    out_path = Path(csv_path).expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["s", "x", "y", "yaw", "curvature"])
        for idx in range(len(path.s)):
            writer.writerow(
                [
                    float(path.s[idx]),
                    float(path.x[idx]),
                    float(path.y[idx]),
                    float(path.yaw[idx]),
                    float(path.kappa[idx]),
                ]
            )

    return out_path


class ArrayReferenceTrajectory:
    """Reference trajectory backed by in-memory arrays."""

    def __init__(
        self,
        *,
        x: Iterable[float],
        y: Iterable[float],
        s: Optional[Iterable[float]] = None,
        yaw: Optional[Iterable[float]] = None,
        kappa: Optional[Iterable[float]] = None,
        closed: bool = False,
        min_point_dist: float = 1e-3,
    ):
        x_raw = np.asarray(list(x), dtype=float)
        y_raw = np.asarray(list(y), dtype=float)
        if x_raw.ndim != 1 or y_raw.ndim != 1 or len(x_raw) != len(y_raw):
            raise ValueError("x and y must be one-dimensional arrays of the same length")
        if len(x_raw) < 2:
            raise ValueError("At least two reference samples are required")

        valid = np.isfinite(x_raw) & np.isfinite(y_raw)
        x_raw = x_raw[valid]
        y_raw = y_raw[valid]
        if len(x_raw) < 2:
            raise ValueError("Not enough finite (x, y) samples remain after filtering")

        keep = _dedupe_mask(x_raw, y_raw, min_dist=min_point_dist)
        x_ref = x_raw[keep]
        y_ref = y_raw[keep]
        if len(x_ref) < 2:
            raise ValueError("Not enough points remain after dedupe")

        s_ref = None
        if s is not None:
            s_arr = np.asarray(list(s), dtype=float)[valid][keep]
            if len(s_arr) == len(x_ref) and np.all(np.diff(s_arr) > 0.0):
                s_ref = s_arr

        yaw_ref = None
        if yaw is not None:
            yaw_arr = np.asarray(list(yaw), dtype=float)[valid][keep]
            if len(yaw_arr) == len(x_ref):
                yaw_ref = _unwrap_angle(yaw_arr)

        kappa_ref = None
        if kappa is not None:
            kappa_arr = np.asarray(list(kappa), dtype=float)[valid][keep]
            if len(kappa_arr) == len(x_ref):
                kappa_ref = kappa_arr

        if s_ref is None:
            s_ref = _arc_length_from_xy(x_ref, y_ref)
        if yaw_ref is None:
            yaw_ref = _yaw_from_xy(x_ref, y_ref)

        self.closed = bool(closed)
        self.s_ref = np.asarray(s_ref, dtype=float)
        self.x_ref = np.asarray(x_ref, dtype=float)
        self.y_ref = np.asarray(y_ref, dtype=float)
        self.yaw_ref = np.asarray(yaw_ref, dtype=float)
        self.kappa_ref = None if kappa_ref is None else np.asarray(kappa_ref, dtype=float)
        self.length = float(self.s_ref[-1])
        if self.length <= 0.0:
            raise ValueError("Reference length must be > 0")

        self._seg_ax = self.x_ref[:-1]
        self._seg_ay = self.y_ref[:-1]
        self._seg_bx = self.x_ref[1:]
        self._seg_by = self.y_ref[1:]
        self._seg_vx = self._seg_bx - self._seg_ax
        self._seg_vy = self._seg_by - self._seg_ay
        self._seg_len2 = self._seg_vx**2 + self._seg_vy**2

    @classmethod
    def from_frenet_path(cls, path: FrenetPath, *, closed: bool = False) -> "ArrayReferenceTrajectory":
        return cls(
            s=path.s,
            x=path.x,
            y=path.y,
            yaw=path.yaw,
            kappa=path.kappa,
            closed=closed,
        )

    def _wrap_s(self, s: float) -> float:
        if not self.closed:
            return float(np.clip(s, self.s_ref[0], self.s_ref[-1]))
        return float(s % self.length)

    def ref_at_s(self, s: float) -> RefPoint:
        s_q = self._wrap_s(float(s))
        x = float(np.interp(s_q, self.s_ref, self.x_ref))
        y = float(np.interp(s_q, self.s_ref, self.y_ref))
        yaw = float(np.interp(s_q, self.s_ref, self.yaw_ref))
        return RefPoint(s=s_q, x=x, y=y, yaw=yaw)

    def closest_s(
        self,
        x: float,
        y: float,
        *,
        last_s: Optional[float] = None,
        s_back: float = 10.0,
        s_fwd: float = 40.0,
    ) -> tuple[float, float, float, float]:
        px = float(x)
        py = float(y)

        if last_s is None or not np.isfinite(last_s):
            i0 = 0
            i1 = len(self._seg_ax)
        else:
            s_center = self._wrap_s(float(last_s)) if self.closed else float(last_s)
            s0 = float(np.clip(s_center - float(s_back), self.s_ref[0], self.s_ref[-1]))
            s1 = float(np.clip(s_center + float(s_fwd), self.s_ref[0], self.s_ref[-1]))
            i0 = int(np.searchsorted(self.s_ref, s0, side="left")) - 1
            i1 = int(np.searchsorted(self.s_ref, s1, side="right"))
            i0 = int(np.clip(i0, 0, len(self._seg_ax) - 1))
            i1 = int(np.clip(i1, i0 + 1, len(self._seg_ax)))

        best_dist = float("inf")
        best_s = float(self.s_ref[i0])
        best_x = float(self.x_ref[i0])
        best_y = float(self.y_ref[i0])

        eps = 1e-12
        for idx in range(i0, i1):
            len2 = float(self._seg_len2[idx])
            if len2 < eps:
                continue

            ax = float(self._seg_ax[idx])
            ay = float(self._seg_ay[idx])
            vx = float(self._seg_vx[idx])
            vy = float(self._seg_vy[idx])

            t = ((px - ax) * vx + (py - ay) * vy) / len2
            t = float(np.clip(t, 0.0, 1.0))

            proj_x = ax + t * vx
            proj_y = ay + t * vy
            dist = float(np.hypot(px - proj_x, py - proj_y))
            if dist < best_dist:
                s_i = float(self.s_ref[idx])
                s_ip1 = float(self.s_ref[idx + 1])
                best_dist = dist
                best_s = s_i + t * (s_ip1 - s_i)
                best_x = proj_x
                best_y = proj_y

        if self.closed:
            best_s = self._wrap_s(best_s)

        return float(best_s), float(best_x), float(best_y), float(best_dist)


def pose_from_reduced_state(msg) -> Pose2D:
    return Pose2D(
        x=float(getattr(msg, "x", 0.0)),
        y=float(getattr(msg, "y", 0.0)),
        psi=float(getattr(msg, "yaw", 0.0)),
        t=float(getattr(msg, "time", 0.0)),
    )


def closest_reference_state(
    ref: ArrayReferenceTrajectory,
    *,
    x: float,
    y: float,
    last_s: Optional[float] = None,
    s_back: float = 10.0,
    s_fwd: float = 40.0,
) -> ClosestReferenceState:
    s_star, x_proj, y_proj, proj_dist = ref.closest_s(
        x,
        y,
        last_s=last_s,
        s_back=s_back,
        s_fwd=s_fwd,
    )
    ref_state = ref.ref_at_s(s_star)
    return ClosestReferenceState(
        s=float(s_star),
        x_ref=float(x_proj),
        y_ref=float(y_proj),
        psi_ref=float(ref_state.yaw),
        proj_dist=float(proj_dist),
    )


class FrenetHelper(Node):
    """Single Week 9 helper for live Frenet validation."""

    def __init__(
        self,
        *,
        vehicle_name: str,
        path: Optional[FrenetPath] = None,
        s: Optional[Iterable[float]] = None,
        x: Optional[Iterable[float]] = None,
        y: Optional[Iterable[float]] = None,
        yaw: Optional[Iterable[float]] = None,
        kappa: Optional[Iterable[float]] = None,
        closed: bool = True,
        state_topic: Optional[str] = None,
        ref_topic_prefix: Optional[str] = None,
        topic_prefix: Optional[str] = None,
        spin_in_thread: bool = False,
    ):
        super().__init__(f"frenet_helper_{vehicle_name}")

        self.vehicle_name = str(vehicle_name)
        self.ref = self._build_reference(path=path, s=s, x=x, y=y, yaw=yaw, kappa=kappa, closed=closed)
        self._state_topic = state_topic or f"/{self.vehicle_name}/reduced_state"
        self._ref_prefix = ref_topic_prefix or f"/{self.vehicle_name}/week9/ref"
        self._topic_prefix = topic_prefix or f"/{self.vehicle_name}/week9/frenet"

        self._lock = threading.Lock()
        self._latest_state_msg: Optional[ReducedGtStateMsg] = None
        self._latest_state_wall_time: float = 0.0

        self._sub_state = self.create_subscription(
            ReducedGtStateMsg,
            self._state_topic,
            self._on_state,
            10,
        )
        self.get_logger().info(f"Subscribed to {self._state_topic}")

        self._pub_path_sample = self.create_publisher(PathSampleMsg, f"{self._ref_prefix}/sample", 10)
        self._pub_frenet_state = self.create_publisher(FrenetStateMsg, f"{self._topic_prefix}/state", 10)

        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spinning = False
        if spin_in_thread:
            self.start_spin()

    @staticmethod
    def _build_reference(
        *,
        path: Optional[FrenetPath],
        s: Optional[Iterable[float]],
        x: Optional[Iterable[float]],
        y: Optional[Iterable[float]],
        yaw: Optional[Iterable[float]],
        kappa: Optional[Iterable[float]],
        closed: bool,
    ) -> ArrayReferenceTrajectory:
        if path is not None:
            return ArrayReferenceTrajectory.from_frenet_path(path, closed=closed)
        if x is None or y is None:
            raise ValueError("Provide either path=FrenetPath or at least x=..., y=... arrays")
        return ArrayReferenceTrajectory(
            s=s,
            x=x,
            y=y,
            yaw=yaw,
            kappa=kappa,
            closed=closed,
        )

    def _on_state(self, msg: ReducedGtStateMsg) -> None:
        with self._lock:
            self._latest_state_msg = msg
            self._latest_state_wall_time = time.time()

    def _reference_stamp(self, *, sample_offset_ns: int = 0):
        state_msg = self.get_latest_state_msg()
        if state_msg is not None:
            if getattr(state_msg.header.stamp, "sec", 0) != 0 or getattr(state_msg.header.stamp, "nanosec", 0) != 0:
                stamp = state_msg.header.stamp
            else:
                stamp = _time_to_stamp_msg(getattr(state_msg, "time", 0.0))
        else:
            stamp = self.get_clock().now().to_msg()

        total_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec) + int(sample_offset_ns)
        adjusted_stamp = ReducedGtStateMsg().header.stamp
        adjusted_stamp.sec = total_ns // 1_000_000_000
        adjusted_stamp.nanosec = total_ns % 1_000_000_000
        return adjusted_stamp

    def publish_reference_path_points(self, *, sample_period: float = 0.01, repeat: int = 1) -> None:
        if sample_period < 0.0:
            raise ValueError("sample_period must be >= 0")
        if repeat < 1:
            raise ValueError("repeat must be >= 1")

        kappa_values = self.ref.kappa_ref
        if kappa_values is None:
            kappa_values = np.full_like(self.ref.s_ref, np.nan, dtype=float)

        for _ in range(int(repeat)):
            for idx in range(len(self.ref.s_ref)):
                sample_msg = PathSampleMsg()
                sample_msg.header.stamp = self._reference_stamp(sample_offset_ns=idx)
                sample_msg.header.frame_id = "map"
                sample_msg.path_name = "week9_reference"
                sample_msg.sample_index = int(idx)
                sample_msg.s = float(self.ref.s_ref[idx])
                sample_msg.x = float(self.ref.x_ref[idx])
                sample_msg.y = float(self.ref.y_ref[idx])
                sample_msg.yaw = float(self.ref.yaw_ref[idx])
                sample_msg.kappa = float(kappa_values[idx])
                self._pub_path_sample.publish(sample_msg)
                if sample_period > 0.0:
                    time.sleep(sample_period)

    def get_latest_state_msg(self) -> Optional[ReducedGtStateMsg]:
        with self._lock:
            return self._latest_state_msg

    def get_latest_pose(self) -> Optional[Pose2D]:
        msg = self.get_latest_state_msg()
        if msg is None:
            return None
        return pose_from_reduced_state(msg)

    def get_latest_state_age(self) -> float:
        with self._lock:
            if self._latest_state_wall_time <= 0.0:
                return float("inf")
            return float(time.time() - self._latest_state_wall_time)

    def closest_reference_state(
        self,
        *,
        x: float,
        y: float,
        last_s: Optional[float] = None,
        s_back: float = 10.0,
        s_fwd: float = 40.0,
    ) -> ClosestReferenceState:
        return closest_reference_state(
            self.ref,
            x=x,
            y=y,
            last_s=last_s,
            s_back=s_back,
            s_fwd=s_fwd,
        )

    def report_frenet(
        self,
        *,
        s: float,
        e: float,
        dphi: float,
        proj_dist: Optional[float] = None,
        x_ref: Optional[float] = None,
        y_ref: Optional[float] = None,
        psi_ref: Optional[float] = None,
    ) -> None:
        state_src_msg = self.get_latest_state_msg()
        pose = pose_from_reduced_state(state_src_msg) if state_src_msg is not None else None

        state_msg = FrenetStateMsg()
        if state_src_msg is not None:
            if getattr(state_src_msg.header.stamp, "sec", 0) != 0 or getattr(state_src_msg.header.stamp, "nanosec", 0) != 0:
                state_msg.header.stamp = state_src_msg.header.stamp
            else:
                state_msg.header.stamp = _time_to_stamp_msg(getattr(state_src_msg, "time", 0.0))
        else:
            state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = "map"
        state_msg.path_name = "week9_reference"
        state_msg.s = float(s)
        state_msg.e = float(e)
        state_msg.dphi = float(dphi)
        state_msg.proj_dist = float(proj_dist) if proj_dist is not None else float("nan")
        state_msg.x_ref = float(x_ref) if x_ref is not None else float("nan")
        state_msg.y_ref = float(y_ref) if y_ref is not None else float("nan")
        state_msg.yaw_ref = float(_wrap_angle_pi(psi_ref)) if psi_ref is not None else float("nan")

        if pose is not None:
            state_msg.x_vehicle = float(pose.x)
            state_msg.y_vehicle = float(pose.y)
            state_msg.yaw_vehicle = float(_wrap_angle_pi(pose.psi))
        else:
            state_msg.x_vehicle = float("nan")
            state_msg.y_vehicle = float("nan")
            state_msg.yaw_vehicle = float("nan")

        self._pub_frenet_state.publish(state_msg)

    def start_spin(self) -> None:
        if self._spinning:
            return
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spinning = True

        def _run() -> None:
            while self._spinning and rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)

        self._spin_thread = threading.Thread(target=_run, daemon=True)
        self._spin_thread.start()

    def stop_spin(self) -> None:
        self._spinning = False
        if self._executor is not None:
            try:
                self._executor.remove_node(self)
            except Exception:
                pass
        self._executor = None
        self._spin_thread = None
