"""Week 11 tracking and Frenet conversion helpers.

These utilities sit between the week10 trajectory generator and the week11
simulator / MPC code.

Assignment-facing entry points:
- wrap_to_pi(angle): wrap an angle or angle array to [-pi, pi).
- TrackingReference.from_track(...): build a geometric reference from a track.
- TrackingReference.from_traj_result(...): build a full reference with state
    profiles from the week10 trajectory optimization result.
- TrackingReference.ref_at_s(s): sample one reference point at a single path
    position s.
- TrackingReference.get_ref_traj(s_start, horizon_steps, ds): sample a finite
    horizon reference trajectory starting at the current s and spaced on a fixed
    ds grid.
- TrackingReference.cartesian_to_frenet(...): project the measured vehicle pose
    onto the reference and compute Frenet coordinates (s, e, dphi).
- TrackingReference.frenet_to_cartesian(...): map predicted Frenet states back
    to Cartesian x, y, and heading for plotting.

Typical assignment flow:
1. Build a TrackingReference from either a track or a week10 trajectory.
2. Convert each noisy Cartesian simulator measurement to Frenet coordinates.
3. Sample the future reference along s for the MPC horizon.
4. Convert predicted Frenet rollouts back to Cartesian coordinates for
     visualization.

The key contract is:
- keep the reference yaw unwrapped for interpolation and trajectory work
- if a measured vehicle yaw is wrapped, wrap only the difference
- use previewed curvature as the receding-horizon parameter sequence
- keep track curvature and optimized-trajectory curvature distinct when those
    two reference conventions need to coexist
"""

from __future__ import annotations

import importlib
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np


_THIS_DIR = Path(__file__).resolve().parent
_WEEK10_DIR = _THIS_DIR.parent / "week10"
if str(_WEEK10_DIR) not in sys.path:
    sys.path.insert(0, str(_WEEK10_DIR))

_week10_traj = importlib.import_module("traj_opt_helper")

FrenetPath = _week10_traj.FrenetPath
TrackDefinition = _week10_traj.TrackDefinition


def wrap_to_pi(angle: float | np.ndarray) -> float | np.ndarray:
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi

@dataclass(frozen=True)
class FrenetMeasurement:
    s: float
    e: float
    dphi: float
    x_ref: float
    y_ref: float
    psi_ref: float
    kappa_ref: float
    e_min: float
    e_max: float
    proj_dist: float


class TrackingReference:
    """Reference path and preview utilities for week11 tracking."""

    PROFILE_KEYS = (
        "t",
        "r",
        "V",
        "beta",
        "wr",
        "e",
        "dphi",
        "delta",
        "rear_wheel_torque",
    )

    def __init__(
        self,
        *,
        s: np.ndarray,
        x: np.ndarray,
        y: np.ndarray,
        yaw: np.ndarray,
        kappa: np.ndarray,
        speed: np.ndarray,
        e_min: np.ndarray,
        e_max: np.ndarray,
        state_profiles: dict[str, np.ndarray],
        frame_x: Optional[np.ndarray] = None,
        frame_y: Optional[np.ndarray] = None,
        frame_yaw: Optional[np.ndarray] = None,
        frame_kappa: Optional[np.ndarray] = None,
        kappa_track: Optional[np.ndarray] = None,
        kappa_traj: Optional[np.ndarray] = None,
        reference_mode: str = "trajectory",
        closed: bool = False,
    ):
        self.s = np.asarray(s, dtype=float)
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        self.yaw = np.unwrap(np.asarray(yaw, dtype=float))
        self.frame_x = np.asarray(self.x if frame_x is None else frame_x, dtype=float)
        self.frame_y = np.asarray(self.y if frame_y is None else frame_y, dtype=float)
        self.frame_yaw = np.unwrap(np.asarray(self.yaw if frame_yaw is None else frame_yaw, dtype=float))
        self.kappa = np.asarray(kappa if frame_kappa is None else frame_kappa, dtype=float)
        self.kappa_track = np.asarray(self.kappa if kappa_track is None else kappa_track, dtype=float)
        self.kappa_traj = np.asarray(kappa if kappa_traj is None else kappa_traj, dtype=float)
        self.speed = np.asarray(speed, dtype=float)
        self.e_min = np.asarray(e_min, dtype=float)
        self.e_max = np.asarray(e_max, dtype=float)
        self.reference_mode = str(reference_mode)
        self.state_profiles = {
            name: np.asarray(state_profiles[name], dtype=float)
            for name in self.PROFILE_KEYS
        }
        self.closed = bool(closed)
        self.length = float(self.s[-1] - self.s[0])
        self.path = FrenetPath(s=self.s, x=self.frame_x, y=self.frame_y, yaw=self.frame_yaw, kappa=self.kappa)

        self._seg_ax = self.frame_x[:-1]
        self._seg_ay = self.frame_y[:-1]
        self._seg_vx = self.frame_x[1:] - self.frame_x[:-1]
        self._seg_vy = self.frame_y[1:] - self.frame_y[:-1]
        self._seg_len2 = self._seg_vx ** 2 + self._seg_vy ** 2

    @classmethod
    def from_track(
        cls,
        track: TrackDefinition,
        *,
        speed: np.ndarray,
        ds: Optional[float] = None,
        closed: bool = False,
    ) -> "TrackingReference":
        ds_use = ds
        if ds_use is None:
            ds_use = float(np.median(np.diff(track.s))) if len(track.s) > 1 else 1.0
        path = track.get_path(ds=max(ds_use, 0.1))
        e_min, e_max = track.e_bounds_at(path.s)
        speed_arr = np.asarray(speed, dtype=float)
        if speed_arr.ndim == 0:
            speed_arr = np.full_like(path.s, float(speed_arr), dtype=float)
        ds_path = np.diff(path.s)
        t_ref = np.zeros_like(path.s)
        t_ref[1:] = np.cumsum(ds_path / speed_arr[1:])
        return cls(
            s=path.s,
            x=path.x,
            y=path.y,
            yaw=path.yaw,
            kappa=path.kappa,
            speed=speed_arr,
            e_min=e_min,
            e_max=e_max,
            state_profiles={
                "t": t_ref,
                "r": speed_arr * path.kappa,
                "V": speed_arr,
                "beta": np.zeros_like(path.s),
                "wr": speed_arr,
                "e": np.zeros_like(path.s),
                "dphi": np.zeros_like(path.s),
                "delta": np.zeros_like(path.s),
                "rear_wheel_torque": np.zeros_like(path.s),
            },
            closed=closed,
        )

    @classmethod
    def from_traj_result(
        cls,
        result: Any,
        *,
        reference_mode: str = "centerline",
        closed: bool = False,
    ) -> "TrackingReference":
        s_arr = np.asarray(result.s, dtype=float)
        zero_profile = np.zeros_like(s_arr, dtype=float)
        x_traj, y_traj, body_yaw = result.get_xy_trajectory()
        velocity_heading = np.unwrap(np.asarray(body_yaw, dtype=float) + np.asarray(result.beta, dtype=float))
        x_center, y_center, yaw_center = result.path_ref.frenet_to_cartesian(s_arr, zero_profile, zero_profile)
        e_min, e_max = result.track.e_bounds_at(result.s)
        kappa_track = np.asarray(result.track.kappa_at(s_arr), dtype=float)
        kappa_traj = np.gradient(velocity_heading, s_arr, edge_order=1)

        mode = str(reference_mode).lower()
        if mode == "centerline":
            ref_e = np.asarray(result.e, dtype=float)
            ref_dphi = np.asarray(result.dphi, dtype=float)
            frame_x = x_center
            frame_y = y_center
            frame_yaw = yaw_center
            frame_kappa = kappa_track
            e_min_use = e_min
            e_max_use = e_max
        elif mode == "trajectory":
            ref_e = zero_profile
            ref_dphi = zero_profile
            frame_x = np.asarray(x_traj, dtype=float)
            frame_y = np.asarray(y_traj, dtype=float)
            frame_yaw = velocity_heading
            frame_kappa = kappa_traj
            e_shift = np.asarray(result.e, dtype=float)
            e_min_use = e_min - e_shift
            e_max_use = e_max - e_shift
        else:
            raise ValueError("reference_mode must be either 'centerline' or 'trajectory'")

        return cls(
            s=s_arr,
            x=np.asarray(x_traj, dtype=float),
            y=np.asarray(y_traj, dtype=float),
            yaw=velocity_heading,
            kappa=kappa_traj,
            speed=np.asarray(result.V, dtype=float),
            e_min=e_min_use,
            e_max=e_max_use,
            state_profiles={
                "t": np.asarray(result.t, dtype=float),
                "r": np.asarray(result.r, dtype=float),
                "V": np.asarray(result.V, dtype=float),
                "beta": np.asarray(result.beta, dtype=float),
                "wr": np.asarray(result.wr, dtype=float),
                "e": ref_e,
                "dphi": ref_dphi,
                "delta": np.asarray(result.delta, dtype=float),
                "rear_wheel_torque": np.asarray(result.rear_wheel_torque, dtype=float),
            },
            frame_x=frame_x,
            frame_y=frame_y,
            frame_yaw=frame_yaw,
            frame_kappa=frame_kappa,
            kappa_track=kappa_track,
            kappa_traj=kappa_traj,
            reference_mode=mode,
            closed=closed,
        )

    def as_frenet_path(self) -> FrenetPath:
        return self.path

    def frenet_to_cartesian(
        self,
        s_vec: np.ndarray,
        e_vec: np.ndarray,
        dphi_vec: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        s_arr = np.asarray(s_vec, dtype=float)
        e_arr = np.asarray(e_vec, dtype=float)
        dphi_arr = np.asarray(dphi_vec, dtype=float)
        if not (s_arr.shape == e_arr.shape == dphi_arr.shape):
            raise ValueError("s_vec, e_vec, and dphi_vec must have the same shape")
        s_wrapped = np.asarray(self.wrap_s(s_arr), dtype=float)
        return self.path.frenet_to_cartesian(s_wrapped, e_arr, dphi_arr)

    def wrap_s(self, s_query: float | np.ndarray) -> float | np.ndarray:
        if not self.closed:
            return np.clip(s_query, self.s[0], self.s[-1])
        return (np.asarray(s_query) - self.s[0]) % self.length + self.s[0]

    def sample(self, s_query: float | np.ndarray) -> dict[str, np.ndarray]:
        s_in = np.asarray(s_query, dtype=float)
        s_eval = np.asarray(self.wrap_s(s_in), dtype=float)
        out = {
            "s": s_eval,
            "x": np.interp(s_eval, self.s, self.x),
            "y": np.interp(s_eval, self.s, self.y),
            "yaw": np.interp(s_eval, self.s, self.yaw),
            "frame_x": np.interp(s_eval, self.s, self.frame_x),
            "frame_y": np.interp(s_eval, self.s, self.frame_y),
            "frame_yaw": np.interp(s_eval, self.s, self.frame_yaw),
            "kappa": np.interp(s_eval, self.s, self.kappa),
            "kappa_track": np.interp(s_eval, self.s, self.kappa_track),
            "kappa_traj": np.interp(s_eval, self.s, self.kappa_traj),
            "speed": np.interp(s_eval, self.s, self.speed),
            "V": np.interp(s_eval, self.s, self.state_profiles["V"]),
            "e_min": np.interp(s_eval, self.s, self.e_min),
            "e_max": np.interp(s_eval, self.s, self.e_max),
        }
        for name in self.PROFILE_KEYS:
            if name == "V":
                continue
            out[name] = np.interp(s_eval, self.s, self.state_profiles[name])
        return out

    def ref_at_s(self, s_query: float) -> dict[str, float]:
        s_eval = np.asarray(self.wrap_s(np.array([s_query], dtype=float)), dtype=float)
        return {
            "s": float(s_eval[0]),
            "x": float(np.interp(s_eval[0], self.s, self.frame_x)),
            "y": float(np.interp(s_eval[0], self.s, self.frame_y)),
            "yaw": float(np.interp(s_eval[0], self.s, self.frame_yaw)),
            "kappa": float(np.interp(s_eval[0], self.s, self.kappa)),
            "e_min": float(np.interp(s_eval[0], self.s, self.e_min)),
            "e_max": float(np.interp(s_eval[0], self.s, self.e_max)),
        }

    def get_ref_traj(self, s_start: float, horizon_steps: int, ds: float) -> dict[str, np.ndarray]:
        s_grid = float(ds) * np.arange(int(horizon_steps) + 1, dtype=float)
        s_unwrapped = float(s_start) + s_grid
        sampled = self.sample(s_unwrapped)
        sampled["s_wrapped"] = np.array(sampled["s"], copy=True)
        sampled["s"] = s_unwrapped
        sampled["s_grid"] = s_grid
        sampled["s_unwrapped"] = s_unwrapped
        return sampled

    def get_preview(self, s_start: float, horizon_steps: int, ds: float) -> dict[str, np.ndarray]:
        return self.get_ref_traj(s_start, horizon_steps, ds)

    def closest_s(
        self,
        x_query: float,
        y_query: float,
        *,
        last_s: Optional[float] = None,
        s_back: float = 10.0,
        s_fwd: float = 40.0,
    ) -> tuple[float, float, float, float]:
        px = float(x_query)
        py = float(y_query)

        if last_s is None or not np.isfinite(last_s):
            i0 = 0
            i1 = len(self._seg_ax)
        else:
            s_center = float(np.asarray(self.wrap_s(last_s)))
            s0 = float(np.clip(s_center - s_back, self.s[0], self.s[-1]))
            s1 = float(np.clip(s_center + s_fwd, self.s[0], self.s[-1]))
            i0 = int(np.searchsorted(self.s, s0, side="left")) - 1
            i1 = int(np.searchsorted(self.s, s1, side="right"))
            i0 = int(np.clip(i0, 0, len(self._seg_ax) - 1))
            i1 = int(np.clip(i1, i0 + 1, len(self._seg_ax)))

        best_dist = float("inf")
        best_s = float(self.s[i0])
        best_x = float(self.x[i0])
        best_y = float(self.y[i0])

        for idx in range(i0, i1):
            len2 = float(self._seg_len2[idx])
            if len2 <= 1e-12:
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
                s_i = float(self.s[idx])
                s_ip1 = float(self.s[idx + 1])
                best_dist = dist
                best_s = s_i + t * (s_ip1 - s_i)
                best_x = proj_x
                best_y = proj_y

        return float(np.asarray(self.wrap_s(best_s))), best_x, best_y, best_dist

    def cartesian_to_frenet(
        self,
        *,
        x: float,
        y: float,
        psi: float,
        beta: Optional[float] = None,
        last_s: Optional[float] = None,
        s_back: float = 10.0,
        s_fwd: float = 40.0,
    ) -> FrenetMeasurement:
        s_star, x_ref, y_ref, proj_dist = self.closest_s(
            x,
            y,
            last_s=last_s,
            s_back=s_back,
            s_fwd=s_fwd,
        )
        ref = self.ref_at_s(s_star)
        dx = float(x) - x_ref
        dy = float(y) - y_ref
        psi_ref = ref["yaw"]

        e = -dx * math.sin(psi_ref) + dy * math.cos(psi_ref)
        velocity_heading = float(psi) if beta is None else float(psi) + float(beta)
        dphi = float(wrap_to_pi(velocity_heading - psi_ref))
        return FrenetMeasurement(
            s=s_star,
            e=float(e),
            dphi=dphi,
            x_ref=x_ref,
            y_ref=y_ref,
            psi_ref=psi_ref,
            kappa_ref=ref["kappa"],
            e_min=ref["e_min"],
            e_max=ref["e_max"],
            proj_dist=proj_dist,
        )


__all__ = [
    "FrenetMeasurement",
    "FrenetPath",
    "TrackDefinition",
    "TrackingReference",
    "wrap_to_pi",
]