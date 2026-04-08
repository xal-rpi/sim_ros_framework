"""Week 11 visualization helpers for receding-horizon tracking."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import get_backend
from matplotlib.transforms import Affine2D

from simulator import fiala_params


@dataclass(frozen=True)
class MPCVisualizationConfig:
    vehicle_stride: int = 6
    rollout_stride: int = 2
    vehicle_scale: float = 1.5
    rollout_alpha: float = 0.18
    figsize_2d: tuple[int, int] = (12, 10)
    figsize_states: tuple[int, int] = (12, 8)
    live_figsize: tuple[int, int] = (11, 9)
    live_pause: float = 0.001


class VehicleDrawer:
    def __init__(self, veh: dict, scale: float = 1.0):
        self.a = veh["cogToFrontAxle"] * scale
        self.b = veh["cogToRearAxle"] * scale
        self.hw = veh.get("track_width", 1.787) * 0.5 * scale
        self.wl = 0.6 * scale
        self.ww = 0.2 * scale

    def draw(self, ax, x, y, yaw, steer, color="black", alpha=1.0):
        c, s = np.cos(yaw), np.sin(yaw)
        wheel_yaw = yaw + steer
        pts = {
            "fl": (x + self.a * c - self.hw * s, y + self.a * s + self.hw * c),
            "fr": (x + self.a * c + self.hw * s, y + self.a * s - self.hw * c),
            "rl": (x - self.b * c - self.hw * s, y - self.b * s + self.hw * c),
            "rr": (x - self.b * c + self.hw * s, y - self.b * s - self.hw * c),
        }
        ax.plot([x - self.b * c, x + self.a * c], [y - self.b * s, y + self.a * s], color=color, lw=2.5, alpha=alpha)
        ax.plot(*zip(pts["fl"], pts["fr"]), color=color, lw=1.7, alpha=alpha)
        ax.plot(*zip(pts["rl"], pts["rr"]), color=color, lw=1.7, alpha=alpha)

        def _wheel(cx, cy, angle):
            wheel = patches.Rectangle(
                (-self.wl / 2, -self.ww / 2),
                self.wl,
                self.ww,
                fc="black",
                ec="none",
                alpha=alpha,
                zorder=6,
            )
            wheel.set_transform(Affine2D().rotate(angle).translate(cx, cy) + ax.transData)
            ax.add_patch(wheel)

        _wheel(*pts["fl"], wheel_yaw)
        _wheel(*pts["fr"], wheel_yaw)
        _wheel(*pts["rl"], yaw)
        _wheel(*pts["rr"], yaw)


class LiveMPCAnimator:
    """Simple live visualization for the receding-horizon controller.

    The left panel shows:
    - the reference path
    - the closed-loop path so far
    - the current predicted MPC horizon
    - the current vehicle outline

    The right panels track key quantities through the run so students can see
    how the controller evolves while the animation is playing.
    """

    def __init__(self, reference, *, cfg: Optional[MPCVisualizationConfig] = None):
        self.reference = reference
        self.cfg = cfg or MPCVisualizationConfig()
        self.drawer = VehicleDrawer(fiala_params["vehicle"], scale=self.cfg.vehicle_scale)
        self._supports_pause = "agg" not in get_backend().lower()

        self._history: list[dict[str, float]] = []
        self._dynamic_artists: list[Any] = []

        plt.ion()
        self.fig = plt.figure(figsize=self.cfg.live_figsize)
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1.7, 1.0], height_ratios=[1.0, 1.0])
        self.ax_xy = self.fig.add_subplot(gs[:, 0])
        self.ax_frenet = self.fig.add_subplot(gs[0, 1])
        self.ax_ctrl = self.fig.add_subplot(gs[1, 1])

        x_left, y_left, _ = reference.frenet_to_cartesian(reference.s, reference.e_max, np.zeros_like(reference.s))
        x_right, y_right, _ = reference.frenet_to_cartesian(reference.s, reference.e_min, np.zeros_like(reference.s))
        self.ax_xy.plot(reference.x, reference.y, "k--", lw=1.2, label="reference")
        self.ax_xy.plot(x_left, y_left, color="0.35", ls="--", lw=1.0, alpha=0.8, label="left bound")
        self.ax_xy.plot(x_right, y_right, color="0.35", ls="--", lw=1.0, alpha=0.8, label="right bound")
        self.ax_xy.set_aspect("equal")
        self.ax_xy.set_xlabel("x [m]")
        self.ax_xy.set_ylabel("y [m]")
        self.ax_xy.set_title("Live MPC Receding Horizon")
        self.ax_xy.grid(True, alpha=0.3)

        (self.path_line,) = self.ax_xy.plot([], [], color="tab:blue", lw=2.0, label="closed-loop")
        (self.horizon_line,) = self.ax_xy.plot([], [], color="tab:orange", lw=1.8, alpha=0.9, label="active horizon")
        self.start_marker = self.ax_xy.scatter([], [], color="tab:green", s=65, zorder=7, label="start")
        self.current_marker = self.ax_xy.scatter([], [], color="tab:red", s=65, zorder=7, label="current")
        self.ax_xy.legend(loc="best")

        (self.e_line,) = self.ax_frenet.plot([], [], label="e(s)", lw=1.8)
        (self.dphi_line,) = self.ax_frenet.plot([], [], label="dphi(s)", lw=1.8)
        self.ax_frenet.set_xlabel("s [m]")
        self.ax_frenet.set_ylabel("state")
        self.ax_frenet.set_title("Frenet States")
        self.ax_frenet.grid(True, alpha=0.3)
        self.ax_frenet.legend(loc="best")

        (self.delta_line,) = self.ax_ctrl.plot([], [], label="delta(s)", lw=1.8)
        (self.torque_line,) = self.ax_ctrl.plot([], [], label="torque(s)", lw=1.8)
        self.ax_ctrl.set_xlabel("s [m]")
        self.ax_ctrl.set_ylabel("input")
        self.ax_ctrl.set_title("Control Along Path")
        self.ax_ctrl.grid(True, alpha=0.3)
        self.ax_ctrl.legend(loc="best")

        self.fig.tight_layout()
        self.fig.canvas.draw_idle()

    def _clear_dynamic_artists(self) -> None:
        for artist in self._dynamic_artists:
            try:
                artist.remove()
            except ValueError:
                pass
        self._dynamic_artists = []

    def update(self, record: dict[str, Any]) -> None:
        self._history.append(
            {
                "s": float(record["frenet"]["s"]),
                "e": float(record["frenet"]["e"]),
                "dphi": float(record["frenet"]["dphi"]),
                "delta": float(record["control"]["roadwheel_angle"]),
                "torque": float(record["control"]["rear_wheel_torque"]),
                "x": float(record["vehicle_pose"]["x"]),
                "y": float(record["vehicle_pose"]["y"]),
                "psi": float(record["vehicle_pose"]["psi"]),
            }
        )

        hist = self._history
        x_hist = [row["x"] for row in hist]
        y_hist = [row["y"] for row in hist]
        s_hist = [row["s"] for row in hist]
        e_hist = [row["e"] for row in hist]
        dphi_hist = [row["dphi"] for row in hist]
        delta_hist = [row["delta"] for row in hist]
        torque_hist = [row["torque"] for row in hist]

        self.path_line.set_data(x_hist, y_hist)
        self.horizon_line.set_data(record["predicted_cartesian"]["x"], record["predicted_cartesian"]["y"])
        self.start_marker.set_offsets(np.array([[x_hist[0], y_hist[0]]], dtype=float))
        self.current_marker.set_offsets(np.array([[x_hist[-1], y_hist[-1]]], dtype=float))

        self._clear_dynamic_artists()
        before_lines = len(self.ax_xy.lines)
        before_patches = len(self.ax_xy.patches)
        self.drawer.draw(
            self.ax_xy,
            x_hist[-1],
            y_hist[-1],
            hist[-1]["psi"],
            hist[-1]["delta"],
            color="tab:green",
            alpha=0.8,
        )
        self._dynamic_artists.extend(self.ax_xy.lines[before_lines:])
        self._dynamic_artists.extend(self.ax_xy.patches[before_patches:])

        self.e_line.set_data(s_hist, e_hist)
        self.dphi_line.set_data(s_hist, dphi_hist)
        self.delta_line.set_data(s_hist, delta_hist)
        self.torque_line.set_data(s_hist, torque_hist)

        for ax in (self.ax_xy, self.ax_frenet, self.ax_ctrl):
            ax.relim()
            ax.autoscale_view()
        self.ax_xy.set_aspect("equal")
        self.fig.canvas.draw_idle()
        if self._supports_pause:
            plt.pause(self.cfg.live_pause)

    def close(self) -> None:
        plt.ioff()


class LiveVehicleAnimator:
    """Simple live view for the simulator before MPC is enabled."""

    def __init__(self, reference=None, *, cfg: Optional[MPCVisualizationConfig] = None, title: str = "Live Vehicle"):
        self.reference = reference
        self.cfg = cfg or MPCVisualizationConfig()
        self.drawer = VehicleDrawer(fiala_params["vehicle"], scale=self.cfg.vehicle_scale)
        self._supports_pause = "agg" not in get_backend().lower()
        self._history: list[dict[str, float]] = []
        self._dynamic_artists: list[Any] = []

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=self.cfg.live_figsize)
        if reference is not None:
            self.ax.plot(reference.x, reference.y, "k--", lw=1.2, label="reference")
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_title(title)
        self.ax.grid(True, alpha=0.3)

        (self.path_line,) = self.ax.plot([], [], color="tab:blue", lw=2.0, label="closed-loop")
        self.start_marker = self.ax.scatter([], [], color="tab:green", s=65, zorder=7, label="start")
        self.current_marker = self.ax.scatter([], [], color="tab:red", s=65, zorder=7, label="current")
        self.ax.legend(loc="best")
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()

    def _clear_dynamic_artists(self) -> None:
        for artist in self._dynamic_artists:
            try:
                artist.remove()
            except ValueError:
                pass
        self._dynamic_artists = []

    def update(self, record: dict[str, Any]) -> None:
        pose = record["vehicle_pose"]
        control = record.get("control", {})
        self._history.append(
            {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "psi": float(pose["psi"]),
                "delta": float(control.get("roadwheel_angle", 0.0)),
            }
        )

        x_hist = [row["x"] for row in self._history]
        y_hist = [row["y"] for row in self._history]
        self.path_line.set_data(x_hist, y_hist)
        self.start_marker.set_offsets(np.array([[x_hist[0], y_hist[0]]], dtype=float))
        self.current_marker.set_offsets(np.array([[x_hist[-1], y_hist[-1]]], dtype=float))

        self._clear_dynamic_artists()
        before_lines = len(self.ax.lines)
        before_patches = len(self.ax.patches)
        self.drawer.draw(
            self.ax,
            x_hist[-1],
            y_hist[-1],
            self._history[-1]["psi"],
            self._history[-1]["delta"],
            color="tab:green",
            alpha=0.8,
        )
        self._dynamic_artists.extend(self.ax.lines[before_lines:])
        self._dynamic_artists.extend(self.ax.patches[before_patches:])

        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.set_aspect("equal")
        self.fig.canvas.draw_idle()
        if self._supports_pause:
            plt.pause(self.cfg.live_pause)

    def close(self) -> None:
        plt.ioff()


def plot_mpc_rollout_2d(
    reference,
    history: dict[str, np.ndarray],
    solve_log: list[dict[str, Any]],
    *,
    cfg: Optional[MPCVisualizationConfig] = None,
):
    cfg = cfg or MPCVisualizationConfig()
    fig, ax = plt.subplots(figsize=cfg.figsize_2d)

    ax.plot(reference.x, reference.y, "k--", lw=1.4, label="reference")
    ax.plot(history["true_x"], history["true_y"], color="tab:blue", lw=2.0, label="closed-loop path")

    for idx, record in enumerate(solve_log[:: max(1, cfg.rollout_stride)]):
        x_pred = record["predicted_cartesian"]["x"]
        y_pred = record["predicted_cartesian"]["y"]
        label = "predicted horizon" if idx == 0 else None
        ax.plot(x_pred, y_pred, color="tab:orange", alpha=cfg.rollout_alpha, lw=1.4, label=label)

    drawer = VehicleDrawer(fiala_params["vehicle"], scale=cfg.vehicle_scale)
    stride = max(1, cfg.vehicle_stride)
    for idx in range(0, len(solve_log), stride):
        record = solve_log[idx]
        drawer.draw(
            ax,
            record["vehicle_pose"]["x"],
            record["vehicle_pose"]["y"],
            record["vehicle_pose"]["psi"],
            record["control"]["roadwheel_angle"],
            color="tab:green",
            alpha=0.65,
        )

    if solve_log:
        start = solve_log[0]["vehicle_pose"]
        end = solve_log[-1]["vehicle_pose"]
        ax.scatter(start["x"], start["y"], color="tab:green", s=70, zorder=7, label="start")
        ax.scatter(end["x"], end["y"], color="tab:red", s=70, zorder=7, label="end")

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Closed-loop path with receding MPC horizons")
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.tight_layout()
    return fig


def plot_states_vs_s(
    solve_log: list[dict[str, Any]],
    reference,
    *,
    cfg: Optional[MPCVisualizationConfig] = None,
):
    cfg = cfg or MPCVisualizationConfig()
    if not solve_log:
        raise ValueError("solve_log is empty")

    s = np.asarray([record["frenet"]["s"] for record in solve_log], dtype=float)
    e = np.asarray([record["frenet"]["e"] for record in solve_log], dtype=float)
    dphi = np.asarray([record["frenet"]["dphi"] for record in solve_log], dtype=float)
    speed = np.asarray(
        [record["measured_state"].get("V", record["measured_state"]["vel_x"]) for record in solve_log],
        dtype=float,
    )
    yaw_rate = np.asarray([record["measured_state"]["yaw_rate"] for record in solve_log], dtype=float)
    steer = np.asarray([record["control"]["roadwheel_angle"] for record in solve_log], dtype=float)
    torque = np.asarray([record["control"]["rear_wheel_torque"] for record in solve_log], dtype=float)

    bounds = reference.sample(s)

    fig, axs = plt.subplots(3, 2, figsize=cfg.figsize_states, sharex=True)
    axs = axs.flatten()

    axs[0].plot(s, e, lw=2.0, label="e")
    axs[0].fill_between(s, bounds["e_min"], bounds["e_max"], color="tab:red", alpha=0.12, label="track bounds")
    axs[0].set_ylabel("e [m]")
    axs[0].set_title("Lateral Error vs s")
    axs[0].grid(True, alpha=0.3)
    axs[0].legend()

    axs[1].plot(s, dphi, lw=2.0)
    axs[1].set_ylabel("dphi [rad]")
    axs[1].set_title("Heading Error vs s")
    axs[1].grid(True, alpha=0.3)

    axs[2].plot(s, speed, lw=2.0, label="V")
    if np.all(np.isfinite(bounds["speed"])):
        axs[2].plot(s, bounds["speed"], "k--", lw=1.2, label="v_ref")
    axs[2].set_ylabel("speed [m/s]")
    axs[2].set_title("Longitudinal Speed vs s")
    axs[2].grid(True, alpha=0.3)
    axs[2].legend()

    axs[3].plot(s, yaw_rate, lw=2.0)
    axs[3].set_ylabel("r [rad/s]")
    axs[3].set_title("Yaw Rate vs s")
    axs[3].grid(True, alpha=0.3)

    axs[4].plot(s, steer, lw=2.0)
    axs[4].set_ylabel("delta [rad]")
    axs[4].set_xlabel("s [m]")
    axs[4].set_title("Steering vs s")
    axs[4].grid(True, alpha=0.3)

    axs[5].plot(s, torque, lw=2.0)
    axs[5].set_ylabel("rear torque [Nm]")
    axs[5].set_xlabel("s [m]")
    axs[5].set_title("Rear Wheel Torque vs s")
    axs[5].grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


__all__ = [
    "LiveVehicleAnimator",
    "LiveMPCAnimator",
    "MPCVisualizationConfig",
    "VehicleDrawer",
    "plot_mpc_rollout_2d",
    "plot_states_vs_s",
]