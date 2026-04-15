"""Week 10 — Trajectory-generation helper.

This module provides a clean API for generating minimum-time trajectories
using CasADi / IPOPT on top of the Fiala bicycle model defined in
``casadi_dynamics.py``.

The student workflow is:
  1.  Define a reference path   → ``TrackDefinition``
  2.  Configure the OCP         → ``TrajOptProblem``
  3.  Add constraints / costs   → methods on the problem
  4.  Solve                     → ``problem.solve()``
  5.  Analyse / plot            → ``TrajOptResult`` + plotting helpers
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum, auto
from typing import Any, Callable, Dict, List, Optional, Tuple

import casadi as ca
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.transforms import Affine2D

from casadi_dynamics import FialaBicycleCasADi, fiala_params

# ---------------------------------------------------------------------------
# Frenet integration
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class FrenetPath:
    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    kappa: np.ndarray

    def frenet_to_cartesian(
        self, s_query, e_query, dphi_query,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Convert Frenet (s, e, dphi) → Cartesian (x, y, psi)."""
        s_q = np.asarray(s_query, dtype=float)
        e_q = np.asarray(e_query, dtype=float)
        dphi_q = np.asarray(dphi_query, dtype=float)

        x_ref = np.interp(s_q, self.s, self.x)
        y_ref = np.interp(s_q, self.s, self.y)
        psi_ref = np.interp(s_q, self.s, self.yaw)

        x_cart = x_ref - e_q * np.sin(psi_ref)
        y_cart = y_ref + e_q * np.cos(psi_ref)
        psi_cart = psi_ref + dphi_q
        return x_cart, y_cart, psi_cart

    @classmethod
    def from_curvature(
        cls,
        s: np.ndarray,
        kappa_fn: Callable[[float], float],
        x0: float = 0.0,
        y0: float = 0.0,
        psi0: float = 0.0,
    ) -> "FrenetPath":
        s_arr = np.asarray(s, dtype=float)
        kappa_arr = np.array([float(kappa_fn(float(si))) for si in s_arr])
        x = np.empty_like(s_arr)
        y = np.empty_like(s_arr)
        yaw = np.empty_like(s_arr)
        x[0], y[0], yaw[0] = x0, y0, psi0
        for i in range(1, len(s_arr)):
            ds = s_arr[i] - s_arr[i - 1]
            yaw_mid = yaw[i - 1] + 0.5 * ds * kappa_arr[i - 1]
            yaw[i] = yaw[i - 1] + ds * kappa_arr[i - 1]
            x[i] = x[i - 1] + ds * math.cos(yaw_mid)
            y[i] = y[i - 1] + ds * math.sin(yaw_mid)
        return cls(s=s_arr, x=x, y=y, yaw=np.unwrap(yaw), kappa=kappa_arr)


# ---------------------------------------------------------------------------
# Track definition (curvature + lateral bounds)
# ---------------------------------------------------------------------------

def oval_curvature(straight_length: float, radius: float, n_points: int = 20):
    """Return (kappa, s) for a single oval lap.

    The oval has two straights of length ``straight_length`` connected by
    two half-circles of the given ``radius``.
    """
    turn_len = np.pi * radius
    segs_k = [
        np.linspace(0, 0, n_points),
        np.linspace(0, 1 / radius, n_points),
        np.linspace(1 / radius, 0, n_points),
        np.linspace(0, 0, n_points),
        np.linspace(0, 1 / radius, n_points),
        np.linspace(1 / radius, 0, n_points),
        np.linspace(0, 0, n_points),
    ]
    segs_s = [
        np.linspace(0, straight_length / 2 - 1e-1, n_points),
        np.linspace(
            straight_length / 2,
            straight_length / 2 + turn_len - 1e-1,
            n_points,
        ),
        np.linspace(
            straight_length / 2 + turn_len,
            straight_length / 2 + 2 * turn_len - 1e-1,
            n_points,
        ),
        np.linspace(
            straight_length / 2 + 2 * turn_len,
            1.5 * straight_length + 2 * turn_len - 1e-1,
            n_points,
        ),
        np.linspace(
            1.5 * straight_length + 2 * turn_len,
            1.5 * straight_length + 3 * turn_len - 1e-1,
            n_points,
        ),
        np.linspace(
            1.5 * straight_length + 3 * turn_len,
            1.5 * straight_length + 4 * turn_len - 1e-1,
            n_points,
        ),
        np.linspace(
            1.5 * straight_length + 4 * turn_len,
            2.0 * straight_length + 4 * turn_len,
            n_points,
        ),
    ]
    return np.concatenate(segs_k), np.concatenate(segs_s)


def repeat_oval(
    straight_length: float,
    radius: float,
    n_laps: int = 2,
    n_points: int = 20,
) -> Tuple[np.ndarray, np.ndarray]:
    """Repeat an oval ``n_laps`` times, returning (kappa, s)."""
    k_lap, s_lap = oval_curvature(straight_length, radius, n_points)
    L = float(s_lap[-1] - s_lap[0])
    k_list, s_list = [], []
    for i in range(n_laps):
        offset = i * L
        if i > 0:
            k_list.append(k_lap[1:])
            s_list.append(s_lap[1:] + offset)
        else:
            k_list.append(k_lap)
            s_list.append(s_lap + offset)
    return np.concatenate(k_list), np.concatenate(s_list)


@dataclass
class TrackDefinition:
    """Everything the optimizer needs to know about the reference path.

    Build one with ``TrackDefinition.oval(...)`` or by supplying your
    own ``(s, kappa, e_min, e_max)`` arrays.
    """
    s: np.ndarray
    kappa: np.ndarray
    e_min: np.ndarray
    e_max: np.ndarray
    x0: float = 0.0
    y0: float = 0.0
    psi0: float = -np.pi / 2.0

    @classmethod
    def oval(
        cls,
        straight_length: float = 20.0,
        radius: float = 15.0,
        n_laps: int = 2,
        e_half_width: float = 4.0,
        lead_in: float = 50.0,
        lead_out: float = 50.0,
        ds: float = 1.0,
        x0: float = 0.0,
        y0: float = 0.0,
        psi0: float = -np.pi / 2.0,
    ) -> "TrackDefinition":
        """Build an oval track with straight lead-in / lead-out segments."""
        kappa_oval, s_oval_raw = repeat_oval(straight_length, radius, n_laps)

        s_lead = np.arange(0, lead_in, ds)
        s_oval = s_oval_raw + (s_lead[-1] + ds if len(s_lead) else 0.0)
        s_tail = s_oval[-1] + np.arange(ds, lead_out + ds, ds)

        s_full = np.concatenate((s_lead, s_oval, s_tail))
        k_full = np.concatenate(
            (np.zeros_like(s_lead), kappa_oval, np.zeros_like(s_tail))
        )
        return cls(
            s=s_full,
            kappa=k_full,
            e_min=np.full_like(s_full, -e_half_width),
            e_max=np.full_like(s_full, e_half_width),
            x0=x0,
            y0=y0,
            psi0=psi0,
        )

    @classmethod
    def from_kappa_function(
        cls,
        kappa_fn: Callable[[float], float],
        s_start: float,
        s_end: float,
        ds: float = 1.0,
        e_half_width: float = 4.0,
        x0: float = 0.0,
        y0: float = 0.0,
        psi0: float = -np.pi / 2.0,
    ) -> "TrackDefinition":
        """Build a track from any curvature function."""
        s = np.arange(s_start, s_end + ds, ds)
        kappa = np.array([kappa_fn(si) for si in s])
        return cls(
            s=s,
            kappa=kappa,
            e_min=np.full_like(s, -e_half_width),
            e_max=np.full_like(s, e_half_width),
            x0=x0,
            y0=y0,
            psi0=psi0,
        )

    def kappa_at(self, s_query) -> np.ndarray:
        return np.interp(s_query, self.s, self.kappa)

    def e_bounds_at(self, s_query) -> Tuple[np.ndarray, np.ndarray]:
        return (
            np.interp(s_query, self.s, self.e_min),
            np.interp(s_query, self.s, self.e_max),
        )

    @property
    def s_start(self) -> float:
        return float(self.s[0])

    @property
    def s_end(self) -> float:
        return float(self.s[-1])

    def get_path(self, ds: float = 1.0) -> FrenetPath:
        s_eval = np.arange(self.s_start, self.s_end, ds)
        return FrenetPath.from_curvature(
            s_eval,
            lambda si: float(np.interp(si, self.s, self.kappa)),
            x0=self.x0,
            y0=self.y0,
            psi0=self.psi0,
        )

    def plot(self, ds: float = 1.0, ax: Optional[plt.Axes] = None) -> plt.Axes:
        """Quick preview of the track geometry and lateral bounds.

        The plot highlights:
        - centre-line and boundaries
        - start and end points
        - direction of travel using arrows on the centre-line
        """
        path = self.get_path(ds)
        s_ref = path.s

        e_lo, e_hi = self.e_bounds_at(s_ref)
        x_lo, y_lo, _ = path.frenet_to_cartesian(s_ref, e_lo, np.zeros_like(s_ref))
        x_hi, y_hi, _ = path.frenet_to_cartesian(s_ref, e_hi, np.zeros_like(s_ref))

        if ax is None:
            _, ax = plt.subplots(figsize=(10, 10))

        ax.plot(path.x, path.y, "k-", lw=2, label="Reference centre-line")
        ax.plot(x_lo, y_lo, "r--", lw=1, label="Track boundary")
        ax.plot(x_hi, y_hi, "r--", lw=1)

        # Mark start / end points on the centre-line.
        ax.scatter(path.x[0], path.y[0], c="tab:green", s=70, zorder=5, label="Start")
        ax.scatter(path.x[-1], path.y[-1], c="tab:blue", s=70, zorder=5, label="End")

        # Add a few arrows to indicate travel direction.
        if len(path.x) > 2:
            arrow_count = min(8, max(3, len(path.x) // 25))
            arrow_idx = np.linspace(0, len(path.x) - 2, arrow_count, dtype=int)
            for idx in np.unique(arrow_idx):
                ax.annotate(
                    "",
                    xy=(path.x[idx + 1], path.y[idx + 1]),
                    xytext=(path.x[idx], path.y[idx]),
                    arrowprops={"arrowstyle": "->", "color": "tab:orange", "lw": 1.6},
                    zorder=4,
                )

        ax.set_aspect("equal")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.legend()
        ax.grid(True)
        return ax


# ---------------------------------------------------------------------------
# State / control layout
# ---------------------------------------------------------------------------

class StateIdx(IntEnum):
    """State-vector indices used by the internal NLP."""
    s = 0
    t = auto()
    r = auto()      # yaw rate
    V = auto()      # speed
    beta = auto()   # side-slip angle
    wr = auto()     # rear wheel speed (m/s)
    e = auto()      # lateral deviation
    dphi = auto()   # heading error
    delta = auto()  # road-wheel angle (steering)
    rear_wheel_torque = auto()  # rear wheel torque state


class ControlIdx(IntEnum):
    """Control-vector indices."""
    delta_r = 0     # steering rate (rad / s)
    rear_wheel_torque_r = auto()  # rear wheel torque rate (Nm / s)


_STATE_LABELS = {int(v): v.name for v in StateIdx}
_CTRL_LABELS = {int(v): v.name for v in ControlIdx}
NUM_X = len(StateIdx)
NUM_U = len(ControlIdx)


# ---------------------------------------------------------------------------
# Default variable ranges and scaling
# ---------------------------------------------------------------------------

_DEFAULT_RANGES: Dict[str, Tuple[float, float]] = {
    "s": (0.0, 700.0),
    "t": (0.0, 100.0),
    "r": (-1.5, 1.5),
    "V": (2, 30.0),
    "beta": (-0.1, 0.1),
    "wr": (2, 30.0),
    "e": (-10.0, 10.0),
    "dphi": (-1.2, 1.2),
    "delta": (-0.4, 0.4),
    "rear_wheel_torque": (-500.0, 3000.0),
    "delta_r": (-0.9, 0.9),
    "rear_wheel_torque_r": (-2000.0, 2000.0),
}

_DEFAULT_MAG: Dict[str, float] = {
    "s": 500.0,
    "t": 100.0,
    "r": 1.5,
    "V": 30.0,
    "beta": 1.0,
    "wr": 30.0,
    "e": 10.0,
    "dphi": 1.0,
    "delta": 0.5,
    "rear_wheel_torque": 3000.0,
    "delta_r": 1.0,
    "rear_wheel_torque_r": 5000.0,
}

_NAME_ALIASES = {
    "eng_tor": "rear_wheel_torque",
    "eng_tor_r": "rear_wheel_torque_r",
}


def _canonical_name(name: str) -> str:
    return _NAME_ALIASES.get(name, name)


def _build_scaling(s_end: float):
    mag = _DEFAULT_MAG.copy()
    mag["s"] = max(s_end, 1.0)

    ranges = _DEFAULT_RANGES.copy()
    ranges["s"] = (0.0, s_end)

    x_mag = np.array([mag[_STATE_LABELS[i]] for i in range(NUM_X)])
    u_mag = np.array([mag[_CTRL_LABELS[i]] for i in range(NUM_U)])

    x_lo = np.array([ranges[_STATE_LABELS[i]][0] for i in range(NUM_X)])
    x_hi = np.array([ranges[_STATE_LABELS[i]][1] for i in range(NUM_X)])
    u_lo = np.array([ranges[_CTRL_LABELS[i]][0] for i in range(NUM_U)])
    u_hi = np.array([ranges[_CTRL_LABELS[i]][1] for i in range(NUM_U)])

    return x_mag, u_mag, x_lo, x_hi, u_lo, u_hi


# ---------------------------------------------------------------------------
# Dynamics wrapper (builds CasADi graph)
# ---------------------------------------------------------------------------

def _build_dynamics_function(model: FialaBicycleCasADi, x_mag, u_mag, sigma):
    """Return CasADi function  (x_hat, u_hat, kappa) → (dx_ds_hat, feats_dict)."""
    x_sym = ca.SX.sym("x", NUM_X)
    u_sym = ca.SX.sym("u", NUM_U)
    kappa_sym = ca.SX.sym("kappa")

    x = x_sym * x_mag
    u = u_sym * u_mag

    state_ctrl = {
        _STATE_LABELS[i]: x[i] for i in range(NUM_X)
    }
    state_ctrl.update({
        _CTRL_LABELS[i]: u[i] for i in range(NUM_U)
    })
    # Map names for the bicycle model
    state_ctrl["rear_wheel_torque"] = x[StateIdx.rear_wheel_torque]
    state_ctrl["roadwheel_angle"] = x[StateIdx.delta]
    state_ctrl["rear_wheelspeed_ms"] = x[StateIdx.wr]
    state_ctrl["yaw_rate"] = x[StateIdx.r]

    feats = model.get_vectorfield(state_ctrl)

    _V = x[StateIdx.V]
    _dphi = x[StateIdx.dphi]
    _e = x[StateIdx.e]
    _r = x[StateIdx.r]
    kref = kappa_sym

    e_dot = _V * ca.sin(_dphi)
    s_dot = (_V * ca.cos(_dphi)) / (1 - _e * kref)
    dphi_dot = feats["beta_dot"] + _r - kref * s_dot

    all_dots = {
        "s_dot": s_dot,
        "t_dot": 1.0,
        "r_dot": feats["r_dot"],
        "V_dot": feats["V_dot"],
        "beta_dot": feats["beta_dot"],
        "wr_dot": feats["wr_dot"],
        "e_dot": e_dot,
        "dphi_dot": dphi_dot,
        "delta_dot": u[ControlIdx.delta_r],
        "rear_wheel_torque_dot": u[ControlIdx.rear_wheel_torque_r],
    }

    dx_dt = ca.vertcat(*[all_dots[f"{_STATE_LABELS[i]}_dot"] for i in range(NUM_X)])
    dx_ds = dx_dt / s_dot
    dx_ds_hat = dx_ds * (sigma / x_mag)

    f_dyn = ca.Function("f_dyn", [x_sym, u_sym, kappa_sym], [dx_ds_hat])
    f_dyn_dt = ca.Function("f_dyn_dt", [x_sym, u_sym, kappa_sym], [dx_dt])
    return f_dyn, f_dyn_dt, feats


# ---------------------------------------------------------------------------
# Trajectory optimisation result
# ---------------------------------------------------------------------------

@dataclass
class TrajOptResult:
    """Container returned by ``TrajOptProblem.solve()``.

    For custom analysis, inspect:
    - ``result.x`` for the full state array with columns indexed by ``StateIdx``
    - ``result.u`` for the full control array with columns indexed by ``ControlIdx``
    - the convenience properties below for common signals such as ``V`` and ``e``
    """

    x_hat: np.ndarray        # (N, NUM_X) scaled
    u_hat: np.ndarray        # (N-1, NUM_U) scaled
    x: np.ndarray            # (N, NUM_X) physical units
    u: np.ndarray            # (N-1, NUM_U) physical units
    cost: float
    solver_stats: dict
    track: TrackDefinition
    path_ref: FrenetPath
    vehicle_params: Optional[dict] = None

    # -- Convenience accessors on physical-unit arrays --------------------
    @property
    def s(self) -> np.ndarray:
        return self.x[:, StateIdx.s]

    @property
    def t(self) -> np.ndarray:
        return self.x[:, StateIdx.t]

    @property
    def V(self) -> np.ndarray:
        return self.x[:, StateIdx.V]

    @property
    def e(self) -> np.ndarray:
        return self.x[:, StateIdx.e]

    @property
    def dphi(self) -> np.ndarray:
        return self.x[:, StateIdx.dphi]

    @property
    def delta(self) -> np.ndarray:
        return self.x[:, StateIdx.delta]

    @property
    def rear_wheel_torque(self) -> np.ndarray:
        return self.x[:, StateIdx.rear_wheel_torque]

    @property
    def eng_tor(self) -> np.ndarray:
        """Backward-compatible alias for ``rear_wheel_torque``."""
        return self.rear_wheel_torque

    @property
    def beta(self) -> np.ndarray:
        return self.x[:, StateIdx.beta]

    @property
    def wr(self) -> np.ndarray:
        return self.x[:, StateIdx.wr]

    @property
    def r(self) -> np.ndarray:
        return self.x[:, StateIdx.r]

    @property
    def kappa_ref(self) -> np.ndarray:
        return self.track.kappa_at(self.s)

    def get_xy_trajectory(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Convert the Frenet solution back to Cartesian (x, y, yaw)."""
        x_cart, y_cart, psi_cart = self.path_ref.frenet_to_cartesian(
            self.s, self.e, self.dphi
        )
        yaw = psi_cart - self.beta
        return x_cart, y_cart, yaw

    def compute_tire_forces(self) -> Dict[str, np.ndarray]:
        """Evaluate tire-force related quantities along the solved trajectory.

        Returns arrays for front/rear longitudinal/lateral forces,
        slip-angle / slip-ratio diagnostics, and friction-circle metrics
        useful for Part 4 analysis.
        """
        params = self.vehicle_params or fiala_params
        model = FialaBicycleCasADi(params)

        N = self.x.shape[0]
        out = {
            "s": np.array(self.s, copy=True),
            "Fxf": np.zeros(N),
            "Fyf": np.zeros(N),
            "Fxr": np.zeros(N),
            "Fyr": np.zeros(N),
            "Fzf": np.zeros(N),
            "Fzr": np.zeros(N),
            "alpha_f": np.zeros(N),
            "alpha_r": np.zeros(N),
            "kappa_x_front": np.zeros(N),
            "kappa_x_rear": np.zeros(N),
            "mu_front": np.zeros(N),
            "mu_rear": np.zeros(N),
            "front_util": np.zeros(N),
            "rear_util": np.zeros(N),
            "gg_long": np.zeros(N),
            "gg_lat": np.zeros(N),
        }

        mass = float(params["vehicle"]["mass"])
        gravity = float(params["vehicle"]["gravity"])
        mu_f = float(params["tires"]["mu_front"])
        mu_r = float(params["tires"]["mu_rear"])

        for i in range(N):
            state_control = {
                "r": float(self.r[i]),
                "V": float(self.V[i]),
                "beta": float(self.beta[i]),
                "wr": float(self.wr[i]),
                "delta": float(self.delta[i]),
                "rear_wheel_torque": float(self.rear_wheel_torque[i]),
                "brake": 0.0,
            }
            feats = model.get_vectorfield(state_control)

            Fxf = float(feats["Fxf"])
            Fyf = float(feats["Fyf"])
            Fxr = float(feats["Fxr"])
            Fyr = float(feats["Fyr"])
            Fzf = float(feats["Fzf"])
            Fzr = float(feats["Fzr"])
            alpha_f = float(feats["alpha_f"])
            alpha_r = float(feats["alpha_r"])
            kappa_x_front = float(feats["kappa_x_front"])
            kappa_x_rear = float(feats["kappa_x_rear"])
            accel_x = float(feats.get("accel_x", feats["vx_dot"]))
            accel_y = float(feats.get("accel_y", feats["vy_dot"]))

            out["Fxf"][i] = Fxf
            out["Fyf"][i] = Fyf
            out["Fxr"][i] = Fxr
            out["Fyr"][i] = Fyr
            out["Fzf"][i] = Fzf
            out["Fzr"][i] = Fzr
            out["alpha_f"][i] = alpha_f
            out["alpha_r"][i] = alpha_r
            out["kappa_x_front"][i] = kappa_x_front
            out["kappa_x_rear"][i] = kappa_x_rear
            out["mu_front"][i] = mu_f
            out["mu_rear"][i] = mu_r

            front_limit = max(mu_f * Fzf, 1e-6)
            rear_limit = max(mu_r * Fzr, 1e-6)
            out["front_util"][i] = np.hypot(Fxf, Fyf) / front_limit
            out["rear_util"][i] = np.hypot(Fxr, Fyr) / rear_limit

            out["gg_long"][i] = accel_x / gravity
            out["gg_lat"][i] = accel_y / gravity

        return out

    def plot_force_vs_alpha_slip(
        self,
        force_data: Optional[Dict[str, np.ndarray]] = None,
        alpha_in_degrees: bool = True,
    ):
        """Plot force maps versus slip angle and slip ratio for front/rear tires."""
        data = self.compute_tire_forces() if force_data is None else force_data
        s_vals = data.get("s", self.s)

        alpha_f = data["alpha_f"]
        alpha_r = data["alpha_r"]
        kappa_f = data["kappa_x_front"]
        kappa_r = data["kappa_x_rear"]
        Fyf = data["Fyf"]
        Fyr = data["Fyr"]
        Fxf = data["Fxf"]
        Fxr = data["Fxr"]

        if alpha_in_degrees:
            alpha_f_plot = np.degrees(alpha_f)
            alpha_r_plot = np.degrees(alpha_r)
            alpha_unit = "deg"
        else:
            alpha_f_plot = alpha_f
            alpha_r_plot = alpha_r
            alpha_unit = "rad"

        fig, axs = plt.subplots(2, 2, figsize=(12, 9), sharey="row")
        fig.subplots_adjust(left=0.08, right=0.88, bottom=0.08, top=0.9, wspace=0.28, hspace=0.32)

        sc0 = axs[0, 0].scatter(alpha_f_plot, Fyf, c=s_vals, cmap="viridis", s=10)
        axs[0, 0].set_xlabel(f"alpha_f ({alpha_unit})")
        axs[0, 0].set_ylabel("Fy front (N)")
        axs[0, 0].set_title("Front lateral force vs slip angle")
        axs[0, 0].grid(True, alpha=0.3)

        axs[0, 1].scatter(alpha_r_plot, Fyr, c=s_vals, cmap="viridis", s=10)
        axs[0, 1].set_xlabel(f"alpha_r ({alpha_unit})")
        axs[0, 1].set_title("Rear lateral force vs slip angle")
        axs[0, 1].grid(True, alpha=0.3)

        axs[1, 0].scatter(kappa_f, Fxf, c=s_vals, cmap="viridis", s=10)
        axs[1, 0].set_xlabel("kappa_x_front (-)")
        axs[1, 0].set_ylabel("Fx front (N)")
        axs[1, 0].set_title("Front longitudinal force vs slip ratio")
        axs[1, 0].grid(True, alpha=0.3)

        axs[1, 1].scatter(kappa_r, Fxr, c=s_vals, cmap="viridis", s=10)
        axs[1, 1].set_xlabel("kappa_x_rear (-)")
        axs[1, 1].set_title("Rear longitudinal force vs slip ratio")
        axs[1, 1].grid(True, alpha=0.3)

        cax = fig.add_axes([0.9, 0.12, 0.02, 0.72])
        fig.colorbar(sc0, cax=cax, label="s (m)")
        fig.suptitle("Force response versus slip angle and slip ratio")
        return fig

    def plot_friction_circle(self, force_data: Optional[Dict[str, np.ndarray]] = None):
        """Plot front/rear tire-force points against normalized friction circles."""
        data = self.compute_tire_forces() if force_data is None else force_data

        Fxf = data["Fxf"]
        Fyf = data["Fyf"]
        Fxr = data["Fxr"]
        Fyr = data["Fyr"]
        Fzf = data["Fzf"]
        Fzr = data["Fzr"]
        mu_f = data["mu_front"]
        mu_r = data["mu_rear"]

        # Normalize by friction limits to show unit circles.
        nf_x = Fxf / np.maximum(mu_f * Fzf, 1e-6)
        nf_y = Fyf / np.maximum(mu_f * Fzf, 1e-6)
        nr_x = Fxr / np.maximum(mu_r * Fzr, 1e-6)
        nr_y = Fyr / np.maximum(mu_r * Fzr, 1e-6)

        th = np.linspace(0.0, 2.0 * np.pi, 200)
        fig, axs = plt.subplots(1, 2, figsize=(12, 5))
        fig.subplots_adjust(left=0.08, right=0.88, bottom=0.12, top=0.88, wspace=0.3)

        axs[0].plot(np.cos(th), np.sin(th), "k--", lw=1.2, label="|F| = mu Fz")
        axs[0].scatter(nf_x, nf_y, c=self.s, cmap="viridis", s=10)
        axs[0].set_title("Front tire")
        axs[0].set_xlabel("Fx / (mu_f Fzf)")
        axs[0].set_ylabel("Fy / (mu_f Fzf)")
        axs[0].set_aspect("equal", adjustable="box")
        axs[0].grid(True, alpha=0.3)

        sc = axs[1].scatter(nr_x, nr_y, c=self.s, cmap="viridis", s=10)
        axs[1].plot(np.cos(th), np.sin(th), "k--", lw=1.2, label="|F| = mu Fz")
        axs[1].set_title("Rear tire")
        axs[1].set_xlabel("Fx / (mu_r Fzr)")
        axs[1].set_aspect("equal", adjustable="box")
        axs[1].grid(True, alpha=0.3)

        cax = fig.add_axes([0.9, 0.18, 0.02, 0.6])
        fig.colorbar(sc, cax=cax, label="s (m)")
        fig.suptitle("Normalized friction-circle usage")
        return fig

    def plot_gg_diagram(self, force_data: Optional[Dict[str, np.ndarray]] = None):
        """Plot longitudinal vs lateral acceleration in g-units."""
        data = self.compute_tire_forces() if force_data is None else force_data
        ax_g = data["gg_long"]
        ay_g = data["gg_lat"]

        fig, ax = plt.subplots(figsize=(7, 6))
        fig.subplots_adjust(left=0.12, right=0.84, bottom=0.11, top=0.9)
        sc = ax.scatter(ax_g, ay_g, c=self.s, cmap="viridis", s=10)
        ax.set_xlabel("a_x / g")
        ax.set_ylabel("a_y / g")
        ax.set_title("GG diagram")
        ax.grid(True, alpha=0.3)
        ax.axis("equal")
        cax = fig.add_axes([0.87, 0.14, 0.025, 0.7])
        fig.colorbar(sc, cax=cax, label="s (m)")
        return fig

    # -- Plotting ---------------------------------------------------------

    def plot_states(self, figsize=(14, 12)):
        """Plot every state and control vs. path distance."""
        s_vals = self.s
        s_ctrl = s_vals[:-1]

        state_names = [_STATE_LABELS[i] for i in range(NUM_X)]
        ctrl_names = [_CTRL_LABELS[i] for i in range(NUM_U)]
        n_plots = NUM_X + NUM_U
        ncols = 3
        nrows = int(np.ceil(n_plots / ncols))
        fig, axes = plt.subplots(nrows, ncols, figsize=figsize, sharex=True)
        axes = axes.flatten()

        for i in range(NUM_X):
            ax = axes[i]
            ax.plot(s_vals, self.x[:, i], lw=2, label=state_names[i])
            if i == int(StateIdx.V):
                ax.plot(s_vals, self.wr, "k:", lw=1.5, label="wr")
            if i == int(StateIdx.e):
                e_lo, e_hi = self.track.e_bounds_at(s_vals)
                ax.fill_between(s_vals, e_lo, e_hi, alpha=0.15, color="red", label="bounds")
            ax.set_ylabel(state_names[i])
            ax.grid(True, ls="--", alpha=0.6)
            ax.legend(fontsize=9)

        for i in range(NUM_U):
            ax = axes[NUM_X + i]
            ax.plot(s_ctrl, self.u[:, i], lw=2, color="red", label=ctrl_names[i])
            ax.set_ylabel(ctrl_names[i])
            ax.grid(True, ls="--", alpha=0.6)
            ax.legend(fontsize=9)

        for j in range(n_plots, len(axes)):
            axes[j].axis("off")
        for ax in axes[-ncols:]:
            ax.set_xlabel("s (m)")
        fig.suptitle("Optimal State / Control Profiles", fontsize=15)
        plt.tight_layout()
        return fig

    def plot_trajectory_2d(self, figsize=(10, 10)):
        """XY plot of the race line overlaid on the track."""
        path = self.path_ref
        s_ref = path.s
        e_lo, e_hi = self.track.e_bounds_at(s_ref)
        x_lo, y_lo, _ = path.frenet_to_cartesian(s_ref, e_lo, np.zeros_like(s_ref))
        x_hi, y_hi, _ = path.frenet_to_cartesian(s_ref, e_hi, np.zeros_like(s_ref))

        x_sol, y_sol, _ = self.get_xy_trajectory()

        fig, ax = plt.subplots(figsize=figsize)
        ax.plot(path.x, path.y, "k-", lw=1.5, alpha=0.4, label="Centre-line")
        ax.plot(x_lo, y_lo, "r--", lw=1, label="Track edge")
        ax.plot(x_hi, y_hi, "r--", lw=1)
        sc = ax.scatter(x_sol, y_sol, c=self.V, cmap="viridis", s=15, zorder=3)
        fig.colorbar(sc, ax=ax, label="Speed (m/s)")
        ax.set_aspect("equal")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_title("Race-line (colour = speed)")
        plt.tight_layout()
        return fig

    def plot_with_vehicles(
        self,
        s_range: Optional[Tuple[float, float]] = None,
        step: int = 10,
        color_state: str = "V",
        vehicle_scale: float = 2.0,
        figsize: Tuple[int, int] = (12, 10),
    ):
        """XY plot with vehicle outlines drawn at sampled points."""
        path = self.path_ref
        x_sol, y_sol, yaw_sol = self.get_xy_trajectory()
        s_sol = self.s

        if s_range is None:
            s_range = (float(s_sol[0]), float(s_sol[-1]))

        idx_range = np.where((s_sol >= s_range[0]) & (s_sol <= s_range[1]))[0]
        if len(idx_range) == 0:
            raise ValueError("No points in s_range")
        idxs = idx_range[::step]

        color_data = getattr(self, color_state)
        cmap = plt.colormaps["viridis"]
        norm = plt.Normalize(vmin=color_data[idx_range].min(), vmax=color_data[idx_range].max())

        # Track edges
        s_ref = path.s
        lim = np.where((s_ref >= s_range[0]) & (s_ref <= s_range[1]))[0]
        e_lo, e_hi = self.track.e_bounds_at(s_ref[lim])
        x_lo, y_lo, _ = path.frenet_to_cartesian(s_ref[lim], e_lo, np.zeros_like(e_lo))
        x_hi, y_hi, _ = path.frenet_to_cartesian(s_ref[lim], e_hi, np.zeros_like(e_hi))

        fig, ax = plt.subplots(figsize=figsize)
        ax.plot(x_lo, y_lo, "r--", lw=1)
        ax.plot(x_hi, y_hi, "r--", lw=1)
        sc = ax.scatter(
            x_sol[idx_range], y_sol[idx_range],
            c=color_data[idx_range], cmap="viridis", s=15, zorder=2,
        )

        drawer = _VehicleDrawer(fiala_params["vehicle"], scale=vehicle_scale)
        for i in idxs:
            c = cmap(norm(color_data[i]))
            drawer.draw(ax, x_sol[i], y_sol[i], yaw_sol[i], self.delta[i], color=c)

        fig.colorbar(sc, ax=ax, label=color_state)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("Trajectory with vehicle outlines")
        plt.tight_layout()
        return fig


# ---------------------------------------------------------------------------
# Vehicle outline drawer (from old_traj_generation)
# ---------------------------------------------------------------------------

class _VehicleDrawer:
    def __init__(self, veh: dict, scale: float = 1.0):
        self.a = veh["cogToFrontAxle"] * scale
        self.b = veh["cogToRearAxle"] * scale
        hw = veh.get("track_width", 1.787) * 0.5 * scale
        self.hw = hw
        self.wl = 0.6 * scale
        self.ww = 0.2 * scale

    def draw(self, ax, x, y, yaw, steer, color="black", alpha=1.0):
        c, s = np.cos(yaw), np.sin(yaw)
        wa = yaw + steer
        pts = {
            "fl": (x + self.a * c - self.hw * s, y + self.a * s + self.hw * c),
            "fr": (x + self.a * c + self.hw * s, y + self.a * s - self.hw * c),
            "rl": (x - self.b * c - self.hw * s, y - self.b * s + self.hw * c),
            "rr": (x - self.b * c + self.hw * s, y - self.b * s - self.hw * c),
        }
        ax.plot(
            [x - self.b * c, x + self.a * c],
            [y - self.b * s, y + self.a * s],
            color=color, lw=3, alpha=alpha,
        )
        ax.plot(*zip(pts["fl"], pts["fr"]), color=color, lw=2, alpha=alpha)
        ax.plot(*zip(pts["rl"], pts["rr"]), color=color, lw=2, alpha=alpha)

        def _wheel(cx, cy, angle):
            r = patches.Rectangle(
                (-self.wl / 2, -self.ww / 2), self.wl, self.ww,
                fc="black", ec="none", alpha=alpha, zorder=6,
            )
            r.set_transform(Affine2D().rotate(angle).translate(cx, cy) + ax.transData)
            ax.add_patch(r)

        _wheel(*pts["fl"], wa)
        _wheel(*pts["fr"], wa)
        _wheel(*pts["rl"], yaw)
        _wheel(*pts["rr"], yaw)


# ---------------------------------------------------------------------------
# Main: Trajectory Optimisation Problem
# ---------------------------------------------------------------------------

@dataclass
class TrajOptConfig:
    """User-facing knobs for the trajectory optimisation."""
    num_nodes: int = 200
    start_velocity: Tuple[float, float] = (3.0, 5.0)   # (lb, ub) m/s
    initial_state_bounds: Optional[Dict[str, Tuple[float, float]]] = None
    initial_control_rate_zero: bool = True
    cost_control_rate_weight: float = 0.05
    ipopt_print_level: int = 3
    ipopt_max_iter: int = 2000
    target_steady_velocity: Tuple[float, float] = None


class TrajOptProblem:
    """Set up and solve a minimum-time trajectory NLP.

    Typical usage::

        track = TrackDefinition.oval()
        problem = TrajOptProblem(track)

        # optionally add custom constraints before solving
        problem.add_accel_bounds(a_min=0.0, a_max=5.0, s_range=(0, 50))

        result = problem.solve()
        result.plot_states()
        result.plot_trajectory_2d()
    """

    def __init__(
        self,
        track: TrackDefinition,
        config: Optional[TrajOptConfig] = None,
        vehicle_params: Optional[dict] = None,
    ):
        self.track = track
        self.cfg = config or TrajOptConfig()
        self.veh_params = vehicle_params or fiala_params
        self.model = FialaBicycleCasADi(self.veh_params)

        # Scaling
        self._x_mag, self._u_mag, self._x_lo, self._x_hi, self._u_lo, self._u_hi = (
            _build_scaling(track.s_end)
        )
        self._x_lo_hat = self._x_lo / self._x_mag
        self._x_hi_hat = self._x_hi / self._x_mag
        self._u_lo_hat = self._u_lo / self._u_mag
        self._u_hi_hat = self._u_hi / self._u_mag

        # NLP timeline
        N = self.cfg.num_nodes
        sigma = track.s_end - track.s_start
        tau = np.linspace(0.0, 1.0, N)
        self._sigma = sigma
        self._tau = tau
        self._s_nodes = track.s_start + tau * sigma
        self._kappa_nodes = ca.DM(track.kappa_at(self._s_nodes))

        # Build dynamics CasADi function
        self._f_dyn, self._f_dyn_dt, _ = _build_dynamics_function(
            self.model, self._x_mag, self._u_mag, sigma,
        )

        # Storage for user constraints before solve()
        self._extra_accel_bounds: List[Tuple[float, float, float, float]] = []
        self._extra_constraints: List[Callable] = []

    def _apply_initial_state_bounds(self, lb_init: np.ndarray, ub_init: np.ndarray):
        """Apply user-provided initial-state bounds from the config.

        The dictionary keys use the ``StateIdx`` names, for example:
        ``{"e": (-0.1, 0.1), "beta": (-0.05, 0.05), "rear_wheel_torque": (0.0, 500.0)}``.
        """
        if not self.cfg.initial_state_bounds:
            return

        for raw_name, bounds in self.cfg.initial_state_bounds.items():
            name = _canonical_name(raw_name)
            if name not in {_STATE_LABELS[i] for i in range(NUM_X)}:
                raise ValueError(f"Unknown initial-state name: {raw_name}")
            idx = [i for i in range(NUM_X) if _STATE_LABELS[i] == name][0]
            lb_phys, ub_phys = bounds
            lb_init[idx] = max(lb_phys / self._x_mag[idx], self._x_lo_hat[idx])
            ub_init[idx] = min(ub_phys / self._x_mag[idx], self._x_hi_hat[idx])

    # -- Public constraint-adding methods --------------------------------

    def add_accel_bounds(
        self,
        a_min: float,
        a_max: float,
        s_range: Optional[Tuple[float, float]] = None,
    ):
        """Add longitudinal-acceleration bounds (m/s²) for a range of s.

        If ``s_range`` is ``None`` the bounds apply everywhere.
        """
        if s_range is None:
            s_range = (self.track.s_start, self.track.s_end)
        self._extra_accel_bounds.append((a_min, a_max, s_range[0], s_range[1]))

    def set_e_bounds(self, e_min: float, e_max: float, s_range: Tuple[float, float]):
        """Override lateral-error bounds for a sub-range of s."""
        mask = (self.track.s >= s_range[0]) & (self.track.s <= s_range[1])
        self.track.e_min[mask] = e_min
        self.track.e_max[mask] = e_max

    def set_state_range(self, name: str, lb: float, ub: float):
        """Override the global range for a state or control variable."""
        name = _canonical_name(name)
        if name in {_STATE_LABELS[i] for i in range(NUM_X)}:
            idx = [i for i in range(NUM_X) if _STATE_LABELS[i] == name][0]
            self._x_lo[idx] = lb
            self._x_hi[idx] = ub
            self._x_lo_hat[idx] = lb / self._x_mag[idx]
            self._x_hi_hat[idx] = ub / self._x_mag[idx]
        elif name in {_CTRL_LABELS[i] for i in range(NUM_U)}:
            idx = [i for i in range(NUM_U) if _CTRL_LABELS[i] == name][0]
            self._u_lo[idx] = lb
            self._u_hi[idx] = ub
            self._u_lo_hat[idx] = lb / self._u_mag[idx]
            self._u_hi_hat[idx] = ub / self._u_mag[idx]

    # -- Solve -----------------------------------------------------------

    def solve(self) -> TrajOptResult:
        """Transcribe and solve the NLP. Returns a ``TrajOptResult``."""
        N = self.cfg.num_nodes
        tau = self._tau
        kappa_nodes = self._kappa_nodes

        # ---------- trim (equilibrium initial guess) ----------
        x_trim, u_trim = self._solve_trim()

        # ---------- NLP variables ----------
        w, w0, lbw, ubw = [], [], [], []
        g, lbg, ubg = [], [], []
        J = ca.SX(0)

        kappa_par = ca.SX.sym("kappa_par", N)

        # Initial state
        Xk = ca.SX.sym("X0", NUM_X)
        w.append(Xk)

        v_lb, v_ub = self.cfg.start_velocity
        if v_lb < 3.0 or v_ub < 3.0:
            raise ValueError("Use an initial speed of at least 3 m/s. Values below that make the path-domain model ill-conditioned.")
        lb_init = np.array(self._x_lo_hat)
        ub_init = np.array(self._x_hi_hat)
        lb_init[StateIdx.s] = self.track.s_start / self._x_mag[StateIdx.s]
        ub_init[StateIdx.s] = self.track.s_start / self._x_mag[StateIdx.s]
        lb_init[StateIdx.t] = 0.0
        ub_init[StateIdx.t] = 0.0
        lb_init[StateIdx.V] = max(v_lb / self._x_mag[StateIdx.V], self._x_lo_hat[StateIdx.V])
        ub_init[StateIdx.V] = min(v_ub / self._x_mag[StateIdx.V], self._x_hi_hat[StateIdx.V])
        lb_init[StateIdx.wr] = max(v_lb / self._x_mag[StateIdx.wr], self._x_lo_hat[StateIdx.wr])
        ub_init[StateIdx.wr] = min(v_ub / self._x_mag[StateIdx.wr], self._x_hi_hat[StateIdx.wr])
        self._apply_initial_state_bounds(lb_init, ub_init)

        lbw.extend(lb_init.tolist())
        ubw.extend(ub_init.tolist())
        w0.extend(x_trim.tolist())

        # wr ≈ V at start
        g.append(Xk[StateIdx.wr] - Xk[StateIdx.V])
        lbg.append(-0.001)
        ubg.append(0.001)

        e_lo_nodes, e_hi_nodes = self.track.e_bounds_at(self._s_nodes)

        time_start, time_end = None, None

        for k in range(N - 1):
            past_Xk = Xk

            # Control
            Uk = ca.SX.sym(f"U_{k}", NUM_U)
            w.append(Uk)
            if k == 0 and self.cfg.initial_control_rate_zero:
                lbw.extend([0.0] * NUM_U)
                ubw.extend([0.0] * NUM_U)
            else:
                lbw.extend(self._u_lo_hat.tolist())
                ubw.extend(self._u_hi_hat.tolist())
            w0.extend(u_trim.tolist())

            # Next state
            k1 = k + 1
            Xk = ca.SX.sym(f"X_{k1}", NUM_X)

            # Dynamics defect (implicit Euler)
            h = tau[k1] - tau[k]
            f_eval = self._f_dyn(Xk, Uk, kappa_par[k1])
            defect = Xk - (past_Xk + h * f_eval)
            g.append(defect)
            lbg.extend([0.0] * NUM_X)
            ubg.extend([0.0] * NUM_X)

            # State bounds
            lbw_k = np.array(self._x_lo_hat)
            ubw_k = np.array(self._x_hi_hat)
            lbw_k[StateIdx.e] = e_lo_nodes[k1] / self._x_mag[StateIdx.e]
            ubw_k[StateIdx.e] = e_hi_nodes[k1] / self._x_mag[StateIdx.e]

            # Final-state tightening
            if k1 == N - 1:
                for idx_tight in [StateIdx.beta, StateIdx.e, StateIdx.r]:
                    lbw_k[idx_tight] = -0.1 / self._x_mag[idx_tight]
                    ubw_k[idx_tight] = 0.1 / self._x_mag[idx_tight]

            w.append(Xk)
            lbw.extend(lbw_k.tolist())
            ubw.extend(ubw_k.tolist())

            w0_k = np.array(x_trim)
            w0_k[StateIdx.s] = self._s_nodes[k1] / self._x_mag[StateIdx.s]
            w0.extend(w0_k.tolist())

            # Extra accel constraints
            for a_lo, a_hi, s_lo, s_hi in self._extra_accel_bounds:
                if s_lo <= self._s_nodes[k1] <= s_hi:
                    dx_dt = self._f_dyn_dt(Xk, Uk, kappa_par[k1])
                    V_dot = dx_dt[StateIdx.V]
                    g.append(V_dot - a_lo)
                    lbg.append(0.0)
                    ubg.append(float("inf"))
                    g.append(V_dot - a_hi)
                    lbg.append(-float("inf"))
                    ubg.append(0.0)

            # Cost: penalise control rates
            J += ca.sumsqr(Uk * self.cfg.cost_control_rate_weight)

            # Track time at track entry / exit
            if self._s_nodes[k1] >= self.track.s_start + 5.0 and time_start is None:
                time_start = Xk[StateIdx.t]
            if self._s_nodes[k1] >= self.track.s_end - 5.0 and time_end is None:
                time_end = Xk[StateIdx.t]
            
            # Steady state constraint
            if self.cfg.target_steady_velocity is not None:
                target_s, target_vel = self.cfg.target_steady_velocity
                if self._s_nodes[k1] >= target_s:
                    g.append(Xk[StateIdx.V] - target_vel / self._x_mag[StateIdx.V])
                    lbg.append(0.0)
                    ubg.append(0.0)

        # Minimum-time cost
        if time_start is not None and time_end is not None:
            J += time_end - time_start

        nlp = {
            "f": J,
            "x": ca.vertcat(*w),
            "g": ca.vertcat(*g),
            "p": kappa_par,
        }
        opts = {
            "ipopt.print_level": self.cfg.ipopt_print_level,
            "ipopt.max_iter": self.cfg.ipopt_max_iter,
            "ipopt.mu_strategy": "adaptive",
            "print_time": True,
        }
        solver = ca.nlpsol("traj_opt", "ipopt", nlp, opts)

        sol = solver(
            x0=w0,
            lbx=lbw,
            ubx=ubw,
            lbg=lbg,
            ubg=ubg,
            p=kappa_nodes,
        )

        # Parse solution
        w_opt = np.array(sol["x"]).flatten()
        x_traj, u_traj = [], []
        ptr = 0
        for k in range(N):
            x_traj.append(w_opt[ptr:ptr + NUM_X])
            ptr += NUM_X
            if k < N - 1:
                u_traj.append(w_opt[ptr:ptr + NUM_U])
                ptr += NUM_U

        x_hat = np.array(x_traj)
        u_hat = np.array(u_traj)
        x_phys = x_hat * self._x_mag
        u_phys = u_hat * self._u_mag

        # Build reference path for Frenet → Cartesian
        path_ref = FrenetPath.from_curvature(
            x_phys[:, StateIdx.s],
            lambda si: float(np.interp(si, self.track.s, self.track.kappa)),
            x0=self.track.x0,
            y0=self.track.y0,
            psi0=self.track.psi0,
        )

        return TrajOptResult(
            x_hat=x_hat,
            u_hat=u_hat,
            x=x_phys,
            u=u_phys,
            cost=float(sol["f"]),
            solver_stats=solver.stats(),
            track=self.track,
            path_ref=path_ref,
            vehicle_params=self.veh_params,
        )

    # -- Trim solver (private) -------------------------------------------

    def _solve_trim(self) -> Tuple[np.ndarray, np.ndarray]:
        """Find a steady-state (trim) point to use as initial guess."""
        x_sym = ca.SX.sym("x", NUM_X)
        u_sym = ca.SX.sym("u", NUM_U)
        kappa_sym = ca.SX.sym("kappa")

        _, f_dt, _ = _build_dynamics_function(
            self.model, self._x_mag, self._u_mag, self._sigma,
        )
        dx_dt = f_dt(x_sym, u_sym, kappa_sym)

        cost = (
            dx_dt[StateIdx.r] ** 2
            + dx_dt[StateIdx.V] ** 2
            + dx_dt[StateIdx.beta] ** 2
            + dx_dt[StateIdx.wr] ** 2
        )
        f_cost = ca.Function("trim_cost", [x_sym, u_sym, kappa_sym], [cost])

        z = ca.SX.sym("z", NUM_X + NUM_U)
        kp = ca.SX.sym("kp")
        x_d = z[:NUM_X]
        u_d = z[NUM_X:]
        nlp = {"f": f_cost(x_d, u_d, kp), "x": z, "g": ca.SX(0, 1), "p": kp}

        lb = np.concatenate([self._x_lo_hat, self._u_lo_hat])
        ub = np.concatenate([self._x_hi_hat, self._u_hi_hat])

        # Tighten trim bounds
        v_mid = 0.5 * sum(self.cfg.start_velocity)
        lb[StateIdx.V] = max(v_mid * 0.9 / self._x_mag[StateIdx.V], lb[StateIdx.V])
        ub[StateIdx.V] = min(v_mid * 1.1 / self._x_mag[StateIdx.V], ub[StateIdx.V])
        lb[StateIdx.s] = self.track.s_start / self._x_mag[StateIdx.s]
        ub[StateIdx.s] = self.track.s_start / self._x_mag[StateIdx.s]
        lb[StateIdx.t] = 0.0
        ub[StateIdx.t] = 0.0
        if self.cfg.initial_state_bounds:
            for raw_name, bounds in self.cfg.initial_state_bounds.items():
                name = _canonical_name(raw_name)
                if name not in {_STATE_LABELS[i] for i in range(NUM_X)}:
                    continue
                idx = [i for i in range(NUM_X) if _STATE_LABELS[i] == name][0]
                lb_phys, ub_phys = bounds
                lb[idx] = max(lb_phys / self._x_mag[idx], lb[idx])
                ub[idx] = min(ub_phys / self._x_mag[idx], ub[idx])

        lb = np.clip(lb, np.concatenate([self._x_lo_hat, self._u_lo_hat]),
                      np.concatenate([self._x_hi_hat, self._u_hi_hat]))
        ub = np.clip(ub, np.concatenate([self._x_lo_hat, self._u_lo_hat]),
                      np.concatenate([self._x_hi_hat, self._u_hi_hat]))

        z0 = 0.5 * (lb + ub)
        solver = ca.nlpsol(
            "trim", "ipopt", nlp,
            {"ipopt.print_level": 0, "print_time": False},
        )
        sol = solver(x0=z0, lbx=lb, ubx=ub, lbg=[], ubg=[], p=self._kappa_nodes[0])
        z_opt = np.array(sol["x"]).flatten()
        return z_opt[:NUM_X], z_opt[NUM_X:]
