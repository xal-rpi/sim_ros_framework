"""Week 11 receding-horizon tracking MPC helper.

The controller is intentionally compact and educational:
- the horizon uses a fixed ds grid anchored at the current s position
- the reference trajectory is sampled internally from that current s plus s-grid
- the vehicle dynamics use the same Fiala bicycle model as the simulator
- the state follows the Week 10 V / beta path-domain convention
- the transcription uses scaled variables and implicit Euler in s
- s is provided by the fixed preview grid rather than optimized as a state

Assignment-facing entry points:
- TrackingMPCConfig: configuration for horizon length, fixed ds, bounds, and
    tracking weights.
- FrenetTrackingMPC(...): build the nonlinear tracking MPC once and reuse it at
    every control step.
- FrenetTrackingMPC.solve(current_state, ref_window, prev_control=...): solve
    one receding-horizon problem from the current Frenet state and an already
    extracted reference window over the horizon.
- FrenetTrackingMPC.solve_with_ref_traj(current_state, ref_traj, ds=...,
    prev_control=...): convenience wrapper that first extracts the horizon window
    and then calls solve(...).

Expected inputs to solve(...):
- current_state should contain the current Frenet/path state, including s,
    t/time, r/yaw_rate, V, beta, wr/rear_wheelspeed_ms, e, and dphi.
- ref_window should be the already sampled horizon window returned by
    TrackingReference.get_ref_traj(...), including s, s_grid, kappa, e_min,
    e_max, and the state-reference profiles.

Main outputs from solve(...):
- u0: first control to apply to the simulator.
- x_pred: predicted state rollout over the horizon, including absolute s and the
    relative s_grid.
- u_pred: predicted control rollout.
- ref_traj: the interpolated reference window actually used by the solver.

Typical assignment flow:
1. Convert the simulator measurement to Frenet coordinates.
2. Extract ref_window = reference.get_ref_traj(...).
3. Build the current_state dictionary.
4. Call solve(...) with that ref_window.
4. Apply u0 to the simulator.
5. Plot x_pred against the sampled reference and the closed-loop trajectory.
"""

from __future__ import annotations

import copy
import importlib
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import casadi as ca
import numpy as np

from tracking_helper import TrackingReference


_THIS_DIR = Path(__file__).resolve().parent
_WEEK10_DIR = _THIS_DIR.parent / "week10"
if str(_WEEK10_DIR) not in sys.path:
    sys.path.insert(0, str(_WEEK10_DIR))

_week10_dynamics = importlib.import_module("casadi_dynamics")

FialaBicycleCasADi = _week10_dynamics.FialaBicycleCasADi
fiala_params = _week10_dynamics.fiala_params


@dataclass(frozen=True)
class TrackingMPCConfig:
    horizon_steps: int = 15
    prediction_ds: Optional[float] = None
    max_steer: float = 0.35
    max_steer_rate: float = 0.9
    min_torque: float = -1500.0
    max_torque: float = 3500.0
    max_torque_rate: float = 5000.0
    min_speed: float = 1.0
    max_speed: float = 30.0
    max_lateral_error: float = 6.0
    weight_time: float = 0.0
    weight_beta: float = 0.0
    weight_wr: float = 0.0
    weight_e: float = 0.0
    weight_dphi: float = 0.0
    weight_speed: float = 4.0
    weight_yaw_rate: float = 0.0
    weight_steer: float = 0.01
    weight_torque: float = 0.01
    weight_steer_increment: float = 0.01
    weight_torque_increment: float = 0.01
    terminal_e: float = 0.0
    terminal_dphi: float = 0.0
    ipopt_print_level: int = 0
    ipopt_max_iter: int = 1000
    accept_limited_solution: bool = True


class FrenetTrackingMPC:
    """Nonlinear receding-horizon controller using curvature preview."""

    STATE_ORDER = (
        "t",
        "r",
        "V",
        "beta",
        "wr",
        "e",
        "dphi",
    )

    CONTROL_ORDER = (
        "roadwheel_angle",
        "rear_wheel_torque",
    )

    STATE_ALIASES = {
        "t": ("time",),
        "r": ("yaw_rate",),
        "wr": ("rear_wheelspeed_ms",),
    }

    CONTROL_ALIASES = {
        "roadwheel_angle": ("delta",),
    }

    DEFAULT_RANGES = {
        "t": (0.0, 100.0),
        "r": (-1.5, 1.5),
        "V": (2.0, 30.0),
        "beta": (-0.5, 0.5),
        "wr": (2.0, 30.0),
        "e": (-10.0, 10.0),
        "dphi": (-1.2, 1.2),
        "roadwheel_angle": (-0.4, 0.4),
        "rear_wheel_torque": (-500.0, 3500.0),
    }

    DEFAULT_MAG = {
        "t": 100.0,
        "r": 1.5,
        "V": 30.0,
        "beta": 1.0,
        "wr": 30.0,
        "e": 10.0,
        "dphi": 1.0,
        "roadwheel_angle": 0.5,
        "rear_wheel_torque": 3000.0,
    }

    def __init__(
        self,
        config: Optional[TrackingMPCConfig] = None,
        *,
        vehicle_params: Optional[dict] = None,
        state_bounds: Optional[dict[str, tuple[float, float]]] = None,
    ):
        self.cfg = config or TrackingMPCConfig()
        self.params = copy.deepcopy(vehicle_params or fiala_params)
        self._state_bounds_override = self._normalize_state_bounds(state_bounds)
        self.model = FialaBicycleCasADi(self.params)
        self._x_mag, self._u_mag, self._x_lo, self._x_hi, self._u_lo, self._u_hi = self._build_scaling()
        self._dyn_fun_ds, self._s_dot_fun = self._build_dynamics_function()
        self._opti, self._X, self._U, self._P = self._build_problem()
        self._warm_x: Optional[np.ndarray] = None
        self._warm_u: Optional[np.ndarray] = None

    @classmethod
    def _canonical_state_name(cls, raw_name: str) -> str:
        if raw_name in cls.STATE_ORDER:
            return raw_name
        for canonical, aliases in cls.STATE_ALIASES.items():
            if raw_name == canonical or raw_name in aliases:
                return canonical
        raise ValueError(f"Unknown state name: {raw_name}")

    @classmethod
    def _normalize_state_bounds(
        cls,
        state_bounds: Optional[dict[str, tuple[float, float]]],
    ) -> dict[str, tuple[float, float]]:
        if not state_bounds:
            return {}

        normalized: dict[str, tuple[float, float]] = {}
        for raw_name, bounds in state_bounds.items():
            name = cls._canonical_state_name(raw_name)
            lb, ub = bounds
            lb_f = float(lb)
            ub_f = float(ub)
            if lb_f > ub_f:
                raise ValueError(f"Invalid bounds for '{raw_name}': lower bound exceeds upper bound")
            normalized[name] = (lb_f, ub_f)
        return normalized

    @classmethod
    def _lookup(cls, values: dict[str, float], name: str, *, default: Optional[float] = None) -> float:
        keys = (name,) + cls.STATE_ALIASES.get(name, ()) + cls.CONTROL_ALIASES.get(name, ())
        for key in keys:
            if key in values:
                return float(values[key])
        if default is None:
            raise KeyError(f"Missing required key '{name}'")
        return float(default)

    @staticmethod
    def _speed_beta_from_xy(values: dict[str, float]) -> tuple[float, float]:
        vel_x = float(values["vel_x"])
        vel_y = float(values["vel_y"])
        return math.hypot(vel_x, vel_y), math.atan2(vel_y, vel_x + 1e-6)

    def _build_scaling(self):
        mag = self.DEFAULT_MAG.copy()
        x_mag = np.array([mag[name] for name in self.STATE_ORDER], dtype=float)
        u_mag = np.array([mag[name] for name in self.CONTROL_ORDER], dtype=float)

        ranges = self.DEFAULT_RANGES.copy()
        ranges["e"] = (-float(self.cfg.max_lateral_error), float(self.cfg.max_lateral_error))
        ranges.update(self._state_bounds_override)
        x_lo = np.array([ranges[name][0] for name in self.STATE_ORDER], dtype=float)
        x_hi = np.array([ranges[name][1] for name in self.STATE_ORDER], dtype=float)
        u_lo = np.array([ranges[name][0] for name in self.CONTROL_ORDER], dtype=float)
        u_hi = np.array([ranges[name][1] for name in self.CONTROL_ORDER], dtype=float)
        return x_mag, u_mag, x_lo, x_hi, u_lo, u_hi

    def _resolve_lateral_error_bounds(self, ref_window: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
        e_override = self._state_bounds_override.get("e")
        if e_override is not None:
            e_lb, e_ub = e_override
            horizon_len = len(np.asarray(ref_window["s"], dtype=float))
            return (
                np.full(horizon_len, e_lb, dtype=float),
                np.full(horizon_len, e_ub, dtype=float),
            )

        return (
            np.asarray(ref_window["e_min"], dtype=float),
            np.asarray(ref_window["e_max"], dtype=float),
        )

    def _build_dynamics_function(self):
        x_hat = ca.SX.sym("x_hat", len(self.STATE_ORDER))
        u_hat = ca.SX.sym("u_hat", len(self.CONTROL_ORDER))
        kappa = ca.SX.sym("kappa")

        x = x_hat * self._x_mag
        u = u_hat * self._u_mag

        yaw_rate = x[1]
        speed = x[2]
        beta = x[3]
        rear_wheelspeed = x[4]
        e = x[5]
        dphi = x[6]
        roadwheel_angle = u[0]
        rear_wheel_torque = u[1]

        feats = self.model.get_vectorfield(
            {
                "r": yaw_rate,
                "V": speed,
                "beta": beta,
                "wr": rear_wheelspeed,
                "delta": roadwheel_angle,
                "rear_wheel_torque": rear_wheel_torque,
                "brake": 0.0,
            }
        )

        safe_denom = 1.0 - e * kappa
        s_dot = (speed * ca.cos(dphi)) / safe_denom
        e_dot = speed * ca.sin(dphi)
        dphi_dot = feats["beta_dot"] + yaw_rate - kappa * s_dot

        x_s = ca.vertcat(
            1.0 / s_dot,
            feats["r_dot"] / s_dot,
            feats["V_dot"] / s_dot,
            feats["beta_dot"] / s_dot,
            feats["wr_dot"] / s_dot,
            e_dot / s_dot,
            dphi_dot / s_dot,
        )
        x_s_hat = x_s / self._x_mag
        return (
            ca.Function("week11_tracking_dynamics_ds", [x_hat, u_hat, kappa], [x_s_hat]),
            ca.Function("week11_tracking_s_dot", [x_hat, u_hat, kappa], [s_dot]),
        )

    def _implicit_euler_defect(self, xk_hat, xkp1_hat, uk_hat, kappakp1, ds_k):
        return xkp1_hat - xk_hat - ds_k * self._dyn_fun_ds(xkp1_hat, uk_hat, kappakp1)

    def _build_problem(self):
        N = self.cfg.horizon_steps
        nx = len(self.STATE_ORDER)
        nu = len(self.CONTROL_ORDER)

        opti = ca.Opti()
        X = opti.variable(nx, N + 1)
        U = opti.variable(nu, N)

        # All varibales here are in scaled form.
        # Except for the curvature preview and lateral error bounds,
        # which are more intuitive to provide in physical units.
        X0 = opti.parameter(nx)
        U_PREV = opti.parameter(nu)
        UREF = opti.parameter(nu, N)
        SGRID = opti.parameter(N + 1)
        KAPPA = opti.parameter(N + 1)
        XREF = opti.parameter(nx, N + 1)
        EMIN = opti.parameter(N + 1)
        EMAX = opti.parameter(N + 1)

        x_lo_hat = self._x_lo / self._x_mag
        x_hi_hat = self._x_hi / self._x_mag
        u_lo_hat = self._u_lo / self._u_mag
        u_hi_hat = self._u_hi / self._u_mag

        opti.subject_to(X[:, 0] == X0)

        objective = 0
        for k in range(N):
            xk = X[:, k]
            xkp1 = X[:, k + 1]
            uk = U[:, k]
            ds_k = SGRID[k + 1] - SGRID[k]

            opti.subject_to(self._implicit_euler_defect(xk, xkp1, uk, KAPPA[k + 1], ds_k) == 0)

            err_k = xkp1 - XREF[:, k + 1]
            u_err_k = uk - UREF[:, k]
            u_prev = U_PREV if k == 0 else U[:, k - 1]
            du_k = uk - u_prev

            objective += self.cfg.weight_time * err_k[0] ** 2
            objective += self.cfg.weight_yaw_rate * err_k[1] ** 2
            objective += self.cfg.weight_speed * err_k[2] ** 2
            objective += self.cfg.weight_beta * err_k[3] ** 2
            objective += self.cfg.weight_wr * err_k[4] ** 2
            objective += self.cfg.weight_e * err_k[5] ** 2
            objective += self.cfg.weight_dphi * err_k[6] ** 2
            objective += self.cfg.weight_steer * u_err_k[0] ** 2
            objective += self.cfg.weight_torque * u_err_k[1] ** 2
            objective += self.cfg.weight_steer_increment * du_k[0] ** 2
            objective += self.cfg.weight_torque_increment * du_k[1] ** 2

            opti.subject_to(opti.bounded(u_lo_hat[0], uk[0], u_hi_hat[0]))
            opti.subject_to(opti.bounded(u_lo_hat[1], uk[1], u_hi_hat[1]))

            # If time-domain slew-rate bounds are needed while integrating in s,
            # use dt ~= ds / s_dot and constrain s_dot * (u_k - u_{k-1}) / ds_k.
            # s_dot_k = self._s_dot_fun(xkp1, uk, KAPPA[k + 1])
            # du_dt_k = s_dot_k * ((uk - u_prev) * self._u_mag) / ds_k
            # opti.subject_to(opti.bounded(-self.cfg.max_steer_rate, du_dt_k[0], self.cfg.max_steer_rate))
            # opti.subject_to(opti.bounded(-self.cfg.max_torque_rate, du_dt_k[1], self.cfg.max_torque_rate))

            for idx in range(nx):
                opti.subject_to(opti.bounded(x_lo_hat[idx], xkp1[idx], x_hi_hat[idx]))

            opti.subject_to(xkp1[5] >= EMIN[k + 1] / self._x_mag[5])
            opti.subject_to(xkp1[5] <= EMAX[k + 1] / self._x_mag[5])

        err_terminal = X[:, N] - XREF[:, N]
        objective += self.cfg.terminal_e * err_terminal[5] ** 2
        objective += self.cfg.terminal_dphi * err_terminal[6] ** 2
        objective += self.cfg.weight_speed * err_terminal[2] ** 2

        opti.minimize(objective)
        opti.solver(
            "ipopt",
            {
                "ipopt.print_level": self.cfg.ipopt_print_level,
                "ipopt.max_iter": self.cfg.ipopt_max_iter,
                "print_time": True,
            },
        )
        parameters = {
            "X0": X0,
            "U_PREV": U_PREV,
            "UREF": UREF,
            "SGRID": SGRID,
            "KAPPA": KAPPA,
            "XREF": XREF,
            "EMIN": EMIN,
            "EMAX": EMAX,
        }
        return opti, X, U, parameters

    def _resolve_prediction_ds(self, ds: Optional[float]) -> float:
        ds_use = self.cfg.prediction_ds if ds is None else ds
        if ds_use is None:
            raise ValueError("A positive ds must be provided either in TrackingMPCConfig.prediction_ds or solve(..., ds=...)")
        ds_f = float(ds_use)
        if ds_f <= 0.0:
            raise ValueError("Prediction ds must be strictly positive")
        return ds_f

    def _extract_ref_traj(
        self,
        ref_traj: TrackingReference | dict[str, np.ndarray],
        current_s: float,
        s_grid: np.ndarray,
    ) -> dict[str, np.ndarray]:
        s_abs = float(current_s) + s_grid

        if isinstance(ref_traj, TrackingReference):
            sampled = ref_traj.sample(s_abs)
            sampled = {key: np.asarray(value, dtype=float) for key, value in sampled.items()}
            sampled["s_wrapped"] = np.array(sampled["s"], copy=True)
        else:
            s_ref = np.asarray(ref_traj["s_unwrapped"], dtype=float) if "s_unwrapped" in ref_traj else np.asarray(ref_traj["s"], dtype=float)
            sampled = {
                "kappa": np.interp(s_abs, s_ref, np.asarray(ref_traj["kappa"], dtype=float)),
                "e_min": np.interp(s_abs, s_ref, np.asarray(ref_traj["e_min"], dtype=float)),
                "e_max": np.interp(s_abs, s_ref, np.asarray(ref_traj["e_max"], dtype=float)),
                "t": np.interp(s_abs, s_ref, np.asarray(ref_traj["t"], dtype=float) if "t" in ref_traj else np.asarray(ref_traj["time"], dtype=float)),
                "r": np.interp(s_abs, s_ref, np.asarray(ref_traj["r"], dtype=float) if "r" in ref_traj else np.asarray(ref_traj["yaw_rate"], dtype=float)),
                "V": np.interp(s_abs, s_ref, np.asarray(ref_traj["V"], dtype=float) if "V" in ref_traj else np.asarray(ref_traj["speed"], dtype=float)),
                "beta": np.interp(s_abs, s_ref, np.asarray(ref_traj["beta"], dtype=float)),
                "wr": np.interp(s_abs, s_ref, np.asarray(ref_traj["wr"], dtype=float) if "wr" in ref_traj else np.asarray(ref_traj["rear_wheelspeed_ms"], dtype=float)),
                "e": np.interp(s_abs, s_ref, np.asarray(ref_traj["e"], dtype=float)),
                "dphi": np.interp(s_abs, s_ref, np.asarray(ref_traj["dphi"], dtype=float)),
            }
            if "s" in ref_traj:
                sampled["s_wrapped"] = np.interp(s_abs, s_ref, np.asarray(ref_traj["s"], dtype=float))

        sampled["speed"] = np.array(sampled["V"], copy=True)
        sampled["s"] = np.array(s_abs, copy=True)
        sampled["s_grid"] = np.array(s_grid, copy=True)
        return sampled

    def _build_reference_state(self, ref_traj: dict[str, np.ndarray]) -> np.ndarray:
        defaults = {
            "t": np.asarray(ref_traj["t"], dtype=float),
            "r": np.asarray(ref_traj["r"], dtype=float),
            "V": np.asarray(ref_traj["V"], dtype=float),
            "beta": np.asarray(ref_traj["beta"], dtype=float),
            "wr": np.asarray(ref_traj["wr"], dtype=float),
            "e": np.asarray(ref_traj["e"], dtype=float),
            "dphi": np.asarray(ref_traj["dphi"], dtype=float),
        }
        return np.vstack([defaults[name] for name in self.STATE_ORDER])

    def _build_reference_control(self, ref_traj: dict[str, np.ndarray], horizon_steps: int) -> np.ndarray:
        controls = {
            "roadwheel_angle": np.asarray(ref_traj["delta"], dtype=float),
            "rear_wheel_torque": np.asarray(ref_traj["rear_wheel_torque"], dtype=float),
        }
        return np.vstack([controls[name][:horizon_steps] for name in self.CONTROL_ORDER])

    def _prepare_ref_window(self, current_state: dict[str, float], ref_window: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray, dict[str, np.ndarray]]:
        s0 = float(current_state["s"])
        s_abs = np.asarray(ref_window["s_unwrapped"], dtype=float) if "s_unwrapped" in ref_window else np.asarray(ref_window["s"], dtype=float)
        s_grid = np.asarray(ref_window["s_grid"], dtype=float) if "s_grid" in ref_window else s_abs - s0
        prepared = {key: np.asarray(value, dtype=float) for key, value in ref_window.items()}
        prepared["s"] = np.array(s_abs, copy=True)
        prepared["s_grid"] = np.array(s_grid, copy=True)
        if "speed" not in prepared and "V" in prepared:
            prepared["speed"] = np.array(prepared["V"], copy=True)
        return s0, s_grid, prepared

    def _coerce_state(self, current_state: dict[str, float]) -> np.ndarray:
        values = dict(current_state)
        if "V" not in values and "vel_x" in values and "vel_y" in values:
            speed, beta = self._speed_beta_from_xy(values)
            values["V"] = speed
            values["beta"] = beta

        return np.array(
            [
                self._lookup(values, "t", default=values.get("time", 0.0)),
                self._lookup(values, "r"),
                self._lookup(values, "V"),
                self._lookup(values, "beta"),
                self._lookup(values, "wr"),
                self._lookup(values, "e"),
                self._lookup(values, "dphi"),
            ],
            dtype=float,
        )

    def solve(
        self,
        current_state: dict[str, float],
        ref_window: dict[str, np.ndarray],
        *,
        prev_control: Optional[dict[str, float] | np.ndarray] = None,
    ) -> dict[str, object]:
        s0, s_grid, ref_window = self._prepare_ref_window(current_state, ref_window)
        N = len(s_grid) - 1

        x0 = self._coerce_state(current_state)
        x0_hat = x0 / self._x_mag
        x_ref = self._build_reference_state(ref_window)
        x_ref_hat = x_ref / self._x_mag.reshape(-1, 1)
        u_ref = self._build_reference_control(ref_window, N)
        u_ref_hat = u_ref / self._u_mag.reshape(-1, 1)

        if prev_control is None:
            u_prev = np.array(
                [
                    self._lookup(current_state, "roadwheel_angle", default=0.0),
                    self._lookup(current_state, "rear_wheel_torque", default=0.0),
                ],
                dtype=float,
            )
        elif isinstance(prev_control, np.ndarray):
            u_prev = np.asarray(prev_control, dtype=float).reshape(2)
        else:
            u_prev = np.array(
                [
                    self._lookup(prev_control, "roadwheel_angle", default=0.0),
                    self._lookup(prev_control, "rear_wheel_torque", default=0.0),
                ],
                dtype=float,
            )
        u_prev_hat = u_prev / self._u_mag

        self._opti.set_value(self._P["X0"], x0_hat)
        self._opti.set_value(self._P["U_PREV"], u_prev_hat)
        self._opti.set_value(self._P["UREF"], u_ref_hat)
        self._opti.set_value(self._P["SGRID"], s_grid)
        self._opti.set_value(self._P["KAPPA"], np.asarray(ref_window["kappa"], dtype=float))
        self._opti.set_value(self._P["XREF"], x_ref_hat)
        e_min, e_max = self._resolve_lateral_error_bounds(ref_window)
        self._opti.set_value(self._P["EMIN"], e_min)
        self._opti.set_value(self._P["EMAX"], e_max)

        if self._warm_x is None:
            warm_x = np.array(x_ref_hat, copy=True)
            warm_x[:, 0] = x0_hat
            warm_u = np.repeat(u_prev_hat.reshape(-1, 1), N, axis=1)
        else:
            warm_x = self._warm_x
            warm_u = self._warm_u
            warm_x = np.array(warm_x, copy=True)
            warm_x[:, 0] = x0_hat
            warm_u = np.array(warm_u, copy=True)
            warm_u[:, 0] = u_prev_hat
        self._opti.set_initial(self._X, warm_x)
        self._opti.set_initial(self._U, warm_u)

        try:
            solution = self._opti.solve()
        except RuntimeError:
            if not self.cfg.accept_limited_solution:
                raise
            solution = self._opti.solve_limited()
        x_pred_hat = np.asarray(solution.value(self._X), dtype=float)
        u_pred_hat = np.asarray(solution.value(self._U), dtype=float)
        x_pred = x_pred_hat * self._x_mag.reshape(-1, 1)
        u_pred = u_pred_hat * self._u_mag.reshape(-1, 1)
        self._warm_x = np.hstack([x_pred_hat[:, 1:], x_pred_hat[:, -1:]])
        self._warm_u = np.hstack([u_pred_hat[:, 1:], u_pred_hat[:, -1:]])

        x_pred_dict = {name: x_pred[idx, :] for idx, name in enumerate(self.STATE_ORDER)}
        x_pred_dict["s"] = s0 + np.array(s_grid, copy=True)
        x_pred_dict["s_grid"] = np.array(s_grid, copy=True)
        x_pred_dict["yaw_rate"] = x_pred_dict["r"]
        x_pred_dict["rear_wheelspeed_ms"] = x_pred_dict["wr"]
        x_pred_dict["time"] = x_pred_dict["t"]
        x_pred_dict["x_hat"] = np.array(x_pred_hat, copy=True)
        x_pred_dict["x_ref_hat"] = np.array(x_ref_hat, copy=True)

        u_pred_dict = {name: u_pred[idx, :] for idx, name in enumerate(self.CONTROL_ORDER)}
        u_pred_dict["delta"] = u_pred_dict["roadwheel_angle"]

        return {
            "u0": {
                "roadwheel_angle": float(u_pred[0, 0]),
                "rear_wheel_torque": float(u_pred[1, 0]),
            },
            "x_pred": x_pred_dict,
            "u_pred": u_pred_dict,
            "ref_traj": {key: np.array(value, copy=True) for key, value in ref_window.items()},
            "cost": float(solution.value(self._opti.f)),
            "solver_stats": self._opti.stats(),
        }

    def solve_with_ref_traj(
        self,
        current_state: dict[str, float],
        ref_traj: TrackingReference | dict[str, np.ndarray],
        *,
        ds: Optional[float] = None,
        prev_control: Optional[dict[str, float] | np.ndarray] = None,
    ) -> dict[str, object]:
        if "s" not in current_state:
            raise KeyError("current_state must contain 's'")

        ds_use = self._resolve_prediction_ds(ds)
        s_grid = ds_use * np.arange(self.cfg.horizon_steps + 1, dtype=float)
        ref_window = self._extract_ref_traj(ref_traj, float(current_state["s"]), s_grid)
        return self.solve(current_state, ref_window, prev_control=prev_control)


__all__ = [
    "FrenetTrackingMPC",
    "TrackingMPCConfig",
]
