"""Reference-free Week 11 simulator scaffold.

This simulator keeps the vehicle state directly in Cartesian pose plus
speed/sideslip coordinates:

- x, y, psi
- V, beta, yaw_rate, rear_wheelspeed_ms

It reuses the Week 10 Fiala bicycle model for the dynamic states while keeping
the simulator independent from any reference path, curvature profile, or Frenet
projection utility.

Assignment-facing entry points:
- SimulatorNoise: measurement-noise settings for each reported channel.
- SimulatorConfig: integration time step, substeps, random seed, and noise
    configuration.
- InitialVehicleState: initial condition for reset(...).
- FialaBicycleCartesianSimulator(...): construct the simulator.
- reset(initial_state): initialize the simulator state.
- step(roadwheel_angle=..., rear_wheel_torque=..., brake=...): advance one time
    step.
- get_state(noisy=True/False): read the current vehicle state.
- get_history(): retrieve the logged rollout for plotting or analysis.

State convention used by the simulator:
- pose: x, y, psi
- dynamic states: V, beta, yaw_rate, rear_wheelspeed_ms
- controls: roadwheel_angle, rear_wheel_torque, brake

Typical assignment flow:
1. Construct the simulator with a chosen dt and noise model.
2. Reset it near the start of the chosen reference path.
3. At each control update, read the noisy state estimate.
4. Pass that measurement through the tracking helper and MPC.
5. Apply the returned control with step(...).
6. Use get_history() for closed-loop plots at the end.
"""

from __future__ import annotations

import copy
import importlib
import sys
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Dict, Optional

import casadi as ca
import numpy as np


_THIS_DIR = Path(__file__).resolve().parent
_WEEK10_DIR = _THIS_DIR.parent / "week10"
if str(_WEEK10_DIR) not in sys.path:
    sys.path.insert(0, str(_WEEK10_DIR))

_week10_dynamics = importlib.import_module("casadi_dynamics")

FialaBicycleCasADi = _week10_dynamics.FialaBicycleCasADi
fiala_params = _week10_dynamics.fiala_params


def _wrap_to_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


@dataclass(frozen=True)
class SimulatorNoise:
    x: float = 0.02
    y: float = 0.02
    psi: float = 0.002
    V: float = 0.05
    beta: float = 0.003
    yaw_rate: float = 0.01
    rear_wheelspeed_ms: float = 0.05
    accel_x: float = 0.08
    accel_y: float = 0.08


@dataclass(frozen=True)
class SimulatorConfig:
    dt: float = 0.02
    integration_substeps: int = 4
    noise: SimulatorNoise = field(default_factory=SimulatorNoise)
    seed: Optional[int] = 7


@dataclass(frozen=True)
class InitialVehicleState:
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    V: float = 5.0
    beta: float = 0.0
    yaw_rate: float = 0.0
    rear_wheelspeed_ms: Optional[float] = None
    roadwheel_angle: float = 0.0
    rear_wheel_torque: float = 0.0
    brake: float = 0.0
    time: float = 0.0


class FialaBicycleCartesianSimulator:
    """Reference-free time-domain simulator around the Week 10 bicycle model."""

    STATE_ORDER = (
        "x",
        "y",
        "psi",
        "V",
        "beta",
        "yaw_rate",
        "rear_wheelspeed_ms",
    )

    CONTROL_ORDER = (
        "roadwheel_angle",
        "rear_wheel_torque",
        "brake",
    )

    def __init__(
        self,
        *,
        config: Optional[SimulatorConfig] = None,
        mu_front: Optional[float] = None,
        mu_rear: Optional[float] = None,
        vehicle_params: Optional[dict] = None,
    ):
        self.config = config or SimulatorConfig()
        self.rng = np.random.default_rng(self.config.seed)

        self.params = copy.deepcopy(vehicle_params or fiala_params)
        if mu_front is not None:
            self.params["tires"]["mu_front"] = float(mu_front)
        if mu_rear is not None:
            self.params["tires"]["mu_rear"] = float(mu_rear)

        self.model = FialaBicycleCasADi(self.params)
        self._dx_fun, self._feature_fun, self._feature_names = self._build_casadi_functions()

        self._true_state: Dict[str, float] = {}
        self._measured_state: Dict[str, float] = {}
        self._last_features: Dict[str, float] = {}
        self._history: list[dict] = []
        self.reset()

    def _build_casadi_functions(self):
        state_sym = ca.SX.sym("state", len(self.STATE_ORDER))
        control_sym = ca.SX.sym("control", len(self.CONTROL_ORDER))

        psi = state_sym[2]
        speed = state_sym[3]
        beta = state_sym[4]
        yaw_rate = state_sym[5]
        rear_wheelspeed = state_sym[6]

        roadwheel_angle = control_sym[0]
        rear_wheel_torque = control_sym[1]
        brake = control_sym[2]

        vel_x = speed * ca.cos(beta)
        vel_y = speed * ca.sin(beta)
        state_control = {
            "vel_x": vel_x,
            "vel_y": vel_y,
            "yaw_rate": yaw_rate,
            "rear_wheelspeed_ms": rear_wheelspeed,
            "roadwheel_angle": roadwheel_angle,
            "rear_wheel_torque": rear_wheel_torque,
            "brake": brake,
            "V": speed,
            "beta": beta,
        }
        feats = self.model.get_vectorfield(state_control)

        x_dot = speed * ca.cos(psi + beta)
        y_dot = speed * ca.sin(psi + beta)
        psi_dot = yaw_rate

        state_dot = ca.vertcat(
            x_dot,
            y_dot,
            psi_dot,
            feats["V_dot"],
            feats["beta_dot"],
            feats["r_dot"],
            feats["wr_dot"],
        )

        feature_names = (
            "vel_x",
            "vel_y",
            "accel_x",
            "accel_y",
            "alpha_f",
            "alpha_r",
            "kappa_x_front",
            "kappa_x_rear",
            "Fxf",
            "Fyf",
            "Fxr",
            "Fyr",
            "Fzf",
            "Fzr",
            "deltaFz",
            "V_dot",
            "beta_dot",
            "tire_saturation",
            "tire_energy",
        )
        feature_vector = ca.vertcat(vel_x, vel_y, *[feats[name] for name in feature_names[2:]])

        dx_fun = ca.Function("week11_cartesian_dx_dt", [state_sym, control_sym], [state_dot])
        feature_fun = ca.Function("week11_cartesian_features", [state_sym, control_sym], [feature_vector])
        return dx_fun, feature_fun, feature_names

    def reset(self, initial_state: Optional[InitialVehicleState] = None) -> Dict[str, float]:
        init = initial_state or InitialVehicleState()
        wr0 = init.rear_wheelspeed_ms
        if wr0 is None:
            wr0 = max(float(init.V) * np.cos(float(init.beta)), 0.1)

        self._true_state = {
            "x": float(init.x),
            "y": float(init.y),
            "psi": float(init.psi),
            "V": float(init.V),
            "beta": float(init.beta),
            "yaw_rate": float(init.yaw_rate),
            "rear_wheelspeed_ms": float(wr0),
            "roadwheel_angle": float(init.roadwheel_angle),
            "rear_wheel_torque": float(init.rear_wheel_torque),
            "brake": float(init.brake),
            "time": float(init.time),
        }
        self._last_features = self._evaluate_features(self._true_state)
        self._measured_state = self._sample_measurement(self._true_state, self._last_features)
        self._history = []
        self._append_history()
        return dict(self._measured_state)

    def get_state(self, *, noisy: bool = True) -> Dict[str, float]:
        return dict(self._measured_state if noisy else self._compose_true_output())

    def get_history(self) -> Dict[str, np.ndarray]:
        if not self._history:
            return {}
        keys = self._history[0].keys()
        return {key: np.asarray([row[key] for row in self._history], dtype=float) for key in keys}

    def step(
        self,
        *,
        roadwheel_angle: float,
        rear_wheel_torque: float,
        brake: float = 0.0,
        dt: Optional[float] = None,
    ) -> Dict[str, float]:
        step_dt = float(self.config.dt if dt is None else dt)
        if step_dt <= 0.0:
            raise ValueError("dt must be > 0")

        self._true_state["roadwheel_angle"] = float(roadwheel_angle)
        self._true_state["rear_wheel_torque"] = float(rear_wheel_torque)
        self._true_state["brake"] = float(brake)

        control_vec = np.array([roadwheel_angle, rear_wheel_torque, brake], dtype=float)
        state_vec = self._state_vector_from_true()

        n_substeps = max(1, int(self.config.integration_substeps))
        dt_sub = step_dt / n_substeps
        for _ in range(n_substeps):
            state_vec = self._rk4_step(state_vec, control_vec, dt_sub)
            state_vec[3] = max(state_vec[3], 0.0)
            state_vec[6] = max(state_vec[6], 0.0)

        for key, value in zip(self.STATE_ORDER, state_vec):
            self._true_state[key] = float(value)
        self._true_state["time"] += step_dt

        self._last_features = self._evaluate_features(self._true_state)
        self._measured_state = self._sample_measurement(self._true_state, self._last_features)
        self._append_history()
        return dict(self._measured_state)

    def _rk4_step(self, state_vec: np.ndarray, control_vec: np.ndarray, dt: float) -> np.ndarray:
        k1 = self._dx(state_vec, control_vec)
        k2 = self._dx(state_vec + 0.5 * dt * k1, control_vec)
        k3 = self._dx(state_vec + 0.5 * dt * k2, control_vec)
        k4 = self._dx(state_vec + dt * k3, control_vec)
        return state_vec + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def _dx(self, state_vec: np.ndarray, control_vec: np.ndarray) -> np.ndarray:
        return np.asarray(self._dx_fun(state_vec, control_vec)).reshape(-1)

    def _evaluate_features(self, state: Dict[str, float]) -> Dict[str, float]:
        state_vec = np.array([state[name] for name in self.STATE_ORDER], dtype=float)
        control_vec = np.array([state[name] for name in self.CONTROL_ORDER], dtype=float)
        values = np.asarray(self._feature_fun(state_vec, control_vec)).reshape(-1)
        return {name: float(value) for name, value in zip(self._feature_names, values)}

    def _state_vector_from_true(self) -> np.ndarray:
        return np.array([self._true_state[name] for name in self.STATE_ORDER], dtype=float)

    def _compose_true_output(self) -> Dict[str, float]:
        output = dict(self._true_state)
        output.update(self._last_features)
        return output

    def _sample_measurement(
        self,
        true_state: Dict[str, float],
        features: Dict[str, float],
    ) -> Dict[str, float]:
        measured_speed = float(true_state["V"] + self.rng.normal(0.0, self.config.noise.V))
        measured_speed = max(measured_speed, 0.0)
        measured_beta = _wrap_to_pi(true_state["beta"] + self.rng.normal(0.0, self.config.noise.beta))

        measurement = {
            "time": float(true_state["time"]),
            "x": float(true_state["x"] + self.rng.normal(0.0, self.config.noise.x)),
            "y": float(true_state["y"] + self.rng.normal(0.0, self.config.noise.y)),
            "psi": _wrap_to_pi(true_state["psi"] + self.rng.normal(0.0, self.config.noise.psi)),
            "V": measured_speed,
            "beta": float(true_state["beta"] + self.rng.normal(0.0, self.config.noise.beta)),
            "yaw_rate": float(true_state["yaw_rate"] + self.rng.normal(0.0, self.config.noise.yaw_rate)),
            "rear_wheelspeed_ms": float(
                true_state["rear_wheelspeed_ms"]
                + self.rng.normal(0.0, self.config.noise.rear_wheelspeed_ms)
            ),
            "roadwheel_angle": float(true_state["roadwheel_angle"]),
            "rear_wheel_torque": float(true_state["rear_wheel_torque"]),
            "brake": float(true_state["brake"]),
            "vel_x": float(measured_speed * np.cos(measured_beta)),
            "vel_y": float(measured_speed * np.sin(measured_beta)),
            "accel_x": float(features["accel_x"] + self.rng.normal(0.0, self.config.noise.accel_x)),
            "accel_y": float(features["accel_y"] + self.rng.normal(0.0, self.config.noise.accel_y)),
        }
        return measurement

    def _append_history(self) -> None:
        true_output = self._compose_true_output()
        record = {f"true_{key}": value for key, value in true_output.items()}
        record.update({f"meas_{key}": value for key, value in self._measured_state.items()})
        self._history.append(record)


def noise_config_from_dict(values: Optional[dict] = None) -> SimulatorNoise:
    if values is None:
        return SimulatorNoise()
    valid = {field.name for field in fields(SimulatorNoise)}
    unknown = set(values) - valid
    if unknown:
        raise KeyError(f"Unknown noise keys: {sorted(unknown)}")
    return SimulatorNoise(**values)


__all__ = [
    "FialaBicycleCartesianSimulator",
    "InitialVehicleState",
    "SimulatorConfig",
    "SimulatorNoise",
    "noise_config_from_dict",
]