"""
This code contains the model for the low-level as well
as a wrapper gym environment for training the model.
"""

import os
from typing import Tuple, Union, Dict, Any, List, Callable
from functools import partial
import traceback

import numpy as np

import jax
import jax.numpy as jnp

from flax import struct
import flax.linen as nn

import environment
import spaces

################################################################
####################### Gen. MLP Model #########################
################################################################


def get_activation_fn_from_name(act_fn_name: str) -> callable:
    """Get the activation function from its name."""
    if hasattr(jnp, act_fn_name):
        act_fn = getattr(jnp, act_fn_name)
    else:
        act_fn = getattr(jax.nn, act_fn_name)
    return act_fn


class MLP(nn.Module):
    """A MLP class."""

    output_dimension: int
    initial_value_range: float = 0.01
    activation_fn: Union[str, callable] = "tanh"
    layers_archictecture: Tuple = (16, 16)
    fake_normalization: bool = False

    def setup(self):
        # Cache the activation function during setup.
        self._activation_fn = (
            get_activation_fn_from_name(self.activation_fn)
            if isinstance(self.activation_fn, str)
            else self.activation_fn
        )
        # Create dense layers for each feature dimension.
        self.layers = [
            nn.Dense(
                feat_dimension,
                kernel_init=nn.initializers.uniform(scale=self.initial_value_range),
            )
            for feat_dimension in self.layers_archictecture
        ]
        # Create final dense layer.
        self.final_dense = nn.Dense(self.output_dimension)

    @nn.compact
    def __call__(self, x: jnp.ndarray) -> jnp.ndarray:
        """Forward pass of the MLP."""
        # Create the first tan-based normalization layer.
        if self.fake_normalization:
            alpha_f = self.param("alpha_f", nn.initializers.ones, ()) * 0.5
            shift_f = self.param("shift_f", nn.initializers.zeros, (x.shape[-1],))
            scale_f = self.param("scale_f", nn.initializers.ones, (x.shape[-1],))
            x = scale_f * jax.nn.tanh(x * alpha_f) + shift_f
        # Apply the dense layers.
        for dense in self.layers:
            x = dense(x)
            x = self._activation_fn(x)
        x = self.final_dense(x)
        return x


def load_network_from_config(name: str, **kwargs) -> nn.Module:
    """Load a network from a configuration dictionary."""
    if name == "MLP":
        return MLP(**kwargs)
    raise ValueError(f"Unknown network name {name}")


def create_networks(config_dict: Dict[str, Any], num_out: int = None) -> nn.Module:
    """Create the networks from the configuration dictionary."""
    if len(config_dict) <= 0:
        return None
    # Construct the neural network
    nn_type = config_dict.get("type", "MLP")
    # Create the network
    if num_out is None:
        return load_network_from_config(nn_type, **config_dict["args"])
    return load_network_from_config(
        nn_type, output_dimension=num_out, **config_dict["args"]
    )


################################################################
########################## Noise Term ##########################
################################################################
class NoiseTerm(nn.Module):
    """Template class for a learnable noise term."""

    output_dimension: int
    noise_nn: Dict[str, Any]
    max_noise: float = 0.1
    ignore_noise: bool = False
    is_constant_noise: bool = False
    stop_gradient: bool = True

    def with_stop_gradient(self, x):
        if self.stop_gradient:
            return jax.lax.stop_gradient(x)
        return x

    def setup(self):
        if self.ignore_noise or self.is_constant_noise:
            self.noise_network = None
            return
        nn_type = self.noise_nn.get("type", "MLP")
        self._noise_network = load_network_from_config(
            nn_type, output_dimension=self.output_dimension, **self.noise_nn["args"]
        )
        self.noise_network = lambda x: self._noise_network(self.with_stop_gradient(x))

    @nn.compact
    def __call__(self, x: jnp.ndarray, u: jnp.ndarray) -> jnp.ndarray:
        """Forward pass of the noise term."""
        # If ignore noise, return zeros
        if self.ignore_noise:
            return jnp.zeros(x.shape[-1]), {
                "max_noise": 0,
                "noise": jnp.zeros(x.shape[-1]),
            }

        # Check if the noise is constant
        if self.is_constant_noise:
            noise_log = self.param("noise_log", nn.initializers.zeros, (x.shape[-1],))
            noise = jnp.exp(noise_log)
            return noise, {"max_noise": jnp.sum(noise), "noise": noise}

        # Get the max noise value as parameter
        max_noise = self.param("max_noise", nn.initializers.zeros, (x.shape[-1],))
        max_noise = jnp.exp(max_noise)
        max_noise = jnp.clip(max_noise, 1.0e-4, jnp.array(self.max_noise))

        # Normalize the state
        data_state = self.get_normalized_state(x)
        noise = self.noise_network(data_state)
        # Initially some non-zero noise
        total_noise = max_noise * (1 - jax.nn.sigmoid(noise) + 1e-3)
        return total_noise, {"max_noise": max_noise, "noise": noise}

    def noise_wrapper(
        self,
        learnable_parameters: Dict[str, Any],
        state: jnp.ndarray,
        control: jnp.ndarray,
    ) -> Tuple[jnp.ndarray, Dict[str, Any]]:
        """Noise function for the noise term."""
        return self.apply(learnable_parameters, state, control)


################################################################
######################### Actual Model #########################
################################################################


class EngineModel(nn.Module):
    """A model for the engine."""

    nns: Dict[str, Any]
    state_action_mean: Dict[str, jnp.ndarray]
    state_action_std: Dict[str, jnp.ndarray]
    state_min: Dict[str, float]
    state_max: Dict[str, float]
    stop_gradient: bool = True
    _history_size: int = 10

    @property
    def name_states(self) -> Tuple[str]:
        return ["engine_speed", "boost_pressure", "rear_wheelspeed"]

    @property
    def name_controls(self) -> Tuple[str]:
        return [
            "throttle",
        ]

    @property
    def name_latents(self) -> Tuple[str]:
        return ["inv_gear_ratio", "angle_engine", "angle_wheel"]

    @property
    def bounds_latents(self) -> Tuple[Dict[str, float]]:
        MAX_VAL = 1.0e6
        dict_min = {"inv_gear_ratio": 0, "angle_engine": 0, "angle_wheel": 0}
        dict_max = {
            "inv_gear_ratio": MAX_VAL,
            "angle_engine": MAX_VAL,
            "angle_wheel": MAX_VAL,
        }
        return dict_min, dict_max

    @property
    def num_states(self) -> int:
        return len(self.name_states)

    @property
    def num_controls(self) -> int:
        return len(self.name_controls)

    @property
    def num_latent(self) -> int:
        return len(self.name_latents)

    @property
    def history_size(self) -> int:
        return self._history_size

    @property
    def names_indexes(self) -> Dict[str, int]:
        data = {name: i for i, name in enumerate(self.name_states)}
        data = {**data, **{name: i for i, name in enumerate(self.name_controls)}}
        data = {**data, **{name: i for i, name in enumerate(self.name_latents)}}
        return data

    @property
    def full_state_indexes(self) -> Dict[str, int]:
        data = {name: i for i, name in enumerate(self.name_states)}
        data = {
            **data,
            **{
                name: i + len(self.name_states)
                for i, name in enumerate(self.name_latents)
            },
        }
        return data

    def full_state_name2index(self, name):
        """
        Convert the full state (state + latent) name to index.
        """
        return self.full_state_indexes[name]

    def any_var_name2index(self, name):
        """
        Convert any state, control or latent name to index for the
        corresponding variable.
        """
        return self.names_indexes[name]

    def with_stop_gradient(self, x):
        """
        For stable training, gradient stopping can be used.
        """
        if self.stop_gradient:
            return jax.lax.stop_gradient(x)
        return x

    def get_parameter_by_name(self, name: str) -> jnp.ndarray:
        """Sample parameter."""
        return jnp.exp(self.param(name, nn.initializers.normal(stddev=1), ()))

    def normalize_state_action(self, data: Dict[str, jnp.ndarray]):
        """Normalize the state and action data."""
        return {
            name: (val - self.state_action_mean[name]) / self.state_action_std[name]
            for name, val in data.items()
            if name in self.state_action_mean
        }

    def get_normalized_state(self, state: jnp.ndarray) -> jnp.ndarray:
        """Normalize the state."""
        normalized_dict = {
            name: (val - self.state_action_mean[name]) / self.state_action_std[name]
            for name, val in zip(self.name_states, state)
        }
        return jnp.array([normalized_dict[name] for name in self.name_states])

    def get_data_from_state_and_control(self, state: jnp.ndarray, control: jnp.ndarray):
        """Get the data dictionary from the state and control."""
        res_unscaled = {
            name: state_val for name, state_val in zip(self.name_states, state)
        }
        if control is not None:
            res_unscaled = {
                **res_unscaled,
                **{
                    name: control_val
                    for name, control_val in zip(self.name_controls, control)
                },
            }
        res = self.normalize_state_action(res_unscaled)
        res = {**res, **{f"{name}_unscaled": val for name, val in res_unscaled.items()}}
        return res

    def get_data_from_full_state(self, full_state: jnp.ndarray, control: jnp.ndarray):
        """Get the data dictionary from the full state."""
        state, latent_state = self.split_state_latent_var(full_state)
        data = self.get_data_from_state_and_control(state, control)
        if full_state.shape[0] == self.num_states:
            return data
        data = {**data, **{_n: _v for _n, _v in zip(self.name_latents, latent_state)}}
        return data

    def constrain_predictions(self, state: jnp.ndarray) -> jnp.ndarray:
        """Apply constraints to the prediction."""
        clipped_state = jnp.array(
            [
                jnp.clip(state_val, self.state_min[name], self.state_max[name])
                for name, state_val in zip(self.name_states, state)
            ]
        )
        return clipped_state

    def split_state_with_history(self, states: jnp.ndarray, controls: jnp.ndarray):
        """Split the state with history."""
        assert states.ndim == 2, "State should be 2D"
        assert controls.ndim == 2, "Controls should be 2D"
        assert (
            states.shape[0] >= self.history_size + 1
        ), f"State shape {states.shape} does not match history size {self.history_size}"
        assert (
            controls.shape[0] > self.history_size
        ), f"Controls shape {controls.shape} does not match history size {self.history_size}"
        assert self.history_size > 0, "History size should be greater than 0"
        hist_states = states[: self.history_size + 1]
        hist_controls = controls[: self.history_size]
        pred_states = states[self.history_size :]
        pred_controls = controls[self.history_size :]
        return (hist_states, hist_controls), (pred_states, pred_controls)

    def split_state_latent_var(self, full_state: jnp.ndarray):
        """Extract the state and latent state from the combined state."""
        return full_state[: self.num_states], full_state[self.num_states :]

    def combine_state_and_latent(self, state: jnp.ndarray, latent_state: jnp.ndarray):
        """Combine the state and latent state."""
        return jnp.concatenate([state, latent_state])

    def constrain_latent(self, latent_state: jnp.ndarray) -> jnp.ndarray:
        """Apply constraints to the prediction."""
        min_dict, max_dict = self.bounds_latents
        clipped_state = jnp.array(
            [
                jnp.clip(state_val, min_dict[name], max_dict[name])
                for name, state_val in zip(self.name_latents, latent_state)
            ]
        )
        return clipped_state

    def convert_features_to_nn_input(self, feats, data):
        """Convert the features to neural network input."""
        return jnp.array([data[feat] for feat in feats])

    @nn.compact
    def __call__(self, state: jnp.ndarray, control: jnp.ndarray):
        return self.vector_field(state, control)

    def setup(self):
        """Setup the model."""
        self._boost_dot = create_networks(self.nns["boost_dot"], 1)
        self.boost_dot = lambda x: self._boost_dot(self.with_stop_gradient(x))
        self._engine_torque = create_networks(self.nns["engine_torque"], 1)
        self.engine_torque = lambda x: self._engine_torque(self.with_stop_gradient(x))
        self._gear_ratio_dyn = create_networks(self.nns["gear_ratio_dyn"], 1)
        self.gear_ratio_dyn = lambda x: self._gear_ratio_dyn(self.with_stop_gradient(x))
        # self.shaft_coeff1 = self.param("shaft_coeff_1", nn.initializers.normal(stddev=1), ())
        # self.shaft_coeff2 = self.param("shaft_coeff_2", nn.initializers.normal(stddev=1), ())
        self._torque_shaft = create_networks(self.nns["torque_shaft"], 1)
        self.torque_shaft = lambda x: self._torque_shaft(self.with_stop_gradient(x))
        self.tau_gear = self.param("tau_gear", nn.initializers.normal(stddev=1), ())

    def calculate_boost_pressure(self, data):
        """Calculate the boost pressure."""
        features = self.nns["boost_dot"].get(
            "features", ["engine_speed", "throttle_unscaled", "boost_pressure"]
        )
        nn_input = self.convert_features_to_nn_input(features, data)
        return self.boost_dot(nn_input)[0]

    def calculate_engine_torque(self, data):
        """Calculate the engine torque."""
        features = self.nns["engine_torque"].get(
            "features",
            ["engine_speed", "throttle_unscaled", "boost_pressure", "rear_wheelspeed"],
        )
        nn_input = self.convert_features_to_nn_input(features, data)
        return self.engine_torque(nn_input)[0]

    # def calculate_shaft_torque(self, data):
    #     """Calculate the shaft torque.
    #     """
    #     inv_gear_ratio, angle_e, angle_w = \
    #         data["inv_gear_ratio"], data["angle_engine"], data["angle_wheel"]
    #     w_e, w_r = data["engine_speed_unscaled"], data["rear_wheelspeed_unscaled"]
    #     delta_angle = inv_gear_ratio * angle_e - angle_w
    #     delta_speed = inv_gear_ratio * w_e - w_r
    #     # coeff_1, coeff_2 = jnp.exp(self.shaft_coeff1), jnp.exp(self.shaft_coeff2)
    #     # return (coeff_1 * delta_angle + coeff_2 * delta_speed)
    #     return self.torque_shaft(jnp.array([delta_angle, delta_speed]))[0]

    def calculate_shaft_torque(self, data):
        """Calculate the shaft torque."""
        # inv_gear_ratio = data["inv_gear_ratio"]
        # w_e, w_r = data["engine_speed"], data["rear_wheelspeed"]
        # return self.torque_shaft(jnp.array([inv_gear_ratio, w_e, w_r]))[0]
        if "engine_torque" in data:
            return data["engine_torque"]
        return self.calculate_engine_torque(data)

    def calculate_res_wheel_speed(self, data):
        """Calculate the rear wheel speed."""
        w_r = data["rear_wheelspeed_unscaled"]
        w_r_2 = w_r * w_r
        wr_c1, wr_c2 = self.get_parameter_by_name("wr_c1"), self.get_parameter_by_name(
            "wr_c2"
        )
        wr_c3 = self.get_parameter_by_name("wr_c3") * 0.001
        wr_res = -(wr_c1 * w_r + wr_c2 * w_r_2 + wr_c3) * 0.001
        return wr_res

    def calculate_latent_space_field(self, data):
        """Calculate the latent space field."""
        inv_gear_ratio = data["inv_gear_ratio"]
        we_u, we_r = data["engine_speed_unscaled"], data["rear_wheelspeed_unscaled"]
        # Calculate dyn of gear ratio
        feat_gr = self.nns["gear_ratio_dyn"].get(
            "features", ["engine_speed", "torque_shaft"]  # Add gear ratio?
        )
        inv_gear_ratio_eq = self.gear_ratio_dyn(
            self.convert_features_to_nn_input(feat_gr, data)
        )[0]
        tau_gear = jnp.exp(self.tau_gear)
        gear_ratio_dot = (inv_gear_ratio_eq - inv_gear_ratio) * tau_gear
        return jnp.array([gear_ratio_dot, we_u, we_r]), {
            "inv_gear_ratio_eq": inv_gear_ratio_eq
        }

    def calculate_initial_guess_for_latent(self, state, control, times):
        """Calculate the initial guess for the latent space."""
        if self.is_initializing():
            return jnp.ones(self.num_latent)
        # Otherwise, set up initial condition of latent space
        init_state = state[0]
        init_gear_ratio = (
            init_state[2] / init_state[0]
        )  # Assume no zero speed and good index
        init_latent_state = jnp.array([init_gear_ratio, 0, 0])
        dt = jnp.diff(times)

        def opt_step(latent_state, aux):
            _state, _control, _dt = aux
            _full_state = self.combine_state_and_latent(_state, latent_state)
            _data = self.get_data_from_full_state(_full_state, _control)
            # _data = self.get_data_from_state_and_control(_state, _control)
            # _data ={**_data, **{_n : _v for _n, _v in zip(self.name_latents, latent_state)}}
            _data = {**_data, "torque_shaft": self.calculate_shaft_torque(_data)}
            latent_dot, _ = self.calculate_latent_space_field(_data)
            next_latent_state = latent_state + latent_dot * _dt
            next_latent_state = self.constrain_latent(next_latent_state)
            return next_latent_state, None

        # Now let's integrate the latent space
        final_latent, _ = jax.lax.scan(
            opt_step, init_latent_state, (state[:-1], control, dt)
        )
        return final_latent

    def vector_field(self, state: jnp.ndarray, control: jnp.ndarray):
        """Compute the vector field."""
        if self.is_initializing():
            latent_init = self.calculate_initial_guess_for_latent(state, control, None)
            state = self.combine_state_and_latent(state, latent_init)
        # First, extract the latent state if it exists.
        # state, latent_state = self.split_state_latent_var(state)
        # data = self.get_data_from_state_and_control(state, control)
        # data = {**data, **{_n : _v for _n, _v in zip(self.name_latents, latent_state)}}
        data = self.get_data_from_full_state(state, control)
        # Start with the engine torque, boost pressure and rear wheel speed
        T_e = self.calculate_engine_torque(data)
        data = {**data, "engine_torque": T_e}
        dot_pb = self.calculate_boost_pressure(data)
        wr_res = self.calculate_res_wheel_speed(data)
        # Shaft torque
        inv_gear_ratio = data["inv_gear_ratio"]
        T_s = self.calculate_shaft_torque(data)
        data = {**data, "torque_shaft": T_s}
        # Wheel speed dynamics
        moment_coeffs = self.get_parameter_by_name("JeJr_1")
        gear_ratio = 1 / (inv_gear_ratio + 1.0e-4)
        dot_wr = gear_ratio * T_s * moment_coeffs + wr_res
        dot_we = T_e - T_s
        state_dot = jnp.array([dot_we, dot_pb, dot_wr])
        # Now calculate the latent space dynamics
        latent_dot, _extra_gear = self.calculate_latent_space_field(data)
        full_state_dot = self.combine_state_and_latent(state_dot, latent_dot)
        extra_return = {
            "engine_torque_est": T_e,
            "torque_shaft": T_s,
            "rear_wheel_speed_res": wr_res,
            "inv_gear_ratio_est": inv_gear_ratio,
            **_extra_gear,
        }
        return full_state_dot, extra_return

    def drift_wrapper(
        self,
        learnable_parameters: Dict[str, Any],
        state: jnp.ndarray,
        control: jnp.ndarray,
    ) -> Tuple[jnp.ndarray, Dict[str, Any]]:
        """Drift function for the engine model."""
        return self.apply(learnable_parameters, state, control)

    def initial_guess_wrapper(
        self,
        learnable_parameters: Dict[str, Any],
        state: jnp.ndarray,
        control: jnp.ndarray,
        times: jnp.ndarray,
    ) -> Tuple[jnp.ndarray, Dict[str, Any]]:
        """Initial guess function for the engine model."""
        return self.apply(
            learnable_parameters,
            state,
            control,
            times,
            method="calculate_initial_guess_for_latent",
        )


class EngineModelV2(EngineModel):
    """A model for the engine."""

    @property
    def name_latents(self) -> Tuple[str]:
        return [
            "clutch_state",
        ]

    @property
    def bounds_latents(self) -> Tuple[Dict[str, float]]:
        dict_min = {
            "clutch_state": 0,
        }
        dict_max = {
            "clutch_state": 1.0,
        }
        return dict_min, dict_max

    def setup(self):
        """Setup the model."""
        self._boost_dot = create_networks(self.nns["boost_dot"], 1)
        self.boost_dot = lambda x: self._boost_dot(self.with_stop_gradient(x))
        self._engine_torque = create_networks(self.nns["engine_torque"], 1)
        self.engine_torque = lambda x: self._engine_torque(self.with_stop_gradient(x))
        self._clutch_dyn = create_networks(self.nns["clutch_dyn"], 1)
        self.clutch_dyn = lambda x: self._clutch_dyn(self.with_stop_gradient(x))
        self._torque_shaft = create_networks(self.nns["torque_shaft"], 1)
        self.torque_shaft = lambda x: self._torque_shaft(self.with_stop_gradient(x))
        self.tau_gear = self.param("tau_gear", nn.initializers.normal(stddev=1), ())

    def calculate_engine_torque(self, data):
        """Calculate the engine torque."""
        features = self.nns["engine_torque"].get(
            "features", ["engine_speed", "throttle_unscaled", "boost_pressure"]
        )
        nn_input = self.convert_features_to_nn_input(features, data)
        return self.engine_torque(nn_input)[0]

    def calculate_shaft_torque(self, data):
        """Calculate the shaft torque."""
        eng_torque = None
        if "engine_torque" in data:
            eng_torque = data["engine_torque"]
        else:
            eng_torque = self.calculate_engine_torque(data)
        w_e = data["engine_speed"]
        w_r = data["rear_wheelspeed"]
        feats = jnp.array([w_e, w_r, eng_torque])
        return self.torque_shaft(feats)[0]

    def calculate_latent_space_field(self, data):
        """Calculate the latent space field."""
        clutch_state = data["clutch_state"]
        feat_gr = self.nns["clutch_dyn"].get(
            "features", ["engine_speed", "torque_shaft"]
        )
        clutch_state_eq = self.clutch_dyn(
            self.convert_features_to_nn_input(feat_gr, data)
        )[0]
        clutch_state_eq = jax.nn.sigmoid(clutch_state_eq)
        tau_gear = jnp.exp(self.tau_gear)
        clutch_state_dot = (clutch_state_eq - clutch_state) * tau_gear
        return jnp.array(
            [
                clutch_state_dot,
            ]
        ), {"clutch_eq": clutch_state_eq}

    def calculate_initial_guess_for_latent(self, state, control, times):
        """Calculate the initial guess for the latent space."""
        if self.is_initializing():
            return jnp.ones(self.num_latent)
        init_latent_state = jnp.array(
            [
                1.0,
            ]
        )
        dt = jnp.diff(times)

        def opt_step(latent_state, aux):
            _state, _control, _dt = aux
            _full_state = self.combine_state_and_latent(_state, latent_state)
            _data = self.get_data_from_full_state(_full_state, _control)
            # _data = self.get_data_from_state_and_control(_state, _control)
            # _data ={**_data, **{_n : _v for _n, _v in zip(self.name_latents, latent_state)}}
            _data = {**_data, "torque_shaft": self.calculate_shaft_torque(_data)}
            latent_dot, _ = self.calculate_latent_space_field(_data)
            next_latent_state = latent_state + latent_dot * _dt
            next_latent_state = self.constrain_latent(next_latent_state)
            return next_latent_state, None

        # Now let's integrate the latent space
        final_latent, _ = jax.lax.scan(
            opt_step, init_latent_state, (state[:-1], control, dt)
        )
        return final_latent

    def vector_field(self, state: jnp.ndarray, control: jnp.ndarray):
        """Compute the vector field."""
        if self.is_initializing():
            latent_init = self.calculate_initial_guess_for_latent(state, control, None)
            state = self.combine_state_and_latent(state, latent_init)
        # First, extract the latent state if it exists.
        # state, latent_state = self.split_state_latent_var(state)
        # data = self.get_data_from_state_and_control(state, control)
        # data = {**data, **{_n : _v for _n, _v in zip(self.name_latents, latent_state)}}
        data = self.get_data_from_full_state(state, control)
        # Start with the engine torque, boost pressure and rear wheel speed
        T_e = self.calculate_engine_torque(data)
        we_u, wr_u = data["engine_speed_unscaled"], data["rear_wheelspeed_unscaled"]
        clutch_state = data["clutch_state"]
        inv_gear_ratio = wr_u / we_u
        data = {**data, "engine_torque": T_e, "inv_gear_ratio": inv_gear_ratio}
        dot_pb = self.calculate_boost_pressure(data)
        wr_res = self.calculate_res_wheel_speed(data)
        # Shaft torque
        T_s = self.calculate_shaft_torque(data)
        data = {**data, "torque_shaft": T_s}
        # Wheel speed dynamics
        moment_coeffs = self.get_parameter_by_name("JeJr_1")
        gear_ratio = clutch_state / (inv_gear_ratio + 1.0e-4)
        dot_wr = gear_ratio * T_s * moment_coeffs + wr_res
        dot_we = T_e - T_s
        state_dot = jnp.array([dot_we, dot_pb, dot_wr])
        # Now calculate the latent space dynamics
        latent_dot, _extra_gear = self.calculate_latent_space_field(data)
        full_state_dot = self.combine_state_and_latent(state_dot, latent_dot)
        extra_return = {
            "engine_torque_est": T_e,
            "torque_shaft": T_s,
            "rear_wheel_speed_res": wr_res,
            "inv_gear_ratio": inv_gear_ratio,
            **_extra_gear,
        }
        return full_state_dot, extra_return


################################################################
####################### Utility Functions ######################
################################################################


def integrate_path(
    init_state: jnp.ndarray,
    controls: jnp.ndarray,
    times: jnp.ndarray,
    rng: jax.random.PRNGKey,
    params: Dict[str, Any],  # Parameters for the dynamics model and noise
    history_data: Any,
    dynamics: EngineModel,
    noise: NoiseTerm,
    return_full_state: bool = False,
    num_iter_between_dt: int = 1,
) -> Tuple[jnp.ndarray, Dict[str, Any]]:
    """
    Integrate the dynamics using the Euler–Maruyama method.

    The next state is assumed to be distributed as
      x_{k+1} ~ N(x_k + f(x_k, u_k) * dt,  (sqrt(dt)*noise)^2 )
    where 'f(x_k, u_k)' is given by the dynamics model and 'noise'
    is either provided by noise_fn or sampled from a standard normal distribution.

    Args:
        init_state: (n,) array representing the initial state.
        controls: (T, m) array of controls (one per time step).
        times: (T+1,) array of time points. (dt = times[i+1] - times[i])
        rng: a JAX random key.
        params: a dictionary of parameters for the dynamics model and noise.
        dynamics: an instance of the dynamics model.
        noise: an instance of the noise term.
        return_full_state: whether to return the full state or just the reduced state.
        num_iter_between_dt: number of iterations between each time step.

    Returns:
        path: (T+1, n) array of integrated states.
        extras: a dictionary of auxiliary outputs.
    """
    dt_array = jnp.diff(times)  # shape: (T,)
    dt_array = jnp.repeat(
        dt_array / num_iter_between_dt, num_iter_between_dt
    )  # shape: (T * num_iter_between_dt,)
    controls = jnp.repeat(
        controls, num_iter_between_dt, axis=0
    )  # shape: (T * num_iter_between_dt,)
    shape_z_norm = dynamics.num_states
    z_norm = jax.random.normal(rng, shape=(dt_array.shape[0], shape_z_norm))

    def step_fn(carry, inputs):
        state = carry
        control, dt, z = inputs  # dt is a scalar for this step
        # Compute the drift using the dynamics model.
        # drift, _extra = dynamics.apply(params["drift"], state, control)
        drift, _extra = dynamics.drift_wrapper(params["drift"], state, control)
        mean_update = state + drift * dt
        mean_update_reduced, mean_update_latent = dynamics.split_state_latent_var(
            mean_update
        )
        # Compute the noise term.
        reduced_state, _ = dynamics.split_state_latent_var(state)
        # noise_term, noise_extra = noise.apply(params["noise"], reduced_state, control)
        noise_term, noise_extra = noise.noise_wrapper(
            params["noise"], reduced_state, control
        )
        noise_sqrt_dt = noise_term * jnp.sqrt(dt)
        # Euler–Maruyama update: add noise scaled by sqrt(dt).
        next_state = mean_update_reduced + noise_sqrt_dt * z
        _extra = {**_extra, **noise_extra, "noise_sqrt": noise_sqrt_dt}
        # Let's clip the state
        next_state = dynamics.constrain_predictions(next_state)
        mean_update_latent = dynamics.constrain_latent(mean_update_latent)
        full_next_state = dynamics.combine_state_and_latent(
            next_state, mean_update_latent
        )
        if return_full_state:
            return full_next_state, (full_next_state, _extra)
        return full_next_state, (next_state, _extra)

    # Let's start with obtaining the latent state information. let's call apply with the name of function
    full_state = init_state
    if history_data is not None:
        # latent_initial_state = dynamics.apply(
        #     params["drift"], *history_data, method="calculate_initial_guess_for_latent")
        latent_initial_state = dynamics.initial_guess_wrapper(
            params["drift"], *history_data
        )
        full_state = dynamics.combine_state_and_latent(init_state, latent_initial_state)
    # Use scan to integrate the dynamics.
    _, (path, extras) = jax.lax.scan(step_fn, full_state, (controls, dt_array, z_norm))
    # Because we subdivide the time steps, we need to return only every num_iter_in steps
    extras = jax.tree_map(lambda x: x[::num_iter_between_dt], extras)
    if return_full_state:
        return (
            jnp.concatenate([full_state[None], path], axis=0)[::num_iter_between_dt],
            extras,
        )
    return (
        jnp.concatenate([init_state[None], path], axis=0)[::num_iter_between_dt],
        extras,
    )


def compute_nll_loss(
    params: Dict[str, Any],
    states: jnp.ndarray,  # shape (T+1, n)
    controls: jnp.ndarray,  # shape (T, m)
    times: jnp.ndarray,  # shape (T+1,)
    rng: jax.random.PRNGKey,
    history_data: Any,
    dynamics: EngineModel,
    noise: NoiseTerm,
    discounts: jnp.ndarray,
    num_iter_between_dt: int = 1,
    scaling_factor: jnp.ndarray = 1.0,
):
    # Check if rng is a single key
    if rng.ndim == 1:
        rng = rng[None]

    # Integrate the dynamics - > We just do one path for now
    path, extras = jax.vmap(
        integrate_path,
        in_axes=(None, None, None, 0, None, None, None, None, None, None),
    )(
        states[0],
        controls,
        times,
        rng,
        params,
        history_data,
        dynamics,
        noise,
        False,
        num_iter_between_dt,
    )
    # Extract noise term
    noise_term = extras["noise_sqrt"]
    if noise.ignore_noise:
        noise_term = jnp.ones_like(noise_term)
    # Compute the log-likelihood of the path.
    diff_state_no_scale = path[:, 1:, :] - states[1:, :][None]
    diff_state = (diff_state_no_scale / noise_term) * scaling_factor
    noise_loss = jnp.mean(
        jnp.sum(jnp.sum(jnp.log(noise_term), axis=-1) * discounts[None], axis=-1)
    )
    log_likelihood = jnp.mean(
        jnp.sum(jnp.sum(diff_state**2, axis=-1) * discounts[None], axis=-1)
    )
    mse = jnp.mean(
        jnp.mean((diff_state_no_scale**2) * discounts.reshape(1, -1, 1), axis=1), axis=0
    )
    ret_dict = {"nll_noise": noise_loss, "nll_mse": log_likelihood, "mse": jnp.sum(mse)}
    return ret_dict, (mse, path), extras


def single_sequence_training_loss(
    params: Dict[str, Any],
    state: jnp.ndarray,
    control: jnp.ndarray,
    rng_key: jnp.ndarray,
    extra_loss_args: Dict[str, Any],
    drift_term: EngineModel,
    noise_term: NoiseTerm,
):
    """
    Implement the training loss function for a single sequence.
    """
    assert (
        state.shape[0] >= drift_term.history_size + 1
    ), "The state should have at least the history size + 1"
    time_val = state[..., -1]
    state = state[..., :-1]
    # Lets split state and control to account for the history
    (hist_state, hist_control), (state, control) = drift_term.split_state_with_history(
        state, control
    )
    t_hist = time_val[: hist_state.shape[0]]
    hist = (hist_state, hist_control, t_hist)
    time_val = time_val[-state.shape[0] :]

    num_samples = extra_loss_args.get("num_samples", 1)
    # Create the discounts array
    if "discount" not in extra_loss_args:
        discounts = jnp.ones_like(control)
    else:
        discount = extra_loss_args["discount"]
        dt_scaling = extra_loss_args.get("discount_dt", 0.01)
        discounts = discount ** ((time_val - time_val[0])[:-1] / dt_scaling)

    if num_samples > 1:
        rng_key = jax.random.split(rng_key, num_samples)

    # Let's calculate the loss
    loss_dict, (mse_state, pred_state), extras = compute_nll_loss(
        params,
        state,
        control,
        time_val,
        rng_key,
        hist,
        drift_term,
        noise_term,
        discounts,
        extra_loss_args.get("num_iter_between_dt", 1),
        extra_loss_args.get("scaling_factor", 1),
    )
    state_loss = {k: _err for k, _err in zip(drift_term.name_states, mse_state)}
    noise_max = jnp.sum(extras["max_noise"][0, 0])
    # All loss
    losses = {**loss_dict, **state_loss, "loss_noise_max": noise_max}
    return losses, (state, pred_state, extras)


def default_extra_loss_fn(
    params: Dict[str, Any],
    gt_state: jnp.ndarray,
    pred_state: jnp.ndarray,
    extras: Dict[str, Any],
):
    """
    A template for the extra loss function beyond the NLL.

    Args:
        params: the parameters of the model. Can be used for regularization.
        pred_state: the predicted state. SIZE = [Batch, Particle, Time, State]
        extras: the extra information from the loss calculation.
            Dictionary, each key, val pair is a (Batch, Particle, Time, m) array, where m could be None.
            These are the extra information obtained during integration.
    Returns:
        losses: a dictionary of extra losses.
    """
    # By default, just apply regularization to parameters
    reg_loss = jnp.sum(
        jnp.array([jnp.sum(jnp.square(v)) for v in jax.tree_leaves(params)])
    )
    return {"loss_reg": reg_loss}


# Let's define the batch version of the training loss
def batch_training_loss(
    params: Dict[str, Any],
    state: jnp.ndarray,
    control: jnp.ndarray,
    rng_key: jnp.ndarray,
    extra_loss_args: Dict[str, Any],
    drift_term: EngineModel,
    noise_term: NoiseTerm,
    extra_loss_fn: Callable = default_extra_loss_fn,
):
    if len(rng_key.shape) == 1:
        rng_key = jax.random.split(rng_key, state.shape[0])

    loss_vals, (gt_state, _pred, _extra) = jax.vmap(
        single_sequence_training_loss, in_axes=(None, 0, 0, 0, None, None, None)
    )(params, state, control, rng_key, extra_loss_args["pred"], drift_term, noise_term)
    losses = {k: jnp.mean(v) for k, v in loss_vals.items()}
    # Regularization loss
    _extra_losses = extra_loss_fn(params, gt_state, _pred, _extra)
    losses = {**losses, **_extra_losses}
    total_loss = jnp.array(
        [losses[k] * reg_v for k, reg_v in extra_loss_args["reg_params"].items()]
    ).sum()
    return total_loss, {**losses, "total_loss": total_loss}


################################################################
####################### Load Model Function ####################
################################################################


def load_model_from_config(
    model_config: Dict[str, Any],
    data_statistics: Dict[str, Any] = {},
    seed: int = 0,
    class_drift: EngineModel = EngineModel,
    class_noise: NoiseTerm = NoiseTerm,
) -> Tuple[EngineModel, NoiseTerm, Dict[str, Any]]:
    """
    Load the model from the configuration dictionary.
    """
    # Setup the drift term
    drift_conf = model_config["drift"]
    drift_args = drift_conf.get("args", {})
    if "state_action_mean" not in drift_args:
        drift_args["state_action_mean"] = data_statistics["state_action_mean"]
        drift_args["state_action_std"] = data_statistics["state_action_std"]
    if "state_min" in data_statistics:
        drift_args["state_min"] = {
            **drift_args.get("state_min", {}),
            **data_statistics["state_min"],
        }
    if "state_max" in data_statistics:
        drift_args["state_max"] = {
            **drift_args.get("state_max", {}),
            **data_statistics["state_max"],
        }
    drift: EngineModel = class_drift(**drift_args)

    # Setup the noise term
    noise_conf = model_config["noise"]
    noise_args = noise_conf.get("args", {})
    noise_args["output_dimension"] = drift.num_states
    class_noise.get_normalized_state = drift.get_normalized_state
    noise: NoiseTerm = class_noise(**noise_args)

    # Set the random key
    rng_key = jax.random.PRNGKey(seed)
    rng_key, rng_drift, rng_noise = jax.random.split(rng_key, 3)

    dummy_state = jnp.zeros((drift.num_states,))
    dummy_control = jnp.zeros((drift.num_controls,))
    drift_params = drift.init(rng_drift, dummy_state, dummy_control)
    dummy_noise_input = jnp.zeros((drift.num_states,))
    noise_params = noise.init(rng_noise, dummy_noise_input, dummy_control)

    # Merge the parameters into a single dictionary.
    params = {"drift": drift_params, "noise": noise_params}
    return drift, noise, params


################################################################
####################### Gym Environment ########################
################################################################


class ModelWithTargetWheelSpeedAndRate:
    """A model based on engine model with desired wheel speed imposed by
    enforcing the state, and control rate envoling in the latent space.
    """

    def __init__(self, model: EngineModel):
        self.model = model

    @property
    def name_controls(self) -> Tuple[str]:
        return [f"{name}_dot" for name in self.model.name_controls] + [
            "forced_road_torque",
        ]

    @property
    def num_controls(self) -> int:
        return len(self.name_controls)

    @property
    def name_latents(self) -> Tuple[str]:
        return self.model.name_latents + self.model.name_controls

    @property
    def num_latent(self) -> int:
        return len(self.name_latents)

    @property
    def actual_num_controls(self) -> int:
        return self.model.num_controls

    @property
    def bounds_latents(self) -> Tuple[Dict[str, float]]:
        min_dict, max_dict = self.model.bounds_latents
        min_dict = {
            **min_dict,
            **{name: self.model.state_min[name] for name in self.model.name_controls},
        }
        max_dict = {
            **max_dict,
            **{name: self.model.state_max[name] for name in self.model.name_controls},
        }
        return min_dict, max_dict

    def get_data_from_full_state(self, full_state):
        """Get the data from the full state."""
        # Let's split the state into state and latent
        base_state = full_state[: -self.model.num_controls]
        base_control = full_state[-self.model.num_controls :]
        data = self.model.get_data_from_full_state(base_state, base_control)
        return data

    def constrain_predictions(self, state):
        return self.model.constrain_predictions(state)

    def split_state_latent_var(self, state):
        """Split the state into state and latent variables."""
        # Let's split the state into state and latent
        base_state = state[: -self.model.num_controls]
        extra_latent = state[-self.model.num_controls :]
        gt_state, gt_latent = self.model.split_state_latent_var(base_state)
        return gt_state, jnp.concatenate([gt_latent, extra_latent])

    def combine_state_and_latent(self, state, latent):
        """Combine the state and latent variables into a single state."""
        # Let's combine the state and latent
        extra_latent = latent[-self.model.num_controls :]
        base_latent = latent[: -self.model.num_controls]
        init_combined = self.model.combine_state_and_latent(state, base_latent)
        return jnp.concatenate([init_combined, extra_latent])

    def constrain_latent(self, latent_state):
        extra_latent = latent_state[-self.model.num_controls :]
        base_latent = latent_state[: -self.model.num_controls]
        base_latent = self.model.constrain_latent(base_latent)
        # Now let's enforce the extra latent by using state_min and max
        extra_latent = jnp.array(
            [
                jnp.clip(val, self.model.state_min[name], self.model.state_max[name])
                for name, val in zip(self.model.name_controls, extra_latent)
            ]
        )
        return jnp.concatenate([base_latent, extra_latent])

    def drift_wrapper(self, params, state, control):
        # Let's decompose the input to be suitable for the EngineModel call
        base_control_rate = control[:-1]
        forced_road_torque = control[-1]
        # Let's enforce the current wheelspeed in state to be the desired one
        state = jnp.array(state)
        # We are not initializing, and the state contains state + latent
        base_control = state[-self.model.num_controls :]
        base_state = state[: -self.model.num_controls]
        # Let's call the base vector field
        # base_state_dot, extra_base = super().vector_field(base_state, base_control)
        base_state_dot, extra_base = self.model.drift_wrapper(
            params, base_state, base_control
        )
        # Let's add the forced road torque to the wheel speed dynamics
        idx_wheel_speed = self.model.full_state_indexes["rear_wheelspeed"]
        base_state_dot = base_state_dot.at[idx_wheel_speed].add(forced_road_torque)
        full_state_dot = jnp.concatenate([base_state_dot, base_control_rate])
        extra_state_dot = {
            f"{_name}_dot": _val
            for _name, _val in zip(
                self.model.name_states + self.name_latents, full_state_dot
            )
        }
        return full_state_dot, {**extra_base, **extra_state_dot}

    def initial_guess_wrapper(self, params, state, control, times):
        """Calculate the initial guess for the latent space.
        state and control are given in the initial space of EngineModel.
        Control being state and (throttle,)
        """
        latent_init = self.model.initial_guess_wrapper(params, state, control, times)
        assert state.ndim == 2, "State should be 2D"
        last_control = control[-1, : self.model.num_controls]
        return jnp.concatenate([latent_init, last_control])

    # def __getattr__(self, name: str) -> Any:
    #     """
    #     Returns the attribute of the class.
    #     This is mainly a wrapper around the drift term of the SDE.

    #     Args:
    #         name: the name of the attribute.
    #             str

    #     Returns:
    #         attribute: the attribute of the class.
    #             Any
    #     """
    #     if name != "apply":
    #         return getattr(self._drift_term, name)
    #     assert False, "The apply method is not implemented in this class."

    #     # TODO: Incorporate the learned parameters and the apply method if needed
    #     def apply_fn(learnable_parameters: Dict[str, Any], *args, **kwargs):
    #         drift_params = learnable_parameters['drift'] \
    #             if "drift" in learnable_parameters else learnable_parameters
    #         return self.model.apply(
    #             drift_params,
    #             *args,
    #             **kwargs
    #         )
    #     return apply_fn


@struct.dataclass
class EnvState:
    """
    Represent the environment state.
    """

    x: jnp.ndarray
    num_iter: int
    # des_wheel_torque: float
    des_road_torque: float
    des_wheel_speed: float
    # num_iter_setpoint: int


@struct.dataclass
class EnvParams:
    max_steps_in_episode: int = 1000


class EngineModelEnv(environment.Environment):
    """A gym environment for the engine model."""

    def __init__(
        self,
        drift: ModelWithTargetWheelSpeedAndRate,
        noise: NoiseTerm,
        model_params: Dict[str, Any],
        env_config: Dict[str, Any],
    ):
        self.drift = drift
        self.noise = noise
        self.model_params = model_params
        self.env_config = env_config
        self.setup_observation_space()

    def step_env(
        self,
        key: jnp.ndarray,
        state: EnvState,
        action: jnp.ndarray,
        params: EnvParams,
    ) -> Tuple[jnp.ndarray, EnvState, float, bool, dict]:
        """Environment-specific step transition."""
        num_iter = state.num_iter
        # Prediction for the next state
        key, key_pred = jax.random.split(key)
        action_unscaled = self.denormalize_control(action)
        full_control = jnp.concatenate(
            [action_unscaled, jnp.array([state.des_road_torque])]
        )
        state_evol, extra_state_info = self.predict_trajectories(
            state.x, full_control, key_pred
        )
        # Extract the next state
        next_state = state_evol[-1]
        extra_state_info = jax.tree_map(lambda x: x[0], extra_state_info)
        target_config = self.extract_target_from_env_state(state)
        # Update the environment state
        new_state = self.create_env_state(next_state, num_iter + 1, target_config)
        # Get the observation
        obs, extra_obs_info = self.get_obs_and_more(new_state, extra_state_info)
        combined_infos = {**extra_state_info, **extra_obs_info}
        # Reward computation
        reward, rew_dict = self.compute_reward(combined_infos, new_state, state, action)
        # Termination and done operation
        is_path_done = new_state.num_iter >= params.max_steps_in_episode
        done_state_out_of_bounds = jnp.logical_or(
            jnp.any(new_state.x < self._state_min),
            jnp.any(new_state.x > self._state_max),
        )
        done_state_out_of_bounds = jnp.any(done_state_out_of_bounds)
        done = jnp.any(jnp.array([is_path_done, done_state_out_of_bounds]))
        truncated = jnp.array(False)
        return (
            obs,
            new_state,
            reward,
            done,
            {
                "do_terminate": done,
                "actual_done": done,
                "truncation_info": {
                    "truncated": truncated,
                    "final_observation": obs,
                },
                "rewards": rew_dict,
                "extra_info": combined_infos,
                "termination": {
                    "done": done,
                    "truncated": truncated,
                    "done_state_out_of_bounds": done_state_out_of_bounds,
                    "is_path_done": is_path_done,
                },
            },
        )

    def reset_env(self, key: jnp.ndarray, params: EnvParams):
        """Perform the reset of the environment."""
        key, key_temp = jax.random.split(key)
        # Get the set where to sample initial conditions
        init_cond_states = self.env_config["init_cond_states"]
        init_cond_controls = self.env_config["init_cond_controls"]
        init_times = self.env_config["init_times"]
        # Sample an index from the history dataset
        idx = jax.random.randint(key_temp, (), 0, init_cond_states.shape[0])
        latent_state = self.drift.initial_guess_wrapper(
            self.model_params["drift"],
            init_cond_states[idx],
            init_cond_controls[idx],
            init_times[idx],
        )
        # Combine the state and latent state
        full_state = self.drift.combine_state_and_latent(
            init_cond_states[idx][-1], latent_state
        )
        # Let's obtain the road torque to use for simulation
        key, key_temp = jax.random.split(key)
        target_config = self.setup_target_config(key_temp, full_state)
        # Let's create the env state
        env_state = self.create_env_state(full_state, 0, target_config)
        # Let's create the observation
        obs, _ = self.get_obs_and_more(env_state)
        return obs, env_state

    def predict_trajectories(
        self,
        state: jnp.ndarray,
        control: jnp.ndarray,
        key: jnp.ndarray,
    ) -> Tuple[jnp.ndarray, Dict[str, jnp.ndarray]]:
        """
        Predict the trajectories of the vehicle given the
        state and control.
        """
        sampling_time = self.env_config["sampling_time"]
        num_iter_between_dt = self.env_config["num_iter_between_dt"]
        _times = jnp.array([0, sampling_time])
        _control = control[None]
        state_evol, extra = integrate_path(
            state,
            _control,
            _times,
            key,
            self.model_params,
            None,
            self.drift,
            self.noise,
            True,
            num_iter_between_dt,
        )
        return state_evol, extra

    def setup_target_config(
        self, key: jnp.ndarray, full_state: jnp.ndarray
    ) -> Dict[str, float]:
        """Setup the target configuration."""
        ang_rate_max = self.env_config["ang_rate_max"]
        ang_rate_min = self.env_config["ang_rate_min"]
        key, ang_key, ang_fric, ang_end_t = jax.random.split(key, 4)
        total_ang_rate = jax.random.uniform(
            ang_key, (), minval=ang_rate_min, maxval=ang_rate_max
        )
        fric_ang_rate = jax.random.uniform(
            ang_fric,
            (),
            minval=jnp.maximum(ang_rate_min, total_ang_rate - ang_rate_max),
            maxval=jnp.minimum(ang_rate_max, total_ang_rate - ang_rate_min),
        )
        eng_torque_eq = total_ang_rate - fric_ang_rate
        Tsim = (
            self.env_config["sampling_time"] * self.env_config["max_steps_in_episode"]
        )
        Tset = Tsim * jax.random.uniform(ang_end_t, (), 0.6, 0.8)
        idx_rear_wheel = self.drift.model.full_state_indexes["rear_wheelspeed"]
        curr_rear_wheel = full_state[idx_rear_wheel]
        target_rear_wheel = curr_rear_wheel + total_ang_rate * Tset
        res_dict = {
            "des_road_torque": eng_torque_eq,
            "des_wheel_speed": target_rear_wheel,
            "Tset": Tset,
            "eng_torque_eq": eng_torque_eq,
        }
        return res_dict

    def get_logging_info(self, info: Dict[str, Any]) -> Dict[str, Any]:
        """
        Get the logging information.
        """
        return info["rewards"]

    def compute_reward(
        self,
        info: Dict[str, Any],
        new_state: EnvState,
        old_state: EnvState,
        action: jnp.ndarray,
    ) -> Tuple[float, Dict[str, float]]:
        """Compute the reward."""
        reward = 0.0
        return reward, {}

    def denormalize_control(self, action: jnp.ndarray) -> jnp.ndarray:
        """Denormalize the control."""
        return action

    def normalize_control(self, action: jnp.ndarray) -> jnp.ndarray:
        """Normalize the control."""
        return action

    def extract_target_from_env_state(self, env_state: EnvState):
        """Extract the target from the environment state."""
        target_dict = {
            "des_road_torque": env_state.des_road_torque,
            "des_wheel_speed": env_state.des_wheel_speed,
        }
        return target_dict

    def create_env_state(
        self, state: jnp.ndarray, num_iter: int, target: Dict[str, float]
    ) -> EnvState:
        """Create the environment state."""
        des_road_torque = target["des_road_torque"]
        des_wheel_speed = target["des_wheel_speed"]
        env_state = EnvState(
            x=state,
            num_iter=num_iter,
            des_road_torque=des_road_torque,
            des_wheel_speed=des_wheel_speed,
        )
        return env_state

    def get_obs_and_more(self, env_state: EnvState, extra_values: Dict[str, Any] = {}):
        """Get the observation and more information."""
        state = env_state.x
        des_wheel_speed = env_state.des_wheel_speed
        des_road_torque = env_state.des_road_torque
        data = self.drift.get_data_from_full_state(state)
        rear_wr = data["rear_wheelspeed_unscaled"]
        diff_wr = des_wheel_speed - rear_wr
        data = {
            **data,
            "des_wheel_speed": des_wheel_speed,
            "des_road_torque": des_road_torque,
            "diff_wr": diff_wr,
        }
        obs = jnp.array([data[name] for name in self._obs_names])
        return obs, data

    def get_info_from_obs(self, obs):
        """
        Extract info from observation function.
        """
        return {name: v for name, v in zip(self._obs_names, obs)}

    def setup_observation_space(self):
        """Setup the observation space."""
        # Setup lower and upper bounds of the state
        MAX_VAL = 1.0e6
        state_min = [self.drift.state_min[name] for name in self.drift.name_states]
        state_max = [self.drift.state_max[name] for name in self.drift.name_states]
        state_min += [
            self.drift.bounds_latents[0][name] for name in self.drift.name_latents
        ]
        state_max += [
            self.drift.bounds_latents[1][name] for name in self.drift.name_latents
        ]
        self._state_min = jnp.array(state_min)
        self._state_max = jnp.array(state_max)
        # Setup the observation space
        full_state_name = self.drift.name_states + self.drift.name_latents
        self._full_state_name = full_state_name
        self._obs_names = self.env_config["obs_names"]
        self._obs_min = jnp.array(
            [self._state_min.get(name, -MAX_VAL) for name in self._obs_names]
        )
        self._obs_max = jnp.array(
            [self._state_max.get(name, MAX_VAL) for name in self._obs_names]
        )
        # Setup the action space
        self._action_names = self.drift.name_controls[: self.drift.model.num_controls]
        action_bounds = self.env_config["action_bounds"]
        self._action_min = jnp.array(
            [action_bounds[name][0] for name in self._action_names]
        )
        self._action_max = jnp.array(
            [action_bounds[name][1] for name in self._action_names]
        )

    @property
    def name(self) -> str:
        """Environment name."""
        return "SDE-Engine-v1"

    @property
    def num_actions(self) -> int:
        """Number of actions possible in environment."""
        return self.drift.model.num_controls

    def action_space(self, params=None):
        """Action space of the environment."""
        return spaces.Box(
            self._action_min, self._action_max, self._action_min.shape, jnp.float32
        )

    def state_space(self, params: EnvParams) -> spaces.Dict:
        """State space of the environment."""
        MAX_VAL = 1.0e6
        return spaces.Dict(
            {
                "x": spaces.Box(
                    self._state_min, self._state_max, self._state_min.shape, jnp.float32
                ),
                "num_iter": spaces.Discrete(params.max_steps_in_episode),
                # "des_wheel_torque": spaces.Box(-MAX_VAL, MAX_VAL, (), jnp.float32),
                "des_road_torque": spaces.Box(-MAX_VAL, MAX_VAL, (), jnp.float32),
                "des_wheel_speed": spaces.Box(
                    self.drift.model.state_min["rear_wheelspeed"],
                    self.drift.model.state_max["rear_wheelspeed"],
                    (),
                    jnp.float32,
                ),
                # "num_iter_setpoint": spaces.Discrete(params.max_steps_in_episode)
            }
        )

    def observation_space(self, params: EnvParams) -> spaces.Box:
        """Observation space of the environment."""
        return spaces.Box(
            self._obs_min, self._obs_max, self._obs_min.shape, jnp.float32
        )

    @property
    def default_params(self) -> EnvParams:
        """Default environment parameters."""
        return EnvParams(max_steps_in_episode=self.env_config["max_steps_in_episode"])
