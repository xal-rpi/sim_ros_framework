"""Lightweight CasADi bicycle model with Fiala tire forces only
for the SBR track veicle --> see techground_sbr.yaml
"""

from __future__ import annotations

from typing import Any, Dict

import casadi as ca


fiala_params = {
    "vehicle": {
        "mass": 1411.235,
        "gravity": 9.81,
        "wheelbase": 2.5,
        "track_width": 1.787,
        "cogToFrontAxle": 1.524,
        "cogToRearAxle": 0.976,
        "cogHeight": 0.6019505943059921,
        "inertia_zz": 2591.114,
        "center_of_mass_offset": {"x": 0.0, "y": 0.0},
        "roadwheel_angle_transform": {"scale": 1.0, "offset": 0.0},
        "wheel_speed_scale": {"front": 1.0, "rear": 1.0},
        "lateral_load_transfer_distribution": {"front": 1.0, "rear": 1.0},
        "Mzx_bias": 0.0,
        "aero_drag_coeff": [1.5343084335327148, 6.762434005737305],
    },
    "tires": {
        "fiala_pure_front": True,
        "mu_front": 0.9548320770263672,
        "mu_rear": 1.263258695602417,
        "Cf_front": 92336.30299568176,
        "Cf_rear": 138502.63357162476,
    },
    "wheel_dynamics": {
        "front_torque_enabled": False,
        "wheel_radius": 0.32,
        "front_wheel_inertia": 4.907045731327228,
        "rear_wheel_inertia": 4.907045731327228,
        "brake_rel_front": 6.6772685050964355,
        "brake_rel_rear": 8.44875717163086
    },
}

class FialaBicycleCasADi:
    """Simple bicycle model for control.
    """

    SHORT_TO_VERBOSE = {
        "r": "yaw_rate",
        "vx": "vel_x",
        "vy": "vel_y",
        "wr": "rear_wheelspeed_ms",
        "delta": "roadwheel_angle",
    }

    def __init__(self, params: Dict[str, Any]):
        self.params = params
        self.vehicle = params["vehicle"]
        self.tires = params["tires"]
        self.wheel = params["wheel_dynamics"]

    @staticmethod
    def _safe_positive(value, eps: float = 1e-4):
        return ca.fmax(value, eps)

    @staticmethod
    def _clip(value, low: float, high: float):
        return ca.fmin(ca.fmax(value, low), high)

    def _normalize_input(self, state_control: Dict[str, Any]) -> Dict[str, Any]:
        data = dict(state_control)
        for short_name, verbose_name in self.SHORT_TO_VERBOSE.items():
            if short_name in data and verbose_name not in data:
                data[verbose_name] = data[short_name]
            if verbose_name in data and short_name not in data:
                data[short_name] = data[verbose_name]

        has_vx_vy = "vel_x" in data and "vel_y" in data
        has_v_beta = "V" in data and "beta" in data
        if has_v_beta and not has_vx_vy:
            data["vel_x"] = data["V"] * ca.cos(data["beta"])
            data["vel_y"] = data["V"] * ca.sin(data["beta"])
            data["vx"] = data["vel_x"]
            data["vy"] = data["vel_y"]
        elif has_vx_vy and not has_v_beta:
            data["V"] = ca.sqrt(data["vel_x"] ** 2 + data["vel_y"] ** 2 + 1e-5)
            data["beta"] = ca.atan2(data["vel_y"], data["vel_x"])

        data.setdefault("brake", 0.0)
        return data

    def _powertrain_scaled_inputs(
        self,
        engine_speed_rads,
        boost_pressure,
        throttle,
        rear_wheelspeed_ms,
    ) -> Dict[str, Any]:
        wheel_radius = float(self.wheel["wheel_radius"])
        axle_speed_rads = rear_wheelspeed_ms / wheel_radius
        return {
            "engine_speed_rads": engine_speed_rads / 300.0,
            "boost_pressure": boost_pressure / 10.0,
            "throttle": throttle,
            "rear_wheelspeed_ms": rear_wheelspeed_ms / 5.0,
            "i_gr": 100.0 * (axle_speed_rads / (engine_speed_rads + 1e-4)),
            "gr": 0.01 * (engine_speed_rads / (axle_speed_rads + 1e-4)),
        }

    def _compute_wheel_torque_terms(self, state_control: Dict[str, Any]) -> Dict[str, Any]:
        """Locally embedded rear-wheel torque surrogate from the identified model."""
        engine_speed_rads = state_control["engine_speed_rads"]
        boost_pressure = state_control["boost_pressure"]
        throttle = state_control["throttle"]
        rear_wheelspeed_ms = state_control["rear_wheelspeed_ms"]

        data_scaled = self._powertrain_scaled_inputs(
            engine_speed_rads,
            boost_pressure,
            throttle,
            rear_wheelspeed_ms,
        )

        eng_scaled = data_scaled["engine_speed_rads"]
        boost_scaled = data_scaled["boost_pressure"]
        gr_scaled = data_scaled["gr"]

        hidden_1 = ca.tanh(
            ((((-0.299247 * eng_scaled) + (-0.0105762 * boost_scaled)) + (2.09084 * throttle)) + (-1.21916 * gr_scaled))
            + 0.314798
        )
        hidden_2 = ca.tanh(
            ((((-0.498726 * eng_scaled) + (0.0547674 * boost_scaled)) + (-0.715504 * throttle)) + (0.819941 * gr_scaled))
            + 0.222718
        )
        hidden_3 = ca.tanh(
            ((((-0.17094 * eng_scaled) + (-0.021485 * boost_scaled)) + (0.0716762 * throttle)) + (3.68147 * gr_scaled))
            + 0.0686006
        )
        hidden_4 = ca.tanh(
            (((((0.745817 * eng_scaled) + (1.42766 * boost_scaled)) + (-1.60501 * throttle)) + (0.00198479 * gr_scaled)))
            + 0.389467
        )
        hidden_5 = ca.tanh(
            ((((-0.212549 * eng_scaled) + (-1.00774 * boost_scaled)) + (1.36391 * throttle)) + (0.0194709 * gr_scaled))
            - 0.128168
        )
        hidden_6 = ca.tanh(
            (((((0.0431327 * eng_scaled) + (0.806935 * boost_scaled)) + (-0.641353 * throttle)) + (0.350162 * gr_scaled)))
            - 0.0780202
        )

        total_torque = 1000.0 * (
            (1.48922 * ca.tanh(((((((-1.94056 * hidden_1) + (-0.0610694 * hidden_2)) + (0.819076 * hidden_3)) + (1.45404 * hidden_4)) + (-0.054226 * hidden_5)) + (0.444599 * hidden_6)) + 0.28967))
            + (-1.88444 * ca.tanh(((((((-0.9117 * hidden_1) + (-1.14981 * hidden_2)) + (2.47463 * hidden_3)) + (-1.55474 * hidden_4)) + (-0.0627599 * hidden_5)) + (-0.375705 * hidden_6)) + 0.0779681))
            + (-0.461754 * ca.tanh((((((((-0.0409663 * hidden_1) + (0.145847 * hidden_2)) + (0.0101948 * hidden_3)) + (-0.00553184 * hidden_4)) + (0.108535 * hidden_5)) + (0.110177 * hidden_6)) + 0.0291548)))
            + (0.213776 * ca.tanh((((((((-0.149709 * hidden_1) + (-0.423617 * hidden_2)) + (0.0647505 * hidden_3)) + (-0.224323 * hidden_4)) + (0.254653 * hidden_5)) + (-0.00174019 * hidden_6)) - 0.0915727)))
            + (3.17494 * ca.tanh((((((((0.31846 * hidden_1) + (-2.02533 * hidden_2)) + (2.54764 * hidden_3)) + (-2.3658 * hidden_4)) + (-0.378261 * hidden_5)) + (0.280427 * hidden_6)) - 0.173948)))
            + (0.888703 * ca.tanh((((((((1.31051 * hidden_1) + (-2.8665 * hidden_2)) + (-0.153358 * hidden_3)) + (-1.22142 * hidden_4)) + (1.16874 * hidden_5)) + (-0.546866 * hidden_6)) + 0.36393)))
            + 0.0936948
        )

        return {
            "data_scaled": data_scaled,
            "total_torque": total_torque,
            "rear_wheel_torque": total_torque,
            "front_wheel_torque": 0.0,
            "torque_dist_r": 1.0,
        }

    def estimate_inverse_throttle(self, state_control: Dict[str, Any], desired_torque):
        """Locally embedded inverse-throttle surrogate from the identified model."""
        engine_speed_rads = state_control["engine_speed_rads"]
        boost_pressure = state_control["boost_pressure"]
        rear_wheelspeed_ms = state_control["rear_wheelspeed_ms"]

        wheel_radius = float(self.wheel["wheel_radius"])
        engine_speed_scaled = engine_speed_rads / 300.0
        boost_scaled = boost_pressure / 10.0
        desired_torque_scaled = desired_torque / 1000.0
        gear_ratio_scaled = 0.01 * (engine_speed_rads / ((rear_wheelspeed_ms / wheel_radius) + 1e-4))
        zero = 0.0

        hidden_1 = ca.fmax((((((0.301603 * engine_speed_scaled) + (-0.170997 * boost_scaled)) + (-0.157463 * desired_torque_scaled)) + (-4.33797 * gear_ratio_scaled)) + 0.143365), zero)
        hidden_2 = ca.fmax((((((-0.0836236 * engine_speed_scaled) + (0.129827 * boost_scaled)) + (-0.141142 * desired_torque_scaled)) + (6.88794 * gear_ratio_scaled)) - 0.0446952), zero)
        hidden_3 = ca.fmax((((((-0.0642931 * engine_speed_scaled) + (0.241317 * boost_scaled)) + (0.348286 * desired_torque_scaled)) + (-0.241327 * gear_ratio_scaled)) + 0.396857), zero)
        hidden_4 = ca.fmax((((((0.166931 * engine_speed_scaled) + (-0.0917765 * boost_scaled)) + (0.618937 * desired_torque_scaled)) + (-1.26335 * gear_ratio_scaled)) + 0.046271), zero)

        return (
            (0.492427 * ca.fmax((((((-0.567724 * hidden_1) + (-1.39205 * hidden_2)) + (0.410955 * hidden_3)) + (-0.241824 * hidden_4)) + 0.272251), zero))
            + (0.841248 * ca.fmax((((((-0.338327 * hidden_1) + (-1.77441 * hidden_2)) + (-0.427265 * hidden_3)) + (0.553774 * hidden_4)) - 0.0660385), zero))
            + (0.202179 * ca.fmax((((((-0.815237 * hidden_1) + (-0.315648 * hidden_2)) + (0.398362 * hidden_3)) + (0.47149 * hidden_4)) + 0.231422), zero))
            + (0.120461 * ca.fmax((((((-0.324339 * hidden_1) + (-0.34381 * hidden_2)) + (-0.129538 * hidden_3)) + (0.487535 * hidden_4)) + 0.0541269), zero))
            - 0.000217977
        )

    def build_powertrain_functions(self):
        """Build reusable CasADi functions for wheel torque estimation and inverse throttle."""
        torque_est_in = ca.SX.sym("eng", 5)
        trq_we = torque_est_in[0]
        trq_pb = torque_est_in[1]
        trq_thr = torque_est_in[2]
        trq_wr = torque_est_in[3]
        trq_des = torque_est_in[4]

        torque_terms = self._compute_wheel_torque_terms(
            {
                "engine_speed_rads": trq_we,
                "boost_pressure": trq_pb,
                "throttle": trq_thr,
                "rear_wheelspeed_ms": trq_wr,
            }
        )
        inv_throttle = self.estimate_inverse_throttle(
            {
                "engine_speed_rads": trq_we,
                "boost_pressure": trq_pb,
                "rear_wheelspeed_ms": trq_wr,
            },
            trq_des,
        )

        return {
            "torque_terms": torque_terms,
            "inv_throttle_expr": inv_throttle,
            "torque_est_fn": ca.Function(
                "torque_est",
                [trq_we, trq_pb, trq_thr, trq_wr],
                [torque_terms["total_torque"]],
            ),
            "inv_throttle_fn": ca.Function(
                "inv_torque",
                [trq_we, trq_pb, trq_wr, trq_des],
                [inv_throttle],
            ),
        }

    def fiala_pure_lateral(self, alpha, c_alpha, mu, fz):
        f_max = mu * fz
        tan_a = ca.tan(alpha)
        abs_tan_a = ca.fabs(tan_a)
        slip_limit = 3.0 * f_max / c_alpha
        fy = (
            -c_alpha * tan_a
            + (c_alpha ** 2 / (3.0 * f_max)) * abs_tan_a * tan_a
            - (c_alpha ** 3 / (27.0 * f_max ** 2)) * tan_a ** 3
        )
        return ca.if_else(abs_tan_a < slip_limit, fy, -f_max * ca.sign(alpha))

    def fiala_combined(self, alpha, kappa, c_alpha, mu, fz):
        tan_a = ca.tan(alpha)
        sigma_total = ca.sqrt(tan_a ** 2 + kappa ** 2 + 1e-6)
        f_max = mu * fz
        slip_limit = 3.0 * f_max / c_alpha
        f_total = (
            c_alpha * sigma_total
            - (c_alpha ** 2 / (3.0 * f_max)) * sigma_total ** 2
            + (c_alpha ** 3 / (27.0 * f_max ** 2)) * sigma_total ** 3
        )
        f_total = ca.if_else(sigma_total < slip_limit, f_total, f_max)
        fy = -f_total * (tan_a / sigma_total)
        fx = f_total * (kappa / sigma_total)
        return fx, fy

    def get_tire_forces(
        self, alpha_f, alpha_r, kappa_x_front, kappa_x_rear, fzf, fzr):
        if self.tires.get("fiala_pure_front", True):
            fy_f = self.fiala_pure_lateral(
                alpha_f,
                self.tires["Cf_front"],
                self.tires["mu_front"],
                fzf,
            )
            fx_f = 0.0
        else:
            fx_f, fy_f = self.fiala_combined(
                alpha_f,
                kappa_x_front,
                self.tires["Cf_front"],
                self.tires["mu_front"],
                fzf,
            )

        fx_r, fy_r = self.fiala_combined(
            alpha_r,
            kappa_x_rear,
            self.tires["Cf_rear"],
            self.tires["mu_rear"],
            fzr,
        )
        return fx_f, fy_f, fx_r, fy_r

    def get_vectorfield(self, state_control: Dict[str, Any]) -> Dict[str, Any]:
        """Return the vector field and diagnostic quantities for the Fiala bicycle model."""
        data = self._normalize_input(state_control)

        mass = self.vehicle["mass"]
        gravity = self.vehicle["gravity"]
        a = self.vehicle["cogToFrontAxle"]
        b = self.vehicle["cogToRearAxle"]
        length = self.vehicle["wheelbase"]
        hcog = self.vehicle["cogHeight"]
        inv_mass = 1.0 / mass
        inv_inertia = 1.0 / self.vehicle["inertia_zz"]
        aero_drag_coeff = self.vehicle["aero_drag_coeff"]

        r = data["yaw_rate"]
        vx = data["vel_x"]
        vy = data["vel_y"]
        delta = data["roadwheel_angle"]
        rear_wheel_torque = data["rear_wheel_torque"]
        brake = data["brake"]
        delta_fx_hat = data.get("deltaFx_hat", data.get("accel_x", 0.0))

        cos_delta = ca.cos(delta)
        sin_delta = ca.sin(delta)
        safe_vx = ca.fmax(vx, 0.1)

        delta_fz = delta_fx_hat * (mass * hcog / length)
        fzf = ((b * mass * gravity) / length) - delta_fz
        fzr = ((a * mass * gravity) / length) + delta_fz
        fzf = ca.fmax(fzf, 1.0)
        fzr = ca.fmax(fzr, 1.0)

        vx_front_wheel = vx * cos_delta + (vy + a * r) * sin_delta
        vy_front_wheel = -vx * sin_delta + (vy + a * r) * cos_delta
        vx_rear_wheel = vx
        vy_rear_wheel = vy - b * r

        wf  = data.get("front_wheelspeed_ms", vx_front_wheel)
        wr = data["rear_wheelspeed_ms"]

        vx_front_safe = ca.fmax(vx_front_wheel, 0.1)
        vx_rear_safe = ca.fmax(vx_rear_wheel, 0.1)
        alpha_f = ca.atan2(vy + a * r, safe_vx) - delta
        alpha_r = ca.atan2(vy - b * r, safe_vx)
        tan_alpha_f = vy_front_wheel / vx_front_safe
        tan_alpha_r = vy_rear_wheel / vx_rear_safe
        kappa_x_front = (wf - vx_front_wheel) / vx_front_safe
        kappa_x_rear = (wr - vx_rear_wheel) / vx_rear_safe

        fx_f, fy_f, fx_r, fy_r = self.get_tire_forces(
            alpha_f,
            alpha_r,
            kappa_x_front,
            kappa_x_rear,
            fzf,
            fzr,
        )

        total_vel = ca.sqrt(vx ** 2 + vy ** 2 + 1e-5)
        faero_x = -aero_drag_coeff[0] * total_vel * vx
        faero_y = -aero_drag_coeff[1] * total_vel * vy

        r_dot = (a * fy_f * cos_delta + a * fx_f * sin_delta - b * fy_r) * inv_inertia
        fx_total = fx_f * cos_delta - fy_f * sin_delta + fx_r + faero_x
        fy_total = fx_f * sin_delta + fy_f * cos_delta + fy_r + faero_y
        vx_dot = fx_total * inv_mass + r * vy
        vy_dot = fy_total * inv_mass - r * vx

        # Body-frame accelerations used for GG analysis.
        # Kept separate from state derivatives to make post-processing explicit.
        accel_x = vx_dot - r * vy
        accel_y = vy_dot + r * vx

        # front_wheel_torque = 0.0
        # wf_dot = (
        #     (self.wheel["wheel_radius"] / self.wheel["front_wheel_inertia"]) * front_wheel_torque
        #     - (self.wheel["wheel_radius"] ** 2 / self.wheel["front_wheel_inertia"])
        #     * (fx_f + brake * self.wheel["brake_rel_front"] * mass)
        # )
        wr_dot = (
            (self.wheel["wheel_radius"] / self.wheel["rear_wheel_inertia"]) * rear_wheel_torque
            - (self.wheel["wheel_radius"] ** 2 / self.wheel["rear_wheel_inertia"])
            * (fx_r + brake * self.wheel["brake_rel_rear"] * mass)
        )

        speed = self._safe_positive(data["V"], 0.1)
        v_dot = vx_dot * ca.cos(data["beta"]) + vy_dot * ca.sin(data["beta"])
        beta_dot = (vy_dot * ca.cos(data["beta"]) - vx_dot * ca.sin(data["beta"])) / speed

        tire_energy = (
            ca.fabs(fy_r * vy_rear_wheel)
            + ca.fabs(fx_r * (wr - vx_rear_wheel))
        ) / mass
        tire_saturation = kappa_x_rear ** 2 + tan_alpha_r ** 2
        total_rear_force = fx_r ** 2 + fy_r ** 2
        total_front_force = fx_f ** 2 + fy_f ** 2

        return {
            "Fxf": fx_f,
            "Fyf": fy_f,
            "Fxr": fx_r,
            "Fyr": fy_r,
            "Fzf": fzf,
            "Fzr": fzr,
            "deltaFz": delta_fz,
            "alpha_f": alpha_f,
            "alpha_r": alpha_r,
            "tan_alpha_f": tan_alpha_f,
            "tan_alpha_r": tan_alpha_r,
            "kappa_x_front": kappa_x_front,
            "kappa_x_rear": kappa_x_rear,
            "vx_front_wheel": vx_front_wheel,
            "vx_rear_wheel": vx_rear_wheel,
            "wr": wr,
            "yaw_rate_dot": r_dot,
            "vel_x_dot": vx_dot,
            "vel_y_dot": vy_dot,
            "rear_wheelspeed_ms_dot": wr_dot,
            "r_dot": r_dot,
            "vx_dot": vx_dot,
            "vy_dot": vy_dot,
            "accel_x": accel_x,
            "accel_y": accel_y,
            "wr_dot": wr_dot,
            "V_dot": v_dot,
            "beta_dot": beta_dot,
            "rear_wheel_torque": rear_wheel_torque,
            "tire_energy": tire_energy,
            "tire_saturation": tire_saturation,
            "total_rear_force": total_rear_force,
            "total_front_force": total_front_force,
        }
