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
