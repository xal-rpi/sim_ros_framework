from bng_simulator.utils.config_manager import ConfigManager
from bng_simulator.utils.resource_manager import ResourceManager
import numpy as np
import math

_cfg = ConfigManager.get_config(None)
if _cfg is None:
    raise RuntimeError("Config must be initialized before path_follower")

hlc = _cfg["high_level_controller"]
_wps = np.loadtxt(
    ResourceManager.get_path("bng_controller", "paths/" + hlc["path_file"]),
    delimiter=",",
)

# Precompute segment vectors, lengths, cumulative s, and unit tangents
seg_vecs = _wps[1:] - _wps[:-1]  # shape (N-1,2)
seg_lens = np.hypot(seg_vecs[:, 0], seg_vecs[:, 1])  # shape (N-1,)
cum_lens = np.concatenate(([0.0], np.cumsum(seg_lens)))  # shape (N,)

# safe‐normalize into unit tangents
tangents = np.zeros_like(seg_vecs)
nonzero = seg_lens > 1e-12
tangents[nonzero, 0] = seg_vecs[nonzero, 0] / seg_lens[nonzero]
tangents[nonzero, 1] = seg_vecs[nonzero, 1] / seg_lens[nonzero]

wheelbase = hlc.get("wheelbase", 2.5)
max_steer = hlc.get("max_steer_rad", 0.69)
lookahead_base = hlc.get("pp_lookahead", 2.0)
lookahead_gain_k = hlc.get("pp_lookahead_gain", 0.4)
pct = hlc.get("lookahead_distances_pct", [0.4, 0.8, 1.2])
default_desired_spd = hlc.get("desired_speed", 10.0)
stanley_k = hlc.get("stanley_k", 0.7)


def compute_control(latest, control_rate, max_latency):
    """
    Calculates a list of targets using a Pure Pursuit strategy for each
    lookahead point.
    """
    # 1) Get current vehicle state from telemetry
    x, y = latest["position"]["x"], latest["position"]["y"]
    fx, fy = latest["direction"]["x"], latest["direction"]["y"]
    v_cur = np.hypot(latest["velocity"]["x"], latest["velocity"]["y"])

    # Ensure the forward vector is normalized
    vehicle_fwd_vector = np.array([fx, fy])
    norm_f = np.linalg.norm(vehicle_fwd_vector)
    if norm_f > 1e-12:
        vehicle_fwd_vector /= norm_f

    # 2) Calculate front axle position (the control reference point)
    x_axle = x + wheelbase * vehicle_fwd_vector[0]
    y_axle = y + wheelbase * vehicle_fwd_vector[1]
    P_axle = np.array([x_axle, y_axle])

    # 3) Find the nearest point on the path to the FRONT AXLE to get our current s
    p0 = _wps[:-1]
    w = P_axle - p0
    t = np.clip((w * seg_vecs).sum(axis=1) / (seg_lens**2 + 1e-12), 0.0, 1.0)
    i_seg = int(np.argmin(np.sum((p0 + (seg_vecs.T * t).T - P_axle) ** 2, axis=1)))
    s_proj_com = cum_lens[i_seg] + t[i_seg] * seg_lens[i_seg]

    # 4) Build and send targets using Pure Pursuit for each point
    targets = []
    time_val = latest.get("simtime", 0.0) + control_rate + max_latency
    v_des = default_desired_spd
    # Define lookahead distances. The first is 0 to represent the current plan.
    lookahead_s_values = [0.0, 5.0, 10.0, 15.0, 20.0]

    for s_la in lookahead_s_values:
        # Find the future point on the path
        s_star = min(s_proj_com + s_la, cum_lens[-1])
        j = int(np.searchsorted(cum_lens, s_star) - 1)
        j = max(min(j, len(seg_lens) - 1), 0)
        t_star = (s_star - cum_lens[j]) / seg_lens[j] if seg_lens[j] > 1e-6 else 0.0
        # This is our target point (lx, ly)
        lx, ly = _wps[j] + seg_vecs[j] * t_star

        # --- Pure Pursuit Calculation for this specific target point ---
        # Vector from front axle to target point
        vec_to_target = np.array([lx - x_axle, ly - y_axle])
        L_la = np.linalg.norm(vec_to_target)  # Actual lookahead distance

        # Find angle alpha between vehicle's fwd vector and the lookahead vector
        # using the cross product method in the vehicle's frame
        yb = np.dot(vec_to_target, [-vehicle_fwd_vector[1], vehicle_fwd_vector[0]])
        alpha = np.arctan2(yb, np.dot(vec_to_target, vehicle_fwd_vector))

        # Calculate the required steering angle to intercept this point
        delta = np.arctan2(2.0 * wheelbase * np.sin(alpha), L_la)
        delta_clipped = np.clip(delta, -max_steer, max_steer)
        # --- End Pure Pursuit Calculation ---

        # Get the path tangent at the target point for the executor's use
        txj, tyj = tangents[j]
        if np.dot([txj, tyj], vehicle_fwd_vector) < 0:
            txj, tyj = -txj, -tyj

        targets.append(
            {
                "x": float(lx),
                "y": float(ly),
                "z": 0.0,
                "s": float(s_la),
                "road_wheel_angle": float(delta_clipped),
                "desired_speed": float(v_des),
                "tx": float(txj),
                "ty": float(tyj),
                "tz": 0.0,
            }
        )
    return {"targets": targets, "time": time_val}


def compute_control_reach_point(latest, control_rate, max_latency):
    # unchanged
    x = latest["position"]["x"]
    y = latest["position"]["y"]
    fx = latest["direction"]["x"]
    fy = latest["direction"]["y"]
    wheelbase = hlc.get("wheelbase", 2.5)
    max_steer = hlc.get("max_steer_rad", 0.69)
    L_la = hlc.get("lookahead_distance_reach_point", 5.0)

    dx = _wps[0, 0] - x
    dy = _wps[0, 1] - y
    xb = dx * fx + dy * fy
    yb = dx * fy - dy * fx

    alpha = np.arctan2(yb, xb)
    delta = np.clip(
        np.arctan2(2 * wheelbase * np.sin(alpha), L_la), -max_steer, max_steer
    )

    simt = latest.get("simtime", 0.0)
    tcmd = simt + control_rate + min(max_latency + 0.005, 0.1)
    return {"road_wheel_angle": float(delta), "time": tcmd}
