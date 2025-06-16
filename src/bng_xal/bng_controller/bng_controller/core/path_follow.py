from bng_simulator.utils.config_manager import ConfigManager
from bng_simulator.utils.resource_manager import ResourceManager
import numpy as np

_cfg = ConfigManager.get_config(None)
if _cfg is None:
    raise RuntimeError("Config must be initialized before path_follower")

hlc = _cfg["high_level_controller"]
_wps = np.loadtxt(
    ResourceManager.get_path("bng_controller", "paths/" + hlc["path_file"]),
    delimiter=",",
)
seg_vecs = _wps[1:] - _wps[:-1]
seg_lens = np.hypot(seg_vecs[:, 0], seg_vecs[:, 1])
cum_lens = np.concatenate(([0.0], np.cumsum(seg_lens)))
wheelbase = hlc.get("wheelbase", 2.5)
max_steer = hlc.get("max_steer_rad", 0.69)
lookahead_base = hlc.get("pp_lookahead", 2.0)
lookahead_gain_k = hlc.get("pp_lookahead_gain", 0.4)
pct = hlc.get("lookahead_distances_pct", [0.4, 0.8, 1.2])


def compute_control(latest, control_rate, max_latency):
    # 1) current speed & reachable distance per HLC cycle
    vx = latest["velocity"]["x"]
    vy = latest["velocity"]["y"]
    v_cur = np.hypot(vx, vy)
    dt = control_rate
    R = v_cur * dt

    # 2) lookahead distances as pct of R (pct>1 → unreachable)
    lookahead_distances = [R * p for p in pct]

    # 3) current pos & heading unit-vector
    x = latest["position"]["x"]
    y = latest["position"]["y"]
    fx = latest["direction"]["x"]
    fy = latest["direction"]["y"]
    norm = np.hypot(fx, fy) + 1e-12
    fx /= norm
    fy /= norm
    P = np.array([x, y])

    # 4) project P onto piecewise-linear path → s_proj
    p0 = _wps[:-1]
    L2 = seg_lens**2 + 1e-12
    w = P - p0
    t = (w * seg_vecs).sum(axis=1) / L2
    t = np.clip(t, 0.0, 1.0)
    projs = p0 + (seg_vecs.T * t).T
    d2 = np.hypot(projs[:, 0] - x, projs[:, 1] - y)
    i_seg = int(np.argmin(d2))
    s_proj = cum_lens[i_seg] + t[i_seg] * seg_lens[i_seg]

    # 5) build targets
    targets = []
    time_val = latest.get("simtime", 0) + dt + max_latency
    lookahead_for_steering = lookahead_base + lookahead_gain_k * v_cur
    v_des = hlc.get("desired_speed", 10.0)

    s_star_first = min(s_proj + lookahead_distances[0], cum_lens[-1])
    j_first = int(np.searchsorted(cum_lens, s_star_first) - 1)
    j_first = max(min(j_first, len(seg_lens) - 1), 0)
    t_star_first = (
        (s_star_first - cum_lens[j_first]) / seg_lens[j_first]
        if seg_lens[j_first] > 1e-6
        else 0.0
    )
    lx_first, ly_first = _wps[j_first] + seg_vecs[j_first] * t_star_first
    dx_first, dy_first = lx_first - x, ly_first - y
    x_b_first = dx_first * fx + dy_first * fy
    y_b_first = dx_first * fy - dy_first * fx
    alpha_first = np.arctan2(y_b_first, x_b_first)
    delta_base = np.arctan2(
        2 * wheelbase * np.sin(alpha_first), lookahead_for_steering
    )
    delta_base = np.clip(delta_base, -max_steer, max_steer)

    # Add the base target at the vehicle's current position with s=0
    targets.append(
        {
            "x": float(x),
            "y": float(y),
            "z": float(latest["position"]["z"]),
            "s": 0.0,
            "road_wheel_angle": float(delta_base),
            "desired_speed": float(v_des),
        }
    )

    # Now, create the future targets as before
    for s_la in lookahead_distances:
        s_star = min(s_proj + s_la, cum_lens[-1])
        j = int(np.searchsorted(cum_lens, s_star) - 1)
        j = max(min(j, len(seg_lens) - 1), 0)

        t_star = (
            (s_star - cum_lens[j]) / seg_lens[j]
            if seg_lens[j] > 1e-6
            else 0.0
        )

        lx, ly = _wps[j] + seg_vecs[j] * t_star
        lz = 0.0  # Assuming flat for now

        # body‐frame
        dx, dy = lx - x, ly - y
        x_b = dx * fx + dy * fy
        y_b = dx * fy - dy * fx

        alpha = np.arctan2(y_b, x_b)
        delta = np.arctan2(
            2 * wheelbase * np.sin(alpha), lookahead_for_steering
        )
        delta = np.clip(delta, -max_steer, max_steer)

        targets.append(
            {
                "x": float(lx),
                "y": float(ly),
                "z": float(lz),
                "s": float(s_la),
                "road_wheel_angle": float(delta),
                "desired_speed": float(v_des),
            }
        )

    return {"targets": targets, "time": time_val}


def compute_control_reach_point(latest, control_rate, max_latency):
    # unchanged
    x = latest["position"]["x"]
    y = latest["position"]["y"]
    vx = latest["velocity"]["x"]
    vy = latest["velocity"]["y"]
    L_la = hlc.get("lookahead_distance_reach_point", 5.0)
    fx = latest["direction"]["x"]
    fy = latest["direction"]["y"]

    dx = _wps[0, 0] - x
    dy = _wps[0, 1] - y

    x_b = dx * fx + dy * fy
    y_b = dx * fy - dy * fx

    alpha = np.arctan2(y_b, x_b)
    delta = np.arctan2(2 * wheelbase * np.sin(alpha), L_la)
    delta = np.clip(delta, -max_steer, max_steer)

    simt = latest.get("simtime", 0.0)
    tcmd = simt + control_rate + min(max_latency + 0.005, 0.1)

    return {"road_wheel_angle": float(delta), "time": tcmd}
