from bng_simulator.utils.config_manager import ConfigManager
from bng_simulator.utils.resource_manager import ResourceManager
import numpy as np

# -- load config & raw waypoints --------------------------------------------
_cfg = ConfigManager.get_config(None)
if _cfg is None:
    raise RuntimeError("Config must be initialized before path_follower")

hlc = _cfg["high_level_controller"]
_wps = np.loadtxt(
    ResourceManager.get_path("bng_controller", "paths/" + hlc["path_file"]),
    delimiter=",",
)  # shape (N,2)

# Precompute segment vectors and cumulative arc‐lengths
seg_vecs = _wps[1:] - _wps[:-1]  # (N-1,2)
seg_lens = np.hypot(seg_vecs[:, 0], seg_vecs[:, 1])  # (N-1,)
cum_lens = np.concatenate(([0.0], np.cumsum(seg_lens)))  # (N,)

# Pure Pursuit parameters
L_la = hlc.get("lookahead_distance", 5.0)
wheelbase = hlc.get("wheelbase", 2.5)
max_steer = hlc.get("max_steer_rad", 1.0)


def compute_control_follow(latest, control_rate, max_latency):
    # 1) current position
    x = latest["position"]["x"]
    y = latest["position"]["y"]

    # 2) heading unit‐vector from BeamNG’s world‐frame direction
    fx = latest["direction"]["x"]
    fy = latest["direction"]["y"]
    # normalize (just in case)
    norm = np.hypot(fx, fy) + 1e-12
    fx /= norm
    fy /= norm

    P = np.array([x, y])

    # 3) project onto each segment to find the closest point
    p0 = _wps[:-1]  # (N-1,2)
    v_seg = seg_vecs  # (N-1,2)
    L2 = seg_lens**2 + 1e-12
    w = P - p0  # (N-1,2)
    t = (w * v_seg).sum(axis=1) / L2
    t = np.clip(t, 0.0, 1.0)
    projs = p0 + (v_seg.T * t).T  # (N-1,2)
    d2 = np.hypot(projs[:, 0] - x, projs[:, 1] - y)
    i_seg = int(np.argmin(d2))
    t_seg = t[i_seg]
    s_proj = cum_lens[i_seg] + t_seg * seg_lens[i_seg]

    # 4) unit‐tangent of the path at the closest segment
    tx, ty = seg_vecs[i_seg]
    Lseg = seg_lens[i_seg]
    tx /= Lseg + 1e-12
    ty /= Lseg + 1e-12

    # 5) lookahead point at arc‐length s* = s_proj + L_la
    s_star = min(s_proj + L_la, cum_lens[-1])
    j = int(np.searchsorted(cum_lens, s_star) - 1)
    j = max(min(j, len(seg_lens) - 1), 0)
    t_star = (s_star - cum_lens[j]) / (seg_lens[j] + 1e-12)
    lx, ly = _wps[j] + seg_vecs[j] * t_star

    # 6) world → right‐handed body‐frame (x forward, +y = right)
    dx = lx - x
    dy = ly - y
    x_b = dx * fx + dy * fy
    y_b = dx * fy - dy * fx

    # 7) pure‐pursuit steering law
    alpha = np.arctan2(y_b, x_b)
    delta = np.arctan2(2 * wheelbase * np.sin(alpha), L_la)
    delta = np.clip(delta, -max_steer, max_steer)

    # 8) package command
    simt = latest.get("simtime", 0.0)
    lat = min(max_latency + 0.005, 0.1)
    tcmd = simt + control_rate + lat

    return {
        "road_wheel_angle": float(delta),
        "time": tcmd,
    }


def compute_control_reach_point(latest, control_rate, max_latency):
    # 1) current pos & vel ------------------------------------------------
    x, y = latest["position"]["x"], latest["position"]["y"]
    vx = latest["velocity"]["x"]
    vy = latest["velocity"]["y"]
    fx = latest["direction"]["x"]
    fy = latest["direction"]["y"]

    # 2) world‐heading & target‐bearing ----------------------------------
    lx, ly = _wps[0]  # first waypoint
    dx, dy = lx - x, ly - y

    # 3) body‐frame transform (right‐handed: +y = right) -----------------
    #    right = ( fy, -fx )
    x_b = dx * fx + dy * fy
    y_b = dx * fy - dy * fx

    # 4) α and δ ----------------------------------------------------------
    α = np.arctan2(y_b, x_b)
    δ_raw = np.arctan2(2 * wheelbase * np.sin(α), L_la)

    δ = np.clip(δ_raw, -max_steer, max_steer)

    # 5) package command -------------------------------------------------
    simt = latest.get("simtime", 0.0)
    lat = min(max_latency + 0.005, 0.1)
    tcmd = simt + control_rate + lat

    return {
        "road_wheel_angle": float(δ),
        "time": tcmd,
    }
