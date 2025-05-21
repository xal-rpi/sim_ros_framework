from bng_simulator.utils.config_manager import ConfigManager
from bng_simulator.utils.resource_manager import ResourceManager
import numpy as np

# -- load config & raw waypoints --------------------------------------------
_cfg = ConfigManager.get_config(None)
if _cfg is None:
    raise RuntimeError("Config must be initialized before path_follower")

hlc = _cfg["high_level_controller"]
_wps = np.loadtxt(
    ResourceManager.get_path("bng_controller",
                             "paths/" + hlc["path_file"]),
    delimiter=","
)  # shape (N,2)

# Precompute segment vectors and cumulative arc‐lengths
seg_vecs = _wps[1:] - _wps[:-1]                              # (N-1,2)
seg_lens = np.hypot(seg_vecs[:,0], seg_vecs[:,1])            # (N-1,)
cum_lens = np.concatenate(([0.0], np.cumsum(seg_lens)))     # (N,)

# Pure Pursuit parameters
L_la = hlc.get("lookahead_distance", 5.0)
wheelbase = hlc.get("wheelbase", 2.5)
max_steer = hlc.get("max_steer_rad", 1.0)

def compute_control_follow(latest, control_rate, max_latency):
    x = latest["position"]["x"]
    y = latest["position"]["y"]
    vx = latest["velocity"]["x"]
    vy = latest["velocity"]["y"]
    speed = np.hypot(vx, vy)
    if speed > 1e-2:
        fx, fy = vx/speed, vy/speed
    else:
        # fallback to heading vector if nearly stopped
        fx = latest["direction"]["x"]
        fy = latest["direction"]["y"]
        # renormalize
        norm = np.hypot(fx, fy) + 1e-12
        fx, fy = fx/norm, fy/norm

    P = np.array([x, y])

    # 1) project onto every segment, find closest point -------------------
    p0 = _wps[:-1]                # segment start points (N-1,2)
    v  = seg_vecs                 # segment vectors    (N-1,2)
    L2 = seg_lens**2 + 1e-12
    w  = P - p0                   # (N-1,2)

    # parameter t along each segment
    t = (w * v).sum(axis=1) / L2
    t = np.clip(t, 0.0, 1.0)      # clamp to segment

    # actual projection points
    projs = p0 + (v.T * t).T      # (N-1,2)
    d2    = np.hypot(projs[:,0] - x,
                    projs[:,1] - y)
    i_seg = int(np.argmin(d2))    # index of closest segment
    t_seg = t[i_seg]

    # nearest‐point world coords
    wp_px, wp_py = projs[i_seg]
    # arc‐length position along path
    s_proj = cum_lens[i_seg] + t_seg * seg_lens[i_seg]

    print(f"[1] pos=({x:.2f},{y:.2f})  "
          f"near_seg={i_seg} near_pt=({wp_px:.2f},{wp_py:.2f})  "
          f"s_proj={s_proj:.2f}")

    # 2) tangent at that segment
    tx, ty = seg_vecs[i_seg]
    Lseg = seg_lens[i_seg]
    tx, ty = tx/(Lseg+1e-12), ty/(Lseg+1e-12)

    # headings
    theta_h = np.arctan2(fy, fx)
    theta_t = np.arctan2(ty, tx)
    theta_v = np.arctan2(vy, vx)
    ang_diff = (theta_h - theta_t + np.pi) % (2*np.pi) - np.pi

    print(f"[2] h=({fx:.2f},{fy:.2f}) θ_h={np.degrees(theta_h):+.1f}°  "
          f"v=({vx:.2f},{vy:.2f}) θ_v={np.degrees(theta_v):+.1f}°  "
          f"t=({tx:.2f},{ty:.2f}) Δθ={np.degrees(ang_diff):+.1f}°")

    # 3) pick lookahead‐arc s* = s_proj + L_la
    s_star = s_proj + L_la
    # clamp to path end
    s_star = min(s_star, cum_lens[-1])

    # find segment j so cum_lens[j] ≤ s_star ≤ cum_lens[j+1]
    j = int(np.searchsorted(cum_lens, s_star) - 1)
    j = max(min(j, len(seg_lens)-1), 0)
    # local interpolation on segment j
    t_star = (s_star - cum_lens[j]) / (seg_lens[j] + 1e-12)
    lx, ly = _wps[j] + seg_vecs[j] * t_star

    arc_len = s_star - s_proj
    print(f"[3] i_seg={i_seg} → lookahead_seg={j} "
          f"arc_len={arc_len:.2f}")

    # 4) world→body
    dx, dy = lx - x, ly - y
    # forward = (fx,fy), left = (-fy,fx)
    x_l = dx*fx + dy*fy
    y_l = dx*(-fy) + dy*fx

    print(f"[4] target_ws=({lx:.2f},{ly:.2f})  "
          f"w2b=({x_l:.2f},{y_l:.2f})")

    # 5) pure pursuit steering
    alpha = np.arctan2(y_l, x_l)
    delta = np.arctan2(2*wheelbase*np.sin(alpha), L_la)
    delta = np.clip(delta, -max_steer, max_steer)

    print(f"[5] α={np.degrees(alpha):+.1f}°  "
          f"δ={np.degrees(delta):+.1f}°\n", flush=True)

    # build command
    simt = latest.get("simtime", 0.0)
    lat  = min(max_latency + 0.005, 0.1)
    tcmd = simt + control_rate + lat
    return {
        "road_wheel_angle": float(delta),
        "time": tcmd,
    }
