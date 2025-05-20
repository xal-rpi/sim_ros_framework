from bng_simulator.utils.config_manager import ConfigManager
from bng_simulator.utils.resource_manager import ResourceManager


# Simple pure-pursuit path follower
import numpy as np
from math import atan2

class PurePursuit:
    def __init__(
        self,
        waypoints: np.ndarray,
        lookahead: float,
        max_steer: float,
    ):
        """
        waypoints: (N,2) array of [x,y].
        lookahead: lookahead distance ℓₑ.
        closed: whether to treat the path as a loop.
        """
        self.wps = waypoints
        self.ld = lookahead
        self.closed = np.allclose(self.wps[0], self.wps[-1])
        self.max_steer = max_steer
        self._compute_path_info()

    def _compute_path_info(self):
        # build segments (A->B), their lengths, and cumulative s
        w = self.wps
        if self.closed:
            A = w
            B = np.vstack((w[1:], w[0:1]))
        else:
            A = w[:-1]
            B = w[1:]
        V = B - A
        L = np.hypot(V[:, 0], V[:, 1])
        # cumulative distance at each waypoint (len = len(L)+1)
        s = np.zeros(len(L) + 1)
        s[1:] = np.cumsum(L)
        self.seg_A = A
        self.seg_V = V
        self.seg_L = L
        self.cum_s = s
        self.path_length = s[-1]

    def _find_lookahead_point(self, x: float, y: float):
        P = np.array([x, y])
        A = self.seg_A
        V = self.seg_V
        L2 = self.seg_L ** 2
        # compute projection param t on each segment
        AP = P - A  # shape (M,2)
        t = np.einsum("ij,ij->i", AP, V) / np.where(L2 > 0, L2, 1e-9)
        t_clamped = np.clip(t, 0.0, 1.0)
        # projection points Q = A + t_clamped * V
        Q = A + (V.T * t_clamped).T
        # distances from P to each Q
        d = np.hypot(*(Q - P).T)
        # choose the segment with minimal distance
        k = int(np.argmin(d))
        s_proj = self.cum_s[k] + t_clamped[k] * self.seg_L[k]

        # target along-track distance
        s_target = s_proj + self.ld
        if self.closed:
            s_target = s_target % self.path_length
        else:
            s_target = min(s_target, self.path_length)

        # find which segment contains s_target
        # cum_s is sorted, so searchsorted works
        j = np.searchsorted(self.cum_s, s_target) - 1
        j = np.clip(j, 0, len(self.seg_L) - 1)
        ds = s_target - self.cum_s[j]
        # param along segment j
        tj = ds / max(float(self.seg_L[j]), 1e-9)
        goal = self.seg_A[j] + tj * self.seg_V[j]
        return goal

    def steering(self, x: float, y: float, yaw: float, v: float):
        """
        Returns the steering angle δ (radians) via
        δ = arctan(2 * y_ld / (v * ℓₑ)), with wheelbase L = 1.
        """
        goal = self._find_lookahead_point(x, y)
        dx = goal[0] - x
        dy = goal[1] - y
        # transform into vehicle frame
        local_x = dx * np.cos(-yaw) - dy * np.sin(-yaw)
        local_y = dx * np.sin(-yaw) + dy * np.cos(-yaw)
        # pure pursuit law
        delta = atan2(2.0 * local_y, max(v, 1e-3) * self.ld)

        return max(-self.max_steer, min(self.max_steer, delta))


# load configuration & path at import time
_cfg = ConfigManager.get_config(None)
if _cfg is None:
    raise RuntimeError("Trying to import path_follower before config is intialized")
hlc = _cfg["high_level_controller"]
# expect a CSV file of [x,y] rows
_wps = np.loadtxt(
    ResourceManager.get_path("bng_controller", "paths/"+hlc["path_file"]), delimiter=","
)
_pp = PurePursuit(
    _wps,
    lookahead=hlc.get("lookahead", 5.0),
    max_steer=hlc.get("max_steer_rad", 0.69),
)


def compute_control_pure(sensor_data: dict, control_rate: float, max_latency: float):
    # extract pose & heading
    pos = sensor_data["position"]
    dir = sensor_data["direction"]
    x, y = pos["x"], pos["y"]
    yaw = atan2(dir["y"], dir["x"])
    v = sensor_data.get("velocity", {}).get("x", 0.0)

    # steering command
    steer = _pp.steering(x, y, yaw, v)
    # time stamping
    simt = sensor_data.get("simtime", 0.0)
    lat = min(max_latency + 0.005, 0.1)
    tcmd = simt + control_rate + lat

    return {
        "road_wheel_angle": steer,
        "time": tcmd,
    }
