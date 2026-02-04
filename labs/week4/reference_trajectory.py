"""reference_trajectory.py

The intent is to hide the geometric bookkeeping (arc-length s, projection onto
polyline, interpolation, etc.) so labs can focus on implementing the
controller law (pure pursuit, Stanley, MPC, ...).

Typical workflow
----------------
1) In a notebook, generate a reference trajectory and write it to a CSV.
2) Load it here as a `ReferenceTrajectory`.
3) Each control step:
   - Estimate current progress `s_closest` from (x, y)
   - Query a lookahead target point given a lookahead distance `Ld`

CSV format
----------
This loader is intentionally flexible.

Required columns (any one of each group):
- x: one of {"x", "pos_x", "X"}
- y: one of {"y", "pos_y", "Y"}

Optional columns:
- s: one of {"s", "arc_length"}
  If absent, s is computed from consecutive Euclidean distances.
- yaw: one of {"yaw", "psi"}
  If absent, yaw is computed from the path tangent.

Dependencies
------------
- numpy only

"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np


def _wrap_angle_pi(angle_rad: np.ndarray | float) -> np.ndarray | float:
    """Wrap angle(s) to [-pi, pi)."""
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def _unwrap_angle(angle_rad: np.ndarray) -> np.ndarray:
    """Unwrap angle array to be continuous."""
    return np.unwrap(np.asarray(angle_rad, dtype=float))


def _read_csv_columns(csv_path: str) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    """Read a CSV with a header using numpy only.

    Returns
    -------
    header : np.ndarray[str]
        Column names.
    cols : dict[str, np.ndarray]
        Each column as float array.

    Notes
    -----
    - Uses `np.genfromtxt` for portability.
    - Will raise a clear error if the file cannot be parsed.
    """
    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=float)
    except Exception as e:
        raise RuntimeError(f"Failed to read CSV '{csv_path}': {e}") from e

    if data.dtype.names is None:
        raise ValueError(
            f"CSV '{csv_path}' appears to have no header. "
            "Expected a header row with column names (e.g., x,y)."
        )

    cols: Dict[str, np.ndarray] = {}
    for name in data.dtype.names:
        cols[name] = np.asarray(data[name], dtype=float)

    header = np.asarray(list(data.dtype.names), dtype=object)
    return header, cols


def _pick_column(cols: Dict[str, np.ndarray], candidates: Tuple[str, ...], csv_path: str) -> np.ndarray:
    """Pick the first existing column from candidates."""
    for key in candidates:
        if key in cols:
            return cols[key]
    raise KeyError(f"CSV '{csv_path}' missing required column. Tried: {candidates}. Found: {list(cols.keys())}")


def _dedupe_by_distance(x: np.ndarray, y: np.ndarray, min_dist: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Drop consecutive near-duplicate points.

    Returns
    -------
    x2, y2, keep_mask
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(x) < 2:
        keep = np.ones_like(x, dtype=bool)
        return x, y, keep

    ds = np.hypot(np.diff(x), np.diff(y))
    keep = np.ones(len(x), dtype=bool)
    keep[1:] = ds > float(min_dist)
    return x[keep], y[keep], keep


def _arc_length_from_xy(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Compute cumulative arc-length s for a polyline."""
    ds = np.hypot(np.diff(x), np.diff(y))
    return np.concatenate(([0.0], np.cumsum(ds)))


def _yaw_from_xy(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Compute path-tangent yaw from x,y samples.

    Uses centered differences for smoother yaw than forward differences.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(x)
    if n < 2:
        return np.zeros(n, dtype=float)

    dx = np.zeros(n, dtype=float)
    dy = np.zeros(n, dtype=float)

    # Centered differences in the interior
    dx[1:-1] = x[2:] - x[:-2]
    dy[1:-1] = y[2:] - y[:-2]

    # One-sided at the ends
    dx[0] = x[1] - x[0]
    dy[0] = y[1] - y[0]
    dx[-1] = x[-1] - x[-2]
    dy[-1] = y[-1] - y[-2]

    yaw = np.arctan2(dy, dx)
    return _unwrap_angle(yaw)


@dataclass(frozen=True)
class RefPoint:
    """A single reference sample."""

    s: float
    x: float
    y: float
    yaw: float


class ReferenceTrajectory:
    """Reference trajectory sampled along arc-length.

    This class provides:
    - Projection of a vehicle position (x,y) onto the path -> s_closest
    - Interpolation at any s -> reference (x,y,yaw)
    - Lookahead query: s_target = s_closest + Ld

    Design notes (for teaching)
    ---------------------------
    - Uses a polyline model (piecewise linear between CSV waypoints).
    - Uses linear interpolation in s.
    - Uses only numpy; no ROS, no SciPy.

    Parameters
    ----------
    csv_path : str
        Path to the CSV file.
    closed : bool
        If True, treat the path as a loop (wrap s). For now, closest-point search
        is still performed on the polyline as given; wrapping affects interpolation.
    min_point_dist : float
        Minimum distance between consecutive points; closer points are dropped.
    """

    def __init__(
        self,
        csv_path: str,
        *,
        closed: bool = False,
        min_point_dist: float = 1e-3,
    ):
        self.csv_path = str(csv_path)
        self.closed = bool(closed)
        self.min_point_dist = float(min_point_dist)

        header, cols = _read_csv_columns(self.csv_path)

        x_raw = _pick_column(cols, ("x", "pos_x", "X"), self.csv_path)
        y_raw = _pick_column(cols, ("y", "pos_y", "Y"), self.csv_path)

        # Remove NaNs early
        valid = np.isfinite(x_raw) & np.isfinite(y_raw)
        x_raw = x_raw[valid]
        y_raw = y_raw[valid]

        if len(x_raw) < 2:
            raise ValueError(f"CSV '{self.csv_path}' must contain at least 2 valid (x,y) points.")

        x, y, keep = _dedupe_by_distance(x_raw, y_raw, min_dist=self.min_point_dist)
        if len(x) < 2:
            raise ValueError(
                f"After dedupe (min_point_dist={self.min_point_dist}), not enough points remain ({len(x)})."
            )

        # Handle optional s / yaw columns; if present, apply the same keep-mask.
        s = None
        for s_key in ("s", "arc_length"):
            if s_key in cols:
                s0 = np.asarray(cols[s_key], dtype=float)[valid]
                s = s0[keep]
                break

        yaw = None
        for yaw_key in ("yaw", "psi"):
            if yaw_key in cols:
                yaw0 = np.asarray(cols[yaw_key], dtype=float)[valid]
                yaw = yaw0[keep]
                break

        if s is None:
            s = _arc_length_from_xy(x, y)
        else:
            # Ensure s is increasing (monotonic). If not, recompute.
            if not np.all(np.diff(s) >= -1e-9):
                s = _arc_length_from_xy(x, y)

        if yaw is None:
            yaw = _yaw_from_xy(x, y)
        else:
            yaw = _unwrap_angle(yaw)

        # Store arrays
        self.s_ref = np.asarray(s, dtype=float)
        self.x_ref = np.asarray(x, dtype=float)
        self.y_ref = np.asarray(y, dtype=float)
        self.yaw_ref = np.asarray(yaw, dtype=float)

        # Precompute segment data for projection
        self._seg_ax = self.x_ref[:-1]
        self._seg_ay = self.y_ref[:-1]
        self._seg_bx = self.x_ref[1:]
        self._seg_by = self.y_ref[1:]
        self._seg_vx = self._seg_bx - self._seg_ax
        self._seg_vy = self._seg_by - self._seg_ay
        self._seg_len2 = self._seg_vx**2 + self._seg_vy**2

        self.length = float(self.s_ref[-1])
        if self.length <= 0.0:
            raise ValueError(f"Reference length is zero for CSV '{self.csv_path}'.")

    # ----------------------------
    # Basic interpolation utilities
    # ----------------------------

    def _wrap_s(self, s: float) -> float:
        """Wrap s for closed-loop references."""
        if not self.closed:
            return float(np.clip(s, self.s_ref[0], self.s_ref[-1]))
        # Wrap into [0, length)
        return float(s % self.length)

    def ref_at_s(self, s: float) -> RefPoint:
        """Interpolate reference (x,y,yaw) at arc-length s."""
        s_q = self._wrap_s(float(s))

        x = float(np.interp(s_q, self.s_ref, self.x_ref))
        y = float(np.interp(s_q, self.s_ref, self.y_ref))

        # yaw_ref is unwrapped; interpolation is meaningful. For closed loops,
        # yaw may have a discontinuity at the wrap point; keep it simple here.
        yaw = float(np.interp(s_q, self.s_ref, self.yaw_ref))
        return RefPoint(s=s_q, x=x, y=y, yaw=yaw)

    def lookahead_from_s(self, s_closest: float, Ld: float) -> RefPoint:
        """Return the lookahead reference point at s_target = s_closest + Ld."""
        if Ld <= 0.0:
            raise ValueError("Ld must be > 0")
        s_target = float(s_closest) + float(Ld)
        return self.ref_at_s(s_target)

    # ----------------------------
    # Closest-point projection
    # ----------------------------

    def closest_s(
        self,
        x: float,
        y: float,
        *,
        last_s: Optional[float] = None,
        s_back: float = 10.0,
        s_fwd: float = 40.0,
    ) -> Tuple[float, float, float, float]:
        """Project point (x,y) onto the polyline.

        Parameters
        ----------
        x, y : float
            Query point in the same frame as the CSV.
        last_s : Optional[float]
            If provided, restrict the projection search to a window around last_s.
            This prevents jumping to a different part of the track on loops.
        s_back, s_fwd : float
            Window size in meters if last_s is used.

        Returns
        -------
        s_proj : float
            Arc-length of the projection point.
        x_proj, y_proj : float
            Projection point coordinates.
        dist : float
            Euclidean distance from (x,y) to projection point.

        Notes
        -----
        This is the core geometric primitive
        """
        px = float(x)
        py = float(y)

        # Decide segment index range to search.
        if last_s is None or not np.isfinite(last_s):
            i0 = 0
            i1 = len(self._seg_ax)
        else:
            s_center = self._wrap_s(float(last_s)) if self.closed else float(last_s)
            s0 = float(np.clip(s_center - float(s_back), self.s_ref[0], self.s_ref[-1]))
            s1 = float(np.clip(s_center + float(s_fwd), self.s_ref[0], self.s_ref[-1]))
            # Convert s bounds to indices; projection is done on segments.
            i0 = int(np.searchsorted(self.s_ref, s0, side="left")) - 1
            i1 = int(np.searchsorted(self.s_ref, s1, side="right"))
            i0 = int(np.clip(i0, 0, len(self._seg_ax) - 1))
            i1 = int(np.clip(i1, i0 + 1, len(self._seg_ax)))

        best_dist = float("inf")
        best_s = float(self.s_ref[i0])
        best_x = float(self.x_ref[i0])
        best_y = float(self.y_ref[i0])

        eps = 1e-12
        for i in range(i0, i1):
            len2 = float(self._seg_len2[i])
            if len2 < eps:
                continue

            ax = float(self._seg_ax[i])
            ay = float(self._seg_ay[i])
            vx = float(self._seg_vx[i])
            vy = float(self._seg_vy[i])

            # Projection parameter t onto the segment (clamped)
            t = ((px - ax) * vx + (py - ay) * vy) / len2
            if t < 0.0:
                t = 0.0
            elif t > 1.0:
                t = 1.0

            proj_x = ax + t * vx
            proj_y = ay + t * vy
            dist = float(np.hypot(px - proj_x, py - proj_y))

            if dist < best_dist:
                s_i = float(self.s_ref[i])
                s_ip1 = float(self.s_ref[i + 1])
                s_proj = s_i + t * (s_ip1 - s_i)

                best_dist = dist
                best_s = s_proj
                best_x = proj_x
                best_y = proj_y

        if self.closed:
            best_s = self._wrap_s(best_s)

        return float(best_s), float(best_x), float(best_y), float(best_dist)

    # ----------------------------
    # Convenience: one-call query
    # ----------------------------

    def lookahead_from_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        Ld: float,
        *,
        last_s: Optional[float] = None,
        s_back: float = 10.0,
        s_fwd: float = 40.0,
    ) -> Dict[str, float]:
        """Closest + lookahead query in one call.

        Returns a dict so it’s easy to log/plot, e.g. with pandas or PlotJuggler.
        """
        s_closest, x_proj, y_proj, dist = self.closest_s(
            x, y, last_s=last_s, s_back=s_back, s_fwd=s_fwd
        )
        ref_target = self.lookahead_from_s(s_closest, Ld)

        heading_to_target = float(np.arctan2(ref_target.y - float(y), ref_target.x - float(x)))
        alpha = float(_wrap_angle_pi(heading_to_target - float(yaw)))

        return {
            "s_closest": float(s_closest),
            "x_proj": float(x_proj),
            "y_proj": float(y_proj),
            "proj_dist": float(dist),
            "s_target": float(ref_target.s),
            "x_target": float(ref_target.x),
            "y_target": float(ref_target.y),
            "yaw_target": float(ref_target.yaw),
            "alpha": float(alpha),
        }
