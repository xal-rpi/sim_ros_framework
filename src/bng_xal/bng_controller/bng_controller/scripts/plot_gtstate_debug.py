#!/usr/bin/env python3
"""Plot gtState accel / vel / angVel (+ debug cross-checks) from a run log.

Loads pickle timeseries via ``bng_simulator.utils.logger_utils.load_run_data``
(rosbag ignored). Edit the hyperparameters block below, then:

    python3 -m bng_controller.scripts.plot_gtstate_debug
    # or, after install:
    ros2 run bng_controller plot_gtstate_debug

Useful comparisons when ``debug_raw`` was enabled in Lua:
  - angVel vs angVelRaw         : filter effect
  - angVelRaw vs angVelObjRPY   : ω_FLU vs raw RPY (q,r flip when A≈B)
  - vel vs velRaw               : filter effect on v_COM
  - velRef vs velRaw            : getVelocity() vs v_ref + ω×r
  - rGeom vs angVelRaw_z        : geometric yaw rate vs unfiltered r
  - transport figure            : velRef / velOmegaR / remapped RPY (SHOW_ATTITUDE_FIG)
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# =============================================================================
# Hyperparameters (edit these)
# =============================================================================

# Run location: provide either RUN_PATH, or (ROOT_DIR + RUN_NUMBER).
# New logs: ~/beamng_log_data/<vehicle>/run_XXX  (set VEHICLE or RUN_PATH).
ROOT_DIR = "~/beamng_log_data"
RUN_NUMBER = 1  # -> run_001
RUN_PATH: Optional[str] = None  # e.g. "~/beamng_log_data/utv_wild/run_001"
VEHICLE: Optional[str] = None  # e.g. "utv_wild" or "utv_canam_x3_loaded"

# Sensor key after load_run_data flattening: "/<vehicle>/<sensor>"
SENSOR_KEY = "/EGO/gtstate"

# Time window relative to first sample (seconds). None = full log.
T_START_REL_S: Optional[float] = None
T_END_REL_S: Optional[float] = None

# Decimate for plotting speed (1 = every sample). Stats use the full window.
PLOT_STRIDE = 5

# Axis labels / which components to show (0=x/p, 1=y/q, 2=z/r).
AXES = (0, 1, 2)
AXIS_NAMES = ("x / p", "y / q", "z / r")

# Raw getRollPitchYawAngularVelocity scalars (do not remap here).
# On FLU, p=+rollAV, q=−pitchAV, r=−yawAV when (A)≈(B). That is fixed.
OBJ_RPY_SIGN = (1.0, 1.0, 1.0)
FLU_FROM_RPY_SIGN = (1.0, -1.0, -1.0)

# Series groups to plot (field prefix without _x/_y/_z).
# Each entry: (label, field_prefix, linestyle, linewidth)
ANGVEL_SERIES = (
    ("angVel (filt)", "angVel", "-", 1.4),
    ("angVelRaw", "angVelRaw", "--", 1.0),
    ("angVelObjRPY", "angVelObjRPY", ":", 1.6),
)
ACCEL_SERIES = (
    ("accel (filt)", "accel", "-", 1.4),
    ("accelRaw", "accelRaw", "--", 1.0),
)
VEL_SERIES = (
    ("vel (published)", "vel", "-", 1.4),
    ("velRaw", "velRaw", "--", 1.0),
    ("velRef (getVelocity)", "velRef", ":", 1.0),
)

# Correlation pairs printed + scatter-plotted: (x_prefix, y_prefix, title)
CORR_PAIRS = (
    ("angVel", "angVelRaw", "Filtered vs Raw angVel"),
    ("angVelRaw", "angVelObjRPY", "ω_FLU vs raw ObjRPY (expect q,r flip)"),
    ("vel", "velRaw", "Filtered vs Raw vel"),
    ("velRef", "velRaw", "getVelocity vs v_COM"),
    ("accel", "accelRaw", "Filtered vs Raw accel"),
    ("dirX", "dirXBody", "Published x vs debug copy"),
)

# COM / transport figure (needs debug_raw: velRef, velOmegaR, angVelObjRPY).
SHOW_ATTITUDE_FIG = True
# 25–35 Hz band share is printed for these vel prefixes (axis = y / lateral).
ATTITUDE_BAND_HZ = (25.0, 35.0)
ATTITUDE_VEL_COMPARE = ("velRaw", "vel", "velRef")

# Display
SHOW_PLOTS = True
SAVE_FIG_PATH: Optional[str] = None  # e.g. "/tmp/gtstate_debug.png"
FIGSIZE = (14, 10)
DPI = 110

# =============================================================================


def _suffix(axis: int) -> str:
    return ("_x", "_y", "_z")[axis]


def _as_array(series) -> np.ndarray:
    return np.asarray(series, dtype=float)


def _get_vec(
    data: Dict[str, Sequence],
    prefix: str,
    mask: np.ndarray,
    *,
    signs: Optional[Sequence[float]] = None,
) -> np.ndarray:
    """Return (N, 3) array for prefix_x/y/z, optionally applying per-axis signs."""
    cols = []
    for i in range(3):
        key = prefix + _suffix(i)
        if key not in data:
            raise KeyError(
                f"Missing field '{key}' in log. Available keys containing "
                f"'{prefix}': {[k for k in data if prefix in k]}"
            )
        cols.append(_as_array(data[key])[mask])
    out = np.column_stack(cols)
    if signs is not None:
        out = out * np.asarray(signs, dtype=float)[None, :]
    return out


def _window_mask(t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    t0 = float(t[0])
    t_rel = t - t0
    mask = np.ones(t.shape, dtype=bool)
    if T_START_REL_S is not None:
        mask &= t_rel >= float(T_START_REL_S)
    if T_END_REL_S is not None:
        mask &= t_rel <= float(T_END_REL_S)
    if not np.any(mask):
        raise ValueError(
            f"Empty window: T_START_REL_S={T_START_REL_S}, T_END_REL_S={T_END_REL_S}, "
            f"t_rel span=[{t_rel[0]:.3f}, {t_rel[-1]:.3f}]"
        )
    return mask, t_rel[mask]


def _pearson(a: np.ndarray, b: np.ndarray) -> float:
    if a.size < 2:
        return float("nan")
    a = a - a.mean()
    b = b - b.mean()
    denom = np.linalg.norm(a) * np.linalg.norm(b)
    if denom < 1e-30:
        return float("nan")
    return float(np.dot(a, b) / denom)


def _fit_scale(x: np.ndarray, y: np.ndarray) -> float:
    """Least-squares scale a in y ~= a * x (through origin)."""
    xx = float(np.dot(x, x))
    if xx < 1e-30:
        return float("nan")
    return float(np.dot(x, y) / xx)


def _print_corr_table(data: Dict, mask: np.ndarray) -> None:
    print("\n=== Correlation / scale (y ≈ a·x) ===")
    header = f"{'pair':<36} {'axis':<8} {'pearson':>9} {'scale_a':>9} {'rms_x':>10} {'rms_y':>10}"
    print(header)
    print("-" * len(header))
    for x_pref, y_pref, title in CORR_PAIRS:
        signs_x = OBJ_RPY_SIGN if x_pref == "angVelObjRPY" else None
        signs_y = OBJ_RPY_SIGN if y_pref == "angVelObjRPY" else None
        try:
            x = _get_vec(data, x_pref, mask, signs=signs_x)
            y = _get_vec(data, y_pref, mask, signs=signs_y)
        except KeyError as exc:
            print(f"{title:<36} SKIP ({exc})")
            continue
        for ax in AXES:
            r = _pearson(x[:, ax], y[:, ax])
            a = _fit_scale(x[:, ax], y[:, ax])
            print(
                f"{title:<36} {AXIS_NAMES[ax]:<8} "
                f"{r:9.4f} {a:9.4f} "
                f"{np.sqrt(np.mean(x[:, ax]**2)):10.4g} "
                f"{np.sqrt(np.mean(y[:, ax]**2)):10.4g}"
            )


def _plot_group(
    ax_row,
    t: np.ndarray,
    data: Dict,
    mask: np.ndarray,
    series_defs,
    ylabel: str,
) -> None:
    idx = np.arange(t.size)[:: max(1, int(PLOT_STRIDE))]
    t_p = t[idx]
    for ax_i, axis in enumerate(AXES):
        ax = ax_row[ax_i]
        for label, prefix, ls, lw in series_defs:
            signs = OBJ_RPY_SIGN if prefix == "angVelObjRPY" else None
            try:
                vec = _get_vec(data, prefix, mask, signs=signs)
            except KeyError:
                continue
            ax.plot(t_p, vec[idx, axis], ls, lw=lw, label=label)
        ax.set_title(AXIS_NAMES[axis])
        ax.grid(True, alpha=0.3)
        if ax_i == 0:
            ax.set_ylabel(ylabel)
        ax.set_xlabel("t - t0 [s]")
    ax_row[0].legend(loc="upper right", fontsize=8)


def _plot_scatter_corr(data: Dict, mask: np.ndarray):
    import matplotlib.pyplot as plt

    n = len(CORR_PAIRS)
    fig, axes = plt.subplots(n, len(AXES), figsize=(4 * len(AXES), 3.2 * n), squeeze=False)
    fig.suptitle("Correlation scatters (subsampled)", fontsize=12)

    idx = np.arange(np.count_nonzero(mask))[:: max(1, int(PLOT_STRIDE))]
    for row, (x_pref, y_pref, title) in enumerate(CORR_PAIRS):
        signs_x = OBJ_RPY_SIGN if x_pref == "angVelObjRPY" else None
        signs_y = OBJ_RPY_SIGN if y_pref == "angVelObjRPY" else None
        try:
            x = _get_vec(data, x_pref, mask, signs=signs_x)
            y = _get_vec(data, y_pref, mask, signs=signs_y)
        except KeyError as exc:
            axes[row][0].set_title(f"{title}\nSKIP: {exc}")
            continue
        for col, axis in enumerate(AXES):
            ax = axes[row][col]
            xx, yy = x[idx, axis], y[idx, axis]
            ax.plot(xx, yy, ".", ms=2, alpha=0.35)
            a = _fit_scale(xx, yy)
            r = _pearson(xx, yy)
            lim = max(np.max(np.abs(xx)), np.max(np.abs(yy)), 1e-9)
            line = np.array([-lim, lim])
            ax.plot(line, line, "k-", lw=0.8, alpha=0.5, label="y=x")
            if np.isfinite(a):
                ax.plot(line, a * line, "r--", lw=1.0, label=f"a={a:.3f}")
            ax.set_aspect("equal", adjustable="datalim")
            ax.grid(True, alpha=0.3)
            ax.set_xlabel(f"{x_pref}[{AXIS_NAMES[axis]}]")
            ax.set_ylabel(f"{y_pref}[{AXIS_NAMES[axis]}]")
            if col == 0:
                ax.set_title(f"{title}\nr={r:.3f}")
            else:
                ax.set_title(f"r={r:.3f}")
            ax.legend(fontsize=7, loc="best")
    fig.tight_layout()
    return fig


def _band_share(x: np.ndarray, dt: float, f0: float, f1: float) -> float:
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    if x.size < 8 or dt <= 0:
        return float("nan")
    freqs = np.fft.rfftfreq(x.size, dt)
    ps = np.abs(np.fft.rfft(x)) ** 2
    tot = ps[freqs > 1.0].sum()
    if tot <= 0:
        return float("nan")
    return float(ps[(freqs >= f0) & (freqs <= f1)].sum() / tot)


def _print_attitude_band_table(data: Dict, mask: np.ndarray, t_rel: np.ndarray) -> None:
    f0, f1 = ATTITUDE_BAND_HZ
    dt = float(np.median(np.diff(t_rel))) if t_rel.size > 1 else float("nan")
    print(f"\n=== Attitude / frame check ({f0:.0f}-{f1:.0f} Hz band share, dt={dt:.4f}s) ===")
    header = f"{'signal':<22} {'axis':<8} {'band_share':>10} {'rms':>10}"
    print(header)
    print("-" * len(header))
    for pref in ATTITUDE_VEL_COMPARE:
        try:
            v = _get_vec(data, pref, mask)
        except KeyError as exc:
            print(f"{pref:<22} SKIP ({exc})")
            continue
        for ax in AXES:
            share = _band_share(v[:, ax], dt, f0, f1)
            print(
                f"{pref:<22} {AXIS_NAMES[ax]:<8} "
                f"{share:10.3f} {np.sqrt(np.mean(v[:, ax] ** 2)):10.4g}"
            )
    # dirY world-x is the usual flex carrier on the utv.
    for pref in ("dirY",):
        key = pref + "_x"
        if key not in data:
            print(f"{pref}_x SKIP (missing)")
            continue
        x = _as_array(data[key])[mask]
        share = _band_share(x, dt, f0, f1)
        print(
            f"{pref + '_x':<22} {'(world)':<8} "
            f"{share:10.3f} {np.sqrt(np.mean(x ** 2)):10.4g}"
        )


def _plot_attitude_frame(data: Dict, mask: np.ndarray, t_rel: np.ndarray):
    """COM transport + remapped RPY vs published ω."""
    import matplotlib.pyplot as plt

    idx = np.arange(t_rel.size)[:: max(1, int(PLOT_STRIDE))]
    t_p = t_rel[idx]

    try:
        vel_raw = _get_vec(data, "velRaw", mask)
        vel_ref = _get_vec(data, "velRef", mask)
    except KeyError as exc:
        print(f"Transport figure SKIP ({exc})")
        return None

    try:
        vel_pub = _get_vec(data, "vel", mask)
    except KeyError:
        vel_pub = None
    try:
        vel_wr = _get_vec(data, "velOmegaR", mask)
    except KeyError:
        vel_wr = None
    try:
        w_raw = _get_vec(data, "angVelRaw", mask)
        w_rpy = _get_vec(data, "angVelObjRPY", mask, signs=FLU_FROM_RPY_SIGN)
    except KeyError:
        w_raw, w_rpy = None, None

    fig, axes = plt.subplots(3, 3, figsize=(14, 9), dpi=DPI)
    fig.suptitle("COM transport / remapped RPY", fontsize=12)

    for col, axis in enumerate(AXES):
        ax = axes[0, col]
        if vel_pub is not None:
            ax.plot(t_p, vel_pub[idx, axis], "-", lw=1.3, label="vel (filt)")
        ax.plot(t_p, vel_raw[idx, axis], "--", lw=1.0, label="velRaw")
        ax.plot(t_p, vel_ref[idx, axis], ":", lw=1.0, label="velRef")
        if vel_wr is not None:
            ax.plot(t_p, vel_wr[idx, axis], "-.", lw=1.0, label="ω×r")
        ax.set_title(f"vel {AXIS_NAMES[axis]}")
        ax.grid(True, alpha=0.3)
        if col == 0:
            ax.set_ylabel("vel [m/s]")
        ax.set_xlabel("t - t0 [s]")
    axes[0, 0].legend(loc="upper right", fontsize=8)

    for col, axis in enumerate(AXES):
        ax = axes[1, col]
        if w_raw is not None:
            ax.plot(t_p, w_raw[idx, axis], "-", lw=1.2, label="angVelRaw")
        if w_rpy is not None:
            ax.plot(t_p, w_rpy[idx, axis], "--", lw=1.0, label="RPY→FLU")
        ax.set_title(f"ω {AXIS_NAMES[axis]}")
        ax.grid(True, alpha=0.3)
        if col == 0:
            ax.set_ylabel("angVel [rad/s]")
        ax.set_xlabel("t - t0 [s]")
    axes[1, 0].legend(loc="upper right", fontsize=8)

    ax = axes[2, 0]
    if "rGeom" in data and "angVelRaw_z" in data:
        rg = _as_array(data["rGeom"])[mask]
        rf = _as_array(data["angVelRaw_z"])[mask]
        ax.plot(t_p, rg[idx], "-", lw=1.2, label="rGeom")
        ax.plot(t_p, rf[idx], "--", lw=1.0, label="angVelRaw.r")
        ax.legend(loc="upper right", fontsize=8)
    ax.set_title("rGeom vs unfiltered r")
    ax.set_xlabel("t - t0 [s]")
    ax.set_ylabel("[rad/s]")
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    if "yawAtoB" in data:
        yaw = _as_array(data["yawAtoB"])[mask]
        ax.plot(t_p, yaw[idx], "-", lw=1.2)
        ax.set_title("yawAtoB (A vs B)")
    else:
        ax.set_title("yawAtoB SKIP")
    ax.set_xlabel("t - t0 [s]")
    ax.set_ylabel("[rad]")
    ax.grid(True, alpha=0.3)

    ax = axes[2, 2]
    if vel_wr is not None:
        residual = vel_raw - vel_ref - vel_wr
        for axis in AXES:
            ax.plot(t_p, residual[idx, axis], lw=1.0, label=AXIS_NAMES[axis])
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title("velRaw − velRef − ω×r")
    else:
        ax.set_title("transport residual SKIP")
    ax.set_xlabel("t - t0 [s]")
    ax.set_ylabel("[m/s]")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def main() -> int:
    from bng_simulator.utils.logger_utils import load_run_data

    run_path = os.path.expanduser(RUN_PATH) if RUN_PATH else None
    print(
        f"Loading run: path={run_path!r} root={ROOT_DIR!r} number={RUN_NUMBER} "
        f"vehicle={VEHICLE!r} (pickle only)"
    )
    merged = load_run_data(
        run_number=None if run_path else RUN_NUMBER,
        run_path=run_path,
        root_dir=ROOT_DIR,
        vehicle=VEHICLE,
        include_pickle=True,
        include_rosbag=False,
    )

    if SENSOR_KEY not in merged:
        # Helpful listing when the key is wrong.
        print(f"SENSOR_KEY={SENSOR_KEY!r} not found. Available keys:")
        for k in sorted(merged.keys(), key=str):
            print(f"  {k}")
        return 1

    data = merged[SENSOR_KEY]
    if "time" not in data:
        print(f"No 'time' field under {SENSOR_KEY}. Keys: {list(data.keys())[:20]}")
        return 1

    t_abs = _as_array(data["time"])
    mask, t_rel = _window_mask(t_abs)
    print(
        f"Loaded {SENSOR_KEY}: N={t_abs.size}, window N={int(mask.sum())}, "
        f"t_rel=[{t_rel[0]:.3f}, {t_rel[-1]:.3f}] s, "
        f"OBJ_RPY_SIGN={OBJ_RPY_SIGN}"
    )

    # Quick availability of debug fields.
    for pref in (
        "angVelRaw",
        "angVelObjRPY",
        "velRaw",
        "velRef",
        "velOmegaR",
        "accelRaw",
        "dirXBody",
        "rFlu",
    ):
        ok = all((pref + _suffix(i)) in data for i in range(3))
        print(f"  field {pref}_*: {'OK' if ok else 'MISSING'}")
    if "yawAtoB" in data:
        yaw = _as_array(data["yawAtoB"])[mask]
        print(
            f"  yawAtoB [rad]: mean={float(np.mean(yaw)):+.4f}  "
            f"rms={float(np.sqrt(np.mean(yaw**2))):.4f}  "
            f"maxabs={float(np.max(np.abs(yaw))):.4f}"
        )
    if "rGeom" in data and "angVelRaw_z" in data:
        rg = _as_array(data["rGeom"])[mask]
        rf = _as_array(data["angVelRaw_z"])[mask]
        r = _pearson(rg, rf)
        a = _fit_scale(rg, rf)
        print(
            f"  rGeom vs angVelRaw_z: pearson={r:+.4f}  scale={a:+.4f}  "
            f"rms_geom={float(np.sqrt(np.mean(rg**2))):.4f}  "
            f"rms_r={float(np.sqrt(np.mean(rf**2))):.4f}"
        )
    if all((f"angVelObjRPY{_suffix(i)}") in data for i in range(3)) and all(
        (f"angVelRaw{_suffix(i)}") in data for i in range(3)
    ):
        rpy = _get_vec(data, "angVelObjRPY", mask, signs=FLU_FROM_RPY_SIGN)
        flu = _get_vec(data, "angVelRaw", mask)
        print("  remapped ObjRPY (+p,−q,−r) vs angVelRaw:")
        for ax in AXES:
            rr = _pearson(rpy[:, ax], flu[:, ax])
            aa = _fit_scale(rpy[:, ax], flu[:, ax])
            print(
                f"    {AXIS_NAMES[ax]:<8} pearson={rr:+.4f}  scale={aa:+.4f}"
            )

    _print_corr_table(data, mask)
    _print_attitude_band_table(data, mask, t_rel)

    if not SHOW_PLOTS and not SAVE_FIG_PATH:
        return 0

    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, len(AXES), figsize=FIGSIZE, dpi=DPI, sharex=True)
    fig.suptitle(f"gtState debug — {SENSOR_KEY} @ {ROOT_DIR} run_{RUN_NUMBER:03d}", fontsize=12)
    _plot_group(axes[0], t_rel, data, mask, ANGVEL_SERIES, "angVel [rad/s]")
    _plot_group(axes[1], t_rel, data, mask, VEL_SERIES, "vel [m/s]")
    _plot_group(axes[2], t_rel, data, mask, ACCEL_SERIES, "accel [m/s²]")
    fig.tight_layout()

    fig2 = _plot_scatter_corr(data, mask)
    fig3 = _plot_attitude_frame(data, mask, t_rel) if SHOW_ATTITUDE_FIG else None

    if SAVE_FIG_PATH:
        out = os.path.expanduser(SAVE_FIG_PATH)
        base, ext = os.path.splitext(out)
        ext = ext or ".png"
        fig.savefig(out, dpi=DPI)
        fig2.savefig(f"{base}_scatter{ext}", dpi=DPI)
        saved = [out, f"{base}_scatter{ext}"]
        if fig3 is not None:
            fig3.savefig(f"{base}_attitude{ext}", dpi=DPI)
            saved.append(f"{base}_attitude{ext}")
        print("Saved figures to " + ", ".join(saved))

    if SHOW_PLOTS:
        plt.show()
    else:
        plt.close("all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
