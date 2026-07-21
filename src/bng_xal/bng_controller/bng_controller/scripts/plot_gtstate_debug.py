#!/usr/bin/env python3
"""Plot gtState accel / vel / angVel / angAccel (+ debug cross-checks) from a run log.

Loads pickle timeseries via ``bng_simulator.utils.logger_utils.load_run_data``
(rosbag ignored). Edit the hyperparameters block below, then:

    python3 -m bng_controller.scripts.plot_gtstate_debug
    # or, after install:
    ros2 run bng_controller plot_gtstate_debug

Useful comparisons when ``debug_raw`` was enabled in Lua:
  - angVel vs angVelRaw         : filter effect
  - angVelRaw vs angVelUncorr   : tilted-triangle M^-1 correction
  - angVelRaw vs angVelObjRPY   : sensor estimate vs engine all-node p,q,r
  - dirY vs dirYTri             : published left vs raw triangle left (attitudeMode)
  - velRaw vs velTri            : same v_world in published vs triangle FLU
  - attitude figure             : dirs + vel overlay + PSD (enable SHOW_ATTITUDE_FIG)
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# =============================================================================
# Hyperparameters (edit these)
# =============================================================================

# Run location: provide either RUN_PATH, or (ROOT_DIR + RUN_NUMBER).
ROOT_DIR = "~/beamng_log_data/px4_replay_sysid_new"
RUN_NUMBER = 1  # -> run_001
RUN_PATH: Optional[str] = None  # e.g. "~/beamng_log_data/px4_replay_sysid_v3/run_001"

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

# Optional per-axis sign for engine ObjRPY (refNode frame may differ from
# sensor FLU). Start with +1; flip an axis if scatter slope is clearly negative.
OBJ_RPY_SIGN = (1.0, 1.0, 1.0)

# Series groups to plot (field prefix without _x/_y/_z).
# Each entry: (label, field_prefix, linestyle, linewidth)
ANGVEL_SERIES = (
    ("angVel (filt)", "angVel", "-", 1.4),
    ("angVelRaw (corr)", "angVelRaw", "-", 1.0),
    ("angVelUncorr", "angVelUncorr", "--", 1.0),
    ("angVelObjRPY", "angVelObjRPY", ":", 1.6),
)
ACCEL_SERIES = (
    ("accel (filt)", "accel", "-", 1.4),
    ("accelRaw", "accelRaw", "--", 1.0),
)
VEL_SERIES = (
    ("vel (published)", "vel", "-", 1.4),
    ("velRaw (published)", "velRaw", "--", 1.0),
    # Legacy: same v_world on attach-triangle axes (chatty vy).
    ("velTri (raw triangle body)", "velTri", ":", 1.2),
)
ANGACCEL_SERIES = (
    ("angAccel", "angAccel", "-", 1.2),
)

# Correlation pairs printed + scatter-plotted: (x_prefix, y_prefix, title)
CORR_PAIRS = (
    ("angVelUncorr", "angVelRaw", "Uncorr vs Corrected (M^-1)"),
    ("angVelRaw", "angVelObjRPY", "Corrected vs ObjRPY (engine)"),
    ("angVelUncorr", "angVelObjRPY", "Uncorr vs ObjRPY (engine)"),
    ("angVel", "angVelRaw", "Filtered vs Raw angVel"),
    ("vel", "velRaw", "Filtered vs Raw vel"),
    ("velRaw", "velTri", "Published velRaw vs raw-triangle velTri"),
    ("accel", "accelRaw", "Filtered vs Raw accel"),
)

# Attitude / frame figure (published vs triangle). Needs debug_raw + a non-triangle
# attitude_mode so Lua logs dir*Tri / velTri.
SHOW_ATTITUDE_FIG = True
# 25–35 Hz band share is printed for these vel prefixes (axis = y / lateral).
ATTITUDE_BAND_HZ = (25.0, 35.0)
ATTITUDE_VEL_COMPARE = ("velRaw", "velTri", "vel")

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
    for pref in ("dirY", "dirYTri"):
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
    """Published vs triangle frame: dirs, lateral vel, and vy spectra."""
    import matplotlib.pyplot as plt

    has_tri = all((f"dirYTri{_suffix(i)}") in data for i in range(3)) and all(
        (f"velTri{_suffix(i)}") in data for i in range(3)
    )
    if not has_tri:
        print("Attitude figure SKIP (need dirYTri_* and velTri_* from a non-triangle run)")
        return None

    idx = np.arange(t_rel.size)[:: max(1, int(PLOT_STRIDE))]
    t_p = t_rel[idx]
    dt = float(np.median(np.diff(t_rel))) if t_rel.size > 1 else 0.005
    f0, f1 = ATTITUDE_BAND_HZ

    dir_y = _get_vec(data, "dirY", mask)
    dir_y_tri = _get_vec(data, "dirYTri", mask)
    vel_raw = _get_vec(data, "velRaw", mask)
    vel_tri = _get_vec(data, "velTri", mask)
    try:
        vel_pub = _get_vec(data, "vel", mask)
    except KeyError:
        vel_pub = None

    fig, axes = plt.subplots(3, 3, figsize=(14, 9), dpi=DPI)
    fig.suptitle(
        "Attitude / frame — published vs triangle (flex check)",
        fontsize=12,
    )

    # Row 0: dirY world components
    world_names = ("dirY·êx", "dirY·êy", "dirY·êz")
    for col in range(3):
        ax = axes[0, col]
        ax.plot(t_p, dir_y[idx, col], "-", lw=1.2, label="dirY (published)")
        ax.plot(t_p, dir_y_tri[idx, col], "--", lw=1.0, label="dirYTri (triangle)")
        ax.set_title(world_names[col])
        ax.grid(True, alpha=0.3)
        if col == 0:
            ax.set_ylabel("direction [-]")
        ax.set_xlabel("t - t0 [s]")
    axes[0, 0].legend(loc="upper right", fontsize=8)

    # Row 1: body velocity (highlight y)
    for col, axis in enumerate(AXES):
        ax = axes[1, col]
        ax.plot(t_p, vel_raw[idx, axis], "-", lw=1.2, label="velRaw (new body)")
        ax.plot(t_p, vel_tri[idx, axis], "--", lw=1.0, label="velTri (old triangle body)")
        if vel_pub is not None:
            ax.plot(t_p, vel_pub[idx, axis], ":", lw=1.4, label="vel (new, filt)")
        ax.set_title(f"vel {AXIS_NAMES[axis]}")
        ax.grid(True, alpha=0.3)
        if col == 0:
            ax.set_ylabel("vel [m/s]")
        ax.set_xlabel("t - t0 [s]")
    axes[1, 0].legend(loc="upper right", fontsize=8)

    # Row 2: spectra of lateral velocity (+ dirY_x as the carrier)
    def _plot_psd(ax, series: np.ndarray, label: str, ls: str = "-") -> None:
        series = series - series.mean()
        freqs = np.fft.rfftfreq(series.size, dt)
        psd = (np.abs(np.fft.rfft(series)) ** 2) / max(series.size, 1)
        ax.semilogy(freqs, psd + 1e-30, ls, lw=1.1, label=label)

    ax = axes[2, 0]
    _plot_psd(ax, vel_raw[:, 1], "velRaw_y")
    _plot_psd(ax, vel_tri[:, 1], "velTri_y", "--")
    if vel_pub is not None:
        _plot_psd(ax, vel_pub[:, 1], "vel_y", ":")
    ax.axvspan(f0, f1, color="C3", alpha=0.15, label=f"{f0:.0f}-{f1:.0f} Hz")
    ax.set_xlim(0, min(50.0, 0.5 / dt))
    ax.set_title("PSD vel_y")
    ax.set_xlabel("f [Hz]")
    ax.set_ylabel("power")
    ax.grid(True, alpha=0.3, which="both")
    ax.legend(fontsize=7, loc="upper right")

    ax = axes[2, 1]
    _plot_psd(ax, dir_y[:, 0], "dirY_x (pub)")
    _plot_psd(ax, dir_y_tri[:, 0], "dirYTri_x", "--")
    ax.axvspan(f0, f1, color="C3", alpha=0.15, label=f"{f0:.0f}-{f1:.0f} Hz")
    ax.set_xlim(0, min(50.0, 0.5 / dt))
    ax.set_title("PSD dirY world-x (flex carrier)")
    ax.set_xlabel("f [Hz]")
    ax.grid(True, alpha=0.3, which="both")
    ax.legend(fontsize=7, loc="upper right")

    ax = axes[2, 2]
    # Same-run residual: triangle lateral minus published-frame lateral.
    dy = vel_tri[:, 1] - vel_raw[:, 1]
    ax.plot(t_p, dy[idx], "-", lw=1.0, color="C3")
    ax.set_title("velTri_y − velRaw_y (flex leak)")
    ax.set_xlabel("t - t0 [s]")
    ax.set_ylabel("[m/s]")
    ax.grid(True, alpha=0.3)
    share = _band_share(dy, dt, f0, f1)
    ax.text(
        0.02,
        0.95,
        f"band share={share:.2f}\nrms={np.sqrt(np.mean(dy**2)):.3f}",
        transform=ax.transAxes,
        va="top",
        fontsize=8,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    fig.tight_layout()
    return fig


def main() -> int:
    from bng_simulator.utils.logger_utils import load_run_data

    run_path = os.path.expanduser(RUN_PATH) if RUN_PATH else None
    print(
        f"Loading run: path={run_path!r} root={ROOT_DIR!r} number={RUN_NUMBER} "
        f"(pickle only)"
    )
    merged = load_run_data(
        run_number=None if run_path else RUN_NUMBER,
        run_path=run_path,
        root_dir=ROOT_DIR,
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
        "angVelUncorr",
        "angVelObjRPY",
        "velRaw",
        "velTri",
        "accelRaw",
        "dirY",
        "dirYTri",
    ):
        ok = all((pref + _suffix(i)) in data for i in range(3))
        print(f"  field {pref}_*: {'OK' if ok else 'MISSING'}")

    _print_corr_table(data, mask)
    _print_attitude_band_table(data, mask, t_rel)

    if not SHOW_PLOTS and not SAVE_FIG_PATH:
        return 0

    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, len(AXES), figsize=FIGSIZE, dpi=DPI, sharex=True)
    fig.suptitle(f"gtState debug — {SENSOR_KEY} @ {ROOT_DIR} run_{RUN_NUMBER:03d}", fontsize=12)
    _plot_group(axes[0], t_rel, data, mask, ANGVEL_SERIES, "angVel [rad/s]")
    _plot_group(axes[1], t_rel, data, mask, ANGACCEL_SERIES, "angAccel [rad/s²]")
    _plot_group(axes[2], t_rel, data, mask, VEL_SERIES, "vel [m/s]")
    _plot_group(axes[3], t_rel, data, mask, ACCEL_SERIES, "accel [m/s²]")
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
