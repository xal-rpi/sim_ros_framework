import os
import logging
import itertools
from dataclasses import dataclass, field
from typing import Dict, Any, List, Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from scipy.signal import find_peaks
from scipy.fft import fft, fftfreq


# ——— Configuration ——————————————————————————————————————————————————————
@dataclass
class Config:
    cycle_duration: float = 60.0
    durations: Dict[str, float] = field(
        default_factory=lambda: {
            "step": 15.0,
            "ramp": 15.0,
            "sine": 15.0,
            "chirp": 15.0,
        }
    )
    num_cycles: int = 2
    plots_dir: str = "torque_analysis_plots"
    base_max_torque: float = 2000.0
    max_speed_for_torque_scaling: float = 38.0
    transmission_efficiency: float = 0.9


# ——— Analyzer Class —————————————————————————————————————————————————————
class TorqueAnalyzer:
    def __init__(self, config: Config):
        self.cfg = config
        self.df: pd.DataFrame
        self.metrics: Dict[str, Any] = {}

    def load_data(self, csv_path: str) -> None:
        """Load CSV and trim to first N cycles."""
        if not os.path.isfile(csv_path):
            logging.error(f"File not found: {csv_path}")
            return
        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            logging.error(f"Failed to read CSV: {e}")
            return
        if df.empty:
            logging.warning(f"CSV is empty: {csv_path}")
            return

        max_t = self.cfg.num_cycles * self.cfg.cycle_duration
        df = df[df["sim_time"] <= max_t].copy()
        df["net_requested_torque"] = (
            df["requested_wheel_torque"] - df["requested_brake_torque"]
        )
        if df is None:
            logging.fatal("No DF")
            exit(1)
        self.df = df
        logging.info(f"Loaded {len(df)} rows up to {max_t}s.")

    def calculate_metrics(self) -> None:
        assert self.df is not None, "Data not loaded"
        self.metrics["torque_tracking"] = self._calc_torque_tracking()
        self.metrics["engine_performance"] = self._calc_engine_performance()
        self.metrics["vehicle_dynamics"] = self._calc_vehicle_dynamics()
        self.metrics["brake_performance"] = self._calc_brake_performance()
        self.metrics["segment_analysis"] = self._calc_segment_analysis()
        self.metrics["system_overall"] = self._calc_system_overall()
        self.metrics["controller_tuning"] = self._calc_controller_tuning()
        self.metrics["separated_controllers"] = self._calc_separated_controllers()
        logging.info("All metrics calculated.")

    def _calc_torque_tracking(self) -> Dict[str, float]:
        df = self.df
        err = df["net_requested_torque"] - df["actual_total_wheel_torque_sensors"]
        return {
            "mae": float(np.mean(np.abs(err))),
            "rmse": float(np.sqrt(np.mean(err**2))),
            "max_err": float(np.max(np.abs(err))),
            "corr_coef": float(
                np.corrcoef(
                    df["net_requested_torque"], df["actual_total_wheel_torque_sensors"]
                )[0, 1]
            ),
            "mean_err": float(np.mean(err)),
            "std_err": float(np.std(err)),
        }

    def _calc_engine_performance(self) -> Dict[str, float]:
        df = self.df
        return {
            "avg_rpm": float(np.mean(df["engine_rpm"])),
            "max_rpm": float(np.max(df["engine_rpm"])),
            "min_rpm": float(np.min(df["engine_rpm"])),
            "rpm_std": float(np.std(df["engine_rpm"])),
            "avg_throttle": float(np.mean(df["actual_engine_throttle"])),
            "max_throttle": float(np.max(df["actual_engine_throttle"])),
            "throttle_above_50pct": float(
                np.sum(df["actual_engine_throttle"] > 0.5) / len(df) * 100
            ),
            "avg_flywheel_torque": float(np.mean(df["actual_flywheel_torque"])),
            "max_flywheel_torque": float(np.max(df["actual_flywheel_torque"])),
        }

    def _calc_vehicle_dynamics(self) -> Dict[str, float]:
        df = self.df
        speed = df["actual_speed"]
        accel = np.diff(speed)
        return {
            "avg_speed": float(np.mean(speed)),
            "max_speed": float(np.max(speed)),
            "speed_std": float(np.std(speed)),
            "max_accel": float(np.max(accel)) if accel.size > 0 else 0.0,
            "max_decel": float(np.min(accel)) if accel.size > 0 else 0.0,
            "avg_gear_ratio": float(np.mean(df["current_gear_ratio"])),
        }

    def _calc_brake_performance(self) -> Dict[str, float]:
        df = self.df
        brake_active = df["requested_brake_torque"] > 0
        return {
            "usage_pct": float(np.sum(brake_active) / len(df) * 100),
            "avg_torque_active": float(
                df.loc[brake_active, "requested_brake_torque"].mean()
                if brake_active.any()
                else 0.0
            ),
            "max_brake_torque": float(np.max(df["requested_brake_torque"])),
            "simul_brake_wheel_pct": float(
                np.sum(
                    (df["requested_brake_torque"] > 0)
                    & (df["requested_wheel_torque"] > 0)
                )
                / len(df)
                * 100
            ),
        }

    def _calc_segment_analysis(self) -> Dict[str, Any]:
        df = self.df
        out: Dict[str, Any] = {}
        for cycle in range(self.cfg.num_cycles):
            t0 = cycle * self.cfg.cycle_duration
            for name, dur in self.cfg.durations.items():
                if dur <= 0:
                    continue
                seg_df = df[(df["sim_time"] >= t0) & (df["sim_time"] < t0 + dur)]
                if seg_df.empty:
                    t0 += dur
                    continue
                err = (
                    seg_df["net_requested_torque"]
                    - seg_df["actual_total_wheel_torque_sensors"]
                )
                key = f"cycle_{cycle}_{name}"
                out[key] = {
                    "rms_err": float(np.sqrt(np.mean(err**2))),
                    "max_err": float(np.max(np.abs(err))),
                    "avg_speed": float(np.mean(seg_df["actual_speed"])),
                    "avg_rpm": float(np.mean(seg_df["engine_rpm"])),
                    "avg_throttle": float(np.mean(seg_df["actual_engine_throttle"])),
                }
                t0 += dur
        return out

    def _calc_system_overall(self) -> Dict[str, float]:
        df = self.df
        energy = np.trapz(
            df["actual_total_wheel_torque_sensors"] * df["actual_speed"],
            df["sim_time"],
        )
        total_time = df["sim_time"].max() - df["sim_time"].min()
        return {
            "total_time": float(total_time),
            "data_points": int(len(df)),
            "avg_rate": float(len(df) / total_time) if total_time > 0 else 0.0,
            "energy": float(energy),
            "avg_wheel_angle": float(np.mean(df["requested_road_wheel_angle"])),
            "max_wheel_angle": float(np.max(np.abs(df["requested_road_wheel_angle"]))),
        }

    def _calc_controller_tuning(self) -> Dict[str, Any]:
        df = self.df
        err = df["net_requested_torque"] - df["actual_total_wheel_torque_sensors"]
        dt = float(np.mean(np.diff(df["sim_time"])))
        tuning: Dict[str, Any] = {}

        # === STEP RESPONSE (improved overshoot) ===
        step_thresh = 100.0
        dur_step = self.cfg.durations["step"]
        cycle_T = self.cfg.cycle_duration
        dt = float(np.mean(np.diff(df["sim_time"])))

        # find all big torque jumps
        torque_diff = np.abs(np.diff(df["net_requested_torque"]))
        candidate_idxs = np.where(torque_diff > step_thresh)[0]

        rise_times: List[float] = []
        overshoots: List[float] = []

        for idx in candidate_idxs:
            t0 = df["sim_time"].iat[idx]
            # only consider jumps that occur in the STEP portion of a cycle
            cycle = int(t0 // cycle_T)
            rel_t = t0 - cycle * cycle_T
            if rel_t > dur_step:
                continue

            # define the analysis window: from idx to idx+window_len
            window_len = min(int(0.2 / dt), len(df) - idx - 1)  # 0.2s
            req = df["net_requested_torque"].iloc[idx : idx + window_len].values
            resp = (
                df["actual_total_wheel_torque_sensors"]
                .iloc[idx : idx + window_len]
                .values
            )
            t_vec = df["sim_time"].iloc[idx : idx + window_len].values - t0

            # plateaus before/after jump
            plateau_n = min(5, max(1, len(req) // 10))
            T_init = float(np.mean(req[:plateau_n]))
            T_fin = float(np.mean(req[-plateau_n:]))
            deltaT = T_fin - T_init
            if abs(deltaT) < step_thresh:
                # skip small steps
                continue

            # find first cross of 10% and 90%
            crosses10 = np.where(
                (resp - T_init) * np.sign(deltaT) >= 0.1 * abs(deltaT)
            )[0]
            crosses90 = np.where(
                (resp - T_init) * np.sign(deltaT) >= 0.9 * abs(deltaT)
            )[0]
            if crosses10.size and crosses90.size:
                t10 = t_vec[crosses10[0]]
                t90 = t_vec[crosses90[0]]
                rise_times.append(t90 - t10)

            # OVERSHOOT: wait until resp ≥90% then look for peak in next 0.2s
            if crosses90.size:
                idx90 = crosses90[0]
                win_end = min(idx90 + int(0.2 / dt), len(resp))
                window_resp = resp[idx90:win_end]
                if deltaT > 0:
                    peak = window_resp.max()
                else:
                    peak = window_resp.min()
                overs_pct = (peak - T_fin) / deltaT * 100
                if overs_pct > 0:
                    overshoots.append(overs_pct)

            if len(rise_times) >= 3 and len(overshoots) >= 3:
                # only take first three of each
                break

        tuning["step_response"] = {}
        if rise_times:
            tuning["step_response"]["avg_rise_time"] = float(np.mean(rise_times))
            tuning["step_response"]["rise_time_std"] = float(np.std(rise_times))
        if overshoots:
            tuning["step_response"]["avg_overshoot"] = float(np.mean(overshoots))
            tuning["step_response"]["max_overshoot"] = float(np.max(overshoots))

        # — Control Effort —
        fly_rate = np.abs(np.diff(df["actual_flywheel_torque"]))
        thr_rate = np.abs(np.diff(df["actual_engine_throttle"]))
        tuning["control_effort"] = {
            "total_flywheel_var": float(np.sum(fly_rate)),
            "avg_flywheel_rate": float(np.mean(fly_rate)),
            "max_flywheel_rate": float(np.max(fly_rate)),
            "throttle_smoothness": float(np.mean(thr_rate)),
            "throttle_variation": float(np.sum(thr_rate)),
        }

        # — Frequency Response (Chirp) —
        chirp_err: List[float] = []
        for cycle in range(self.cfg.num_cycles):
            start = (
                cycle * self.cfg.cycle_duration
                + self.cfg.durations["step"]
                + self.cfg.durations["ramp"]
                + self.cfg.durations["sine"]
            )
            end = start + self.cfg.durations["chirp"]
            mask = (df["sim_time"] >= start) & (df["sim_time"] < end)
            chirp_err.extend(err.loc[mask].tolist())
        if len(chirp_err) > 10:
            F = fft(chirp_err)
            freqs = fftfreq(len(chirp_err), dt)
            mag = np.abs(F)
            cutoff = mag[0] / np.sqrt(2)
            idx = np.where(mag[: len(mag) // 2] < cutoff)[0]
            if idx.size > 0:
                tuning["frequency_response"] = {
                    "estimated_bandwidth_hz": float(freqs[idx[0]]),
                    "peak_error_frequency": float(
                        freqs[: len(mag) // 2][np.argmax(mag[: len(mag) // 2])]
                    ),
                }

        # — Tracking by Segment —
        tracking: Dict[str, Any] = {}
        for name, dur in self.cfg.durations.items():
            if dur <= 0:
                continue
            seg_errs: List[float] = []
            seg_delays: List[float] = []
            for cycle in range(self.cfg.num_cycles):
                start = cycle * self.cfg.cycle_duration + sum(
                    self.cfg.durations[s]
                    for s in list(self.cfg.durations)[
                        : list(self.cfg.durations).index(name)
                    ]
                )
                end = start + dur
                mask = (df["sim_time"] >= start) & (df["sim_time"] < end)
                req = df["net_requested_torque"][mask].values
                act = df["actual_total_wheel_torque_sensors"][mask].values
                if req.size == 0:
                    continue
                seg_errs.extend((req - act).tolist())
                if req.size > 5:
                    corr = np.correlate(
                        act - np.mean(act), req - np.mean(req), mode="full"
                    )
                    delay = (np.argmax(corr) - req.size + 1) * dt
                    seg_delays.append(delay)
            if seg_errs:
                tracking[name] = {
                    "rms_error": float(np.sqrt(np.mean(np.array(seg_errs) ** 2))),
                    "max_error": float(np.max(np.abs(seg_errs))),
                    "avg_delay": float(np.mean(seg_delays)) if seg_delays else 0.0,
                }
        tuning["tracking_by_segment"] = tracking

        # — Stability —
        peaks, _ = find_peaks(np.abs(err), height=np.std(err))
        total_t = df["sim_time"].max() - df["sim_time"].min()
        osc_rate = len(peaks) / total_t if total_t > 0 else 0.0
        consec = max(
            (
                len(list(g))
                for k, g in itertools.groupby(np.abs(err) > 2 * np.std(err))
                if k
            ),
            default=0,
        )
        tuning["stability"] = {
            "error_oscillation_rate": float(osc_rate),
            "max_consecutive_error": float(consec),
            "settling_performance": float(np.mean(np.abs(err[-20:]))),
        }

        return tuning

    def _calc_separated_controllers(self) -> Dict[str, Any]:
        """
        Calculate metrics separately for accel vs braking controllers,
        including improved overshoot calculation.
        """
        df = self.df
        out: Dict[str, Any] = {}

        for mode, mask in [
            ("acceleration", df["net_requested_torque"] > 0),
            ("braking", df["net_requested_torque"] < 0),
        ]:
            mdf = df[mask]
            if mdf.empty:
                continue

            # compute the common stats first
            err = mdf["net_requested_torque"] - mdf["actual_total_wheel_torque_sensors"]

            data: Dict[str, Any] = {
                "data_points": int(len(mdf)),
                "time_percentage": float(len(mdf) / len(df) * 100),
                "mean_absolute_error": float(np.mean(np.abs(err))),
                "root_mean_square_error": float(np.sqrt(np.mean(err**2))),
                "max_absolute_error": float(np.max(np.abs(err))),
                "correlation_coefficient": (
                    float(
                        np.corrcoef(
                            mdf["net_requested_torque"],
                            mdf["actual_total_wheel_torque_sensors"],
                        )[0, 1]
                    )
                    if len(mdf) > 1
                    else 0.0
                ),
                "mean_bias_error": float(np.mean(err)),
                "std_error": float(np.std(err)),
            }

            # STEP-RESPONSE OVERSHOOT (improved)
            torque_diff = np.abs(np.diff(mdf["net_requested_torque"]))
            thr = 50.0 if mode == "acceleration" else 100.0
            step_idxs = np.where(torque_diff > thr)[0]
            overs: List[float] = []
            dt = float(np.mean(np.diff(mdf["sim_time"])))
            dur_step = self.cfg.durations["step"]
            cycle_T = self.cfg.cycle_duration

            for idx in step_idxs:
                t0 = mdf["sim_time"].iat[idx]
                cycle = int(t0 // cycle_T)
                rel_t = t0 - cycle * cycle_T
                if rel_t > dur_step:
                    continue

                # 200 ms window for overshoot
                win_len = min(int(0.2 / dt), len(mdf) - idx - 1)
                window = mdf.iloc[idx : idx + win_len]
                req = window["net_requested_torque"].values
                resp = window["actual_total_wheel_torque_sensors"].values

                plateau_n = min(5, max(1, len(req) // 10))
                T_init = float(np.mean(req[:plateau_n]))
                T_fin = float(np.mean(req[-plateau_n:]))
                ΔT = T_fin - T_init

                if abs(ΔT) <= thr:
                    continue

                # detect when resp first crosses 90%
                target90 = T_init + 0.9 * ΔT
                crosses90 = np.where((resp - T_init) * np.sign(ΔT) >= 0.9 * abs(ΔT))[0]
                if not crosses90.size:
                    continue
                idx90 = crosses90[0]
                subwin = resp[idx90 : min(idx90 + int(0.2 / dt), len(resp))]
                peak = subwin.max() if ΔT > 0 else subwin.min()
                overs_pct = (peak - T_fin) / ΔT * 100
                if overs_pct > 0:
                    overs.append(overs_pct)

                if len(overs) >= 3:
                    break

            if overs:
                data["step_response"] = {
                    "avg_overshoot": float(np.mean(overs)),
                    "max_overshoot": float(np.max(overs)),
                }

            # finally insert this controller’s data into our result
            out[mode] = data

        return out

    def _draw_segments(self, ax: plt.Axes) -> None:
        """Draw vertical lines and labels for each segment and cycle."""
        x_max = self.cfg.num_cycles * self.cfg.cycle_duration
        ylim = ax.get_ylim()
        y_text = ylim[0] + 0.95 * (ylim[1] - ylim[0])
        # draw segment boundaries
        for cycle in range(self.cfg.num_cycles):
            t0 = cycle * self.cfg.cycle_duration
            for name, dur in self.cfg.durations.items():
                if dur <= 0:
                    continue
                mid = t0 + dur / 2
                if mid < x_max:
                    ax.text(
                        mid,
                        y_text,
                        name.capitalize(),
                        ha="center",
                        va="top",
                        fontsize=8,
                        bbox=dict(
                            boxstyle="round,pad=0.2", fc="white", ec="gray", lw=0.5
                        ),
                    )
                t0 += dur
                if t0 < x_max:
                    ax.axvline(t0, color="gray", linestyle="--", linewidth=0.7)
        # draw cycle boundaries
        for i in range(1, self.cfg.num_cycles + 1):
            x = i * self.cfg.cycle_duration
            if x < x_max:
                ax.axvline(x, color="black", linestyle="-.", linewidth=1)
        ax.set_ylim(ylim)

    def plot(self) -> None:
        """Generate a 4‐panel summary plot:
        1) Torque Requests
        2) Net Requested vs. Actual
        3) Engine Throttle
        4) Controller Tuning Stats (text)"""
        assert self.df is not None, "Data not loaded"
        os.makedirs(self.cfg.plots_dir, exist_ok=True)
        df = self.df

        plt.style.use("seaborn-v0_8-whitegrid")
        fig, axs = plt.subplots(3, 1, figsize=(24, 24))

        # 1) Torque Requests
        ax = axs[0]
        ax.plot(
            df["sim_time"],
            df["requested_wheel_torque"],
            label="Wheel torque",
            color="green",
        )
        ax.plot(
            df["sim_time"],
            df["requested_brake_torque"],
            label="Brake torque",
            color="red",
        )
        ax.set_ylabel("Torque (Nm)")
        ax.set_title("Torque Requests")
        self._draw_segments(ax)
        ax.legend(fontsize="small")

        # 2) Net Requested vs Actual (accel/brake)
        ax = axs[1]
        ax.plot(
            df["sim_time"],
            df["net_requested_torque"],
            label="Net Req",
            color="purple",
            alpha=0.8,
        )
        # scatter acceleration vs braking
        accel_mask = df["net_requested_torque"] > 0
        brake_mask = df["net_requested_torque"] < 0
        if accel_mask.any():
            ax.scatter(
                df["sim_time"][accel_mask],
                df["actual_total_wheel_torque_sensors"][accel_mask],
                label="Actual (Accel)",
                color="green",
                s=1,
                alpha=0.7,
            )
        if brake_mask.any():
            ax.scatter(
                df["sim_time"][brake_mask],
                df["actual_total_wheel_torque_sensors"][brake_mask],
                label="Actual (Brake)",
                color="red",
                s=1,
                alpha=0.7,
            )
        ax.set_ylabel("Torque (Nm)")
        ax.set_title("Net Requested vs Actual Wheel Torque")
        self._draw_segments(ax)
        ax.legend(fontsize="small")

        # 3) Engine Throttle
        ax = axs[2]
        ax.plot(
            df["sim_time"],
            df["actual_engine_throttle"],
            label="Throttle",
            color="sienna",
        )
        ax.set_ylabel("Throttle")
        ax.yaxis.set_major_formatter(ticker.PercentFormatter(xmax=1.0))
        ax.set_title("Engine Throttle")
        self._draw_segments(ax)
        ax.legend(fontsize="small")

        fig.savefig(
            cfg.plots_dir + "/llc_torque_perf.png", dpi=300, bbox_inches="tight"
        )

    def print_summary(self) -> None:
        """
        Log a formatted summary of performance metrics and detailed controller tuning analysis.
        """
        m = self.metrics

        # — TORQUE TRACKING —
        tt = m.get("torque_tracking", {})
        logging.info("=== TORQUE TRACKING ===")
        logging.info("Mean Absolute Error:    %.2f Nm", tt.get("mae", float("nan")))
        logging.info("RMS Error:              %.2f Nm", tt.get("rmse", float("nan")))
        logging.info("Max Absolute Error:     %.2f Nm", tt.get("max_err", float("nan")))
        logging.info("Correlation Coefficient:%.3f", tt.get("corr_coef", float("nan")))

        # — ENGINE PERFORMANCE —
        ep = m.get("engine_performance", {})
        logging.info("=== ENGINE PERFORMANCE ===")
        logging.info(
            "RPM Range:            %.0f – %.0f  (sigma=%.1f)",
            ep.get("min_rpm", 0.0),
            ep.get("max_rpm", 0.0),
            ep.get("rpm_std", 0.0),
        )
        logging.info("Average Throttle:     %.1f%%", ep.get("avg_throttle", 0.0) * 100)
        logging.info(
            "Throttle >50%% Usage:  %.1f%%", ep.get("throttle_above_50pct", 0.0)
        )
        logging.info(
            "Flywheel Torque avg/max: %.2f / %.2f Nm",
            ep.get("avg_flywheel_torque", 0.0),
            ep.get("max_flywheel_torque", 0.0),
        )

        # — VEHICLE DYNAMICS —
        vd = m.get("vehicle_dynamics", {})
        logging.info("=== VEHICLE DYNAMICS ===")
        logging.info(
            "Speed avg/max/std:    %.2f / %.2f / %.2f m/s",
            vd.get("avg_speed", 0.0),
            vd.get("max_speed", 0.0),
            vd.get("speed_std", 0.0),
        )
        logging.info(
            "Max accel/decel:      %.2f / %.2f m/s²",
            vd.get("max_accel", 0.0),
            abs(vd.get("max_decel", 0.0)),
        )

        # — BRAKE PERFORMANCE —
        bp = m.get("brake_performance", {})
        logging.info("=== BRAKE PERFORMANCE ===")
        logging.info("Brake usage:          %.1f%% of time", bp.get("usage_pct", 0.0))
        logging.info("Max Brake Torque:     %.2f Nm", bp.get("max_brake_torque", 0.0))
        logging.info(
            "Simultaneous Brake/Wheel: %.1f%%", bp.get("simul_brake_wheel_pct", 0.0)
        )

        # — SYSTEM OVERALL —
        so = m.get("system_overall", {})
        logging.info("=== SYSTEM OVERALL ===")
        logging.info("Sim Time:             %.2f s", so.get("total_time", 0.0))
        logging.info("Data Points:          %d", so.get("data_points", 0))
        logging.info("Avg Sampling Rate:    %.1f Hz", so.get("avg_rate", 0.0))
        logging.info("Estimated Energy:     %.2f J", so.get("energy", 0.0))

        # — CONTROLLER TUNING ANALYSIS —
        ct = m.get("controller_tuning", {})
        if not ct:
            logging.info("No controller tuning data available.")
            return

        logging.info("=== CONTROLLER TUNING ===")

        # 1) Step Response
        sr = ct.get("step_response", {})
        if sr:
            art = sr.get("avg_rise_time")
            std_rt = sr.get("rise_time_std")
            if art is not None:
                logging.info("Average Rise Time:    %.3f s (sigma=%.3f s)", art, std_rt)
                if art > 0.5:
                    logging.warning(
                        "Rise time is above 0.5 s; consider increasing controller bandwidth or gains."
                    )
            sr = m["controller_tuning"].get("step_response", {})
            if sr:
                art = sr.get("avg_rise_time")
                std_rt = sr.get("rise_time_std")
                if art is not None:
                    logging.info(
                        "Average Rise Time:      %.3f s (σ=%.3f s)", art, std_rt
                    )

                # Updated overshoot printing
                avg_ov = sr.get("avg_overshoot")
                max_ov = sr.get("max_overshoot")
                if avg_ov is not None:
                    logging.info("Average Overshoot:      %.1f%%", avg_ov)
                    if avg_ov > 10.0:
                        logging.warning(
                            "Average overshoot exceeds 10%%; consider adding damping or reducing aggressiveness."
                        )
                if max_ov is not None:
                    logging.info("Maximum Overshoot:      %.1f%%", max_ov)

        # 2) Control Effort & Smoothness
        ce = ct.get("control_effort", {})
        logging.info(
            "Total Flywheel Variation: %.2f", ce.get("total_flywheel_var", 0.0)
        )
        logging.info("Average Flywheel Rate:    %.2f", ce.get("avg_flywheel_rate", 0.0))
        tsmooth = ce.get("throttle_smoothness", 0.0)
        logging.info("Throttle Smoothness:      %.4f", tsmooth)
        if tsmooth > 0.1:
            logging.warning(
                "Throttle commands are jerky (smoothness > 0.1); consider filtering or tuning."
            )

        # 3) Frequency Response
        fr = ct.get("frequency_response", {})
        if fr:
            bw = fr.get("estimated_bandwidth_hz")
            pf = fr.get("peak_error_frequency")
            logging.info("Estimated Bandwidth:      %.2f Hz", bw)
            logging.info("Peak Error Frequency:     %.2f Hz", pf)
            if bw is not None and bw < 1.0:
                logging.warning(
                    "Bandwidth below 1 Hz; may struggle with faster transients."
                )

        # 4) Tracking by Segment
        tb = ct.get("tracking_by_segment", {})
        for seg, val in tb.items():
            logging.info(
                "%s Segment: RMS=%.2f Nm, Max=%.2f Nm, Avg Delay=%.3f s",
                seg.capitalize(),
                val.get("rms_error", 0.0),
                val.get("max_error", 0.0),
                val.get("avg_delay", 0.0),
            )
            if val.get("avg_delay", 0.0) > 0.1:
                logging.warning(
                    "%s segment has high tracking delay (>0.1 s).", seg.capitalize()
                )

        # 5) Stability Indicators
        st = ct.get("stability", {})
        logging.info(
            "Error Oscillation Rate:   %.2f peaks/s",
            st.get("error_oscillation_rate", 0.0),
        )
        mce = int(st.get("max_consecutive_error", 0))
        logging.info("Max Consecutive Error:    %d samples", mce)
        if st.get("error_oscillation_rate", 0.0) > 1.0:
            logging.warning(
                "High oscillation rate detected; check for limit cycling or under-damping."
            )
        sp = st.get("settling_performance", 0.0)
        logging.info(
            "Settling Performance:     %.2f Nm (mean abs error over last 20 samples)",
            sp,
        )
        if sp > 5.0:
            logging.warning(
                "Steady-state error is above 5 Nm; consider lowering steady-state gain."
            )


# ——— Main Execution —————————————————————————————————————————————————————
if __name__ == "__main__":
    cfg = Config()
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)

    fmt = logging.Formatter(
        "%(levelname)-8s: %(message)s",
    )

    # file handler (overwrites on each run)
    fh = logging.FileHandler(cfg.plots_dir + "/res.txt", mode="w")
    fh.setLevel(logging.INFO)
    fh.setFormatter(fmt)
    logger.addHandler(fh)

    # console handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(fmt)
    logger.addHandler(ch)
    analyzer = TorqueAnalyzer(cfg)
    analyzer.load_data("wheel_torque_analysis_log.csv")
    if analyzer.df is not None:
        analyzer.calculate_metrics()
        analyzer.plot()
        analyzer.print_summary()
