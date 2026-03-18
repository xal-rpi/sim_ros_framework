#!/usr/bin/env python3
"""lateral_id.py

CLI tool to send *steering* excitation signals for lateral dynamics identification
in the (approximately) linear, low-slip regime.

This is intentionally similar in spirit to `core/torque_map.py`, but it:
- Commands steering (normalized LLC input in [-1, 1])
- Optionally holds a target vehicle speed using the low-level controller
  `vehicle_speed` target (m/s)
- Provides conservative safety gates based on the reduced GT state:
  lateral accel `accel_y`, body slip `beta`, yaw rate `r`, and (optionally)
  longitudinal accel `accel_x`

Steering units:
- BeamNG LLC `steering` expects a *normalized* value in [-1, 1]
- The reduced state includes `delta` (road-wheel angle) in radians

Recommended runs (full-ish linear ID suite across speeds):

  # Conservative gates; tune to your setup.
  # ay_max ~ 2g, beta_max ~ 5 deg, yaw_max ~ 0.6 rad/s.

  # One-command suite (runs multiple blocks at each speed)
  python lateral_id.py suite --speeds 5,10,15 --wheelbase 2.7 --ay-target 1.5 \
    --steer-max-rad 0.69 --speed-settle 5 --dt 0.02 --warmup 3 --cooldown 3 \
    --ramp-up 0.3 --ramp-down 0.3 --gates --ay-max 1.8 --beta-max 0.09 --yaw-max 0.6

  # If you prefer explicit runs per speed (multisine + chirp + step validation)
  python lateral_id.py multisine --speed 5  --wheelbase 2.7 --ay-target 1.5 --freqs 0.2,0.5,1.0 --dur 70 --dt 0.02 --gates --ay-max 1.8 --beta-max 0.09
  python lateral_id.py chirp     --speed 5  --wheelbase 2.7 --ay-target 1.5 --f0 0.2 --f1 1.5 --dur 40 --dt 0.02 --gates --ay-max 1.8 --beta-max 0.09
  python lateral_id.py step      --speed 5  --wheelbase 2.7 --ay-target 1.2 --hold 2.0 --steps 6 --dt 0.02 --gates --ay-max 1.5 --beta-max 0.08

  python lateral_id.py multisine --speed 10 --wheelbase 2.7 --ay-target 1.5 --freqs 0.2,0.6,1.2 --dur 70 --dt 0.02 --gates --ay-max 1.8 --beta-max 0.09
  python lateral_id.py chirp     --speed 10 --wheelbase 2.7 --ay-target 1.5 --f0 0.2 --f1 1.7 --dur 40 --dt 0.02 --gates --ay-max 1.8 --beta-max 0.09
  python lateral_id.py step      --speed 10 --wheelbase 2.7 --ay-target 1.0 --hold 2.0 --steps 6 --dt 0.02 --gates --ay-max 1.4 --beta-max 0.08

  python lateral_id.py multisine --speed 15 --wheelbase 2.7 --ay-target 1.5 --freqs 0.2,0.7,1.4 --dur 70 --dt 0.02 --gates --ay-max 1.8 --beta-max 0.09
  python lateral_id.py chirp     --speed 15 --wheelbase 2.7 --ay-target 1.3 --f0 0.2 --f1 1.7 --dur 40 --dt 0.02 --gates --ay-max 1.6 --beta-max 0.08
  python lateral_id.py step      --speed 15 --wheelbase 2.7 --ay-target 0.9 --hold 2.0 --steps 6 --dt 0.02 --gates --ay-max 1.3 --beta-max 0.07

Notes:
- Multisine is usually the best “workhorse” for parameter fitting.
- Chirp provides a nice validation dataset (broadband, easy to visualize).
- Step-train is excellent for sanity checks / model validation.

"""

from __future__ import annotations

import argparse
import math
import random
import signal
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

from bng_controller import TorqueSpeedController


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def parse_csv_floats(s: str) -> List[float]:
    s = (s or "").strip()
    if not s:
        return []
    return [float(x) for x in s.split(",")]


@dataclass
class Gates:
    enabled: bool = False

    v_min: Optional[float] = None
    v_max: Optional[float] = None

    ay_max: Optional[float] = None
    ax_max: Optional[float] = None

    yaw_rate_max: Optional[float] = None
    beta_max: Optional[float] = None

    delta_max_rad: Optional[float] = None
    state_age_max: Optional[float] = 0.5

    def check(self, state: Optional[object], state_age_sec: Optional[float]) -> Optional[str]:
        if not self.enabled:
            return None

        if self.state_age_max is not None and state_age_sec is not None:
            if state_age_sec > self.state_age_max:
                return f"state_stale ({state_age_sec:.3f}s > {self.state_age_max:.3f}s)"

        if state is None:
            return "state_missing"

        # ReducedGtStateMsg fields (see TorqueSpeedController.get_latest_state_dict)
        try:
            vx = float(state.vx)
            ay = float(state.accel_y)
            ax = float(state.accel_x)
            r = float(state.r)
            beta = float(state.beta)
            delta = float(state.delta)
        except Exception:
            return "state_parse_error"

        if self.v_min is not None and vx < self.v_min:
            return f"speed_below_min ({vx:.2f} < {self.v_min:.2f})"
        if self.v_max is not None and vx > self.v_max:
            return f"speed_above_max ({vx:.2f} > {self.v_max:.2f})"

        if self.ay_max is not None and abs(ay) > self.ay_max:
            return f"ay_limit (|{ay:.2f}| > {self.ay_max:.2f})"
        if self.ax_max is not None and abs(ax) > self.ax_max:
            return f"ax_limit (|{ax:.2f}| > {self.ax_max:.2f})"

        if self.yaw_rate_max is not None and abs(r) > self.yaw_rate_max:
            return f"yaw_rate_limit (|{r:.3f}| > {self.yaw_rate_max:.3f})"
        if self.beta_max is not None and abs(beta) > self.beta_max:
            return f"beta_limit (|{beta:.3f}| > {self.beta_max:.3f})"

        if self.delta_max_rad is not None and abs(delta) > self.delta_max_rad:
            return f"delta_limit (|{delta:.3f}| > {self.delta_max_rad:.3f})"

        return None


class Controller:
    def __init__(
        self,
        *,
        vehicle_name: str,
        subscribe_reduced_state: bool,
        spin_in_thread: bool,
        publish_commands: bool,
    ):
        self._ctl = TorqueSpeedController(
            vehicle_name=vehicle_name,
            subscribe_reduced_state=subscribe_reduced_state,
            spin_in_thread=spin_in_thread,
            publish_commands=publish_commands,
        )
        # Sleep a bit to allow the controller to initialize and receive the first state message.
        time.sleep(1.0)

    def send(self, *, speed_target_ms: Optional[float], steering_norm: float) -> None:
        # steering_norm is normalized LLC input in [-1, 1]
        steering_norm = clamp(steering_norm, -1.0, 1.0)
        # self._ctl.send_command(vehicle_speed=speed_target_ms, steering=steering_norm)
        self._ctl.send_command(vehicle_speed=speed_target_ms, steering=None)

    def get_state_msg(self):
        return self._ctl.get_latest_state_msg()

    def get_state_age_sec(self) -> Optional[float]:
        return self._ctl.get_state_age_sec()


class SteerPlayer:
    def __init__(
        self,
        ctl: Controller,
        *,
        dt: float,
        warmup: float,
        cooldown: float,
        speed_target_ms: Optional[float],
        gates: Gates,
        steering_limit_norm: float,
    ):
        self.ctl = ctl
        self.dt = float(dt)
        self.warmup = float(warmup)
        self.cooldown = float(cooldown)
        self.speed_target_ms = speed_target_ms
        self.gates = gates
        self.steering_limit_norm = float(steering_limit_norm)
        self._stop = False

    def request_stop(self) -> None:
        self._stop = True

    def _sleep_step(self, t_next_wall: float) -> None:
        while True:
            now = time.time()
            if now >= t_next_wall or self._stop:
                break
            time.sleep(min(0.02, t_next_wall - now))

    def _apply_gates(self) -> None:
        # print("Get latest state: ", self.ctl.get_state_msg())
        reason = self.gates.check(
            self.ctl.get_state_msg(), self.ctl.get_state_age_sec()
        )
        if reason is not None:
            raise RuntimeError(f"Gate violated: {reason}")

    def _send(self, steer_norm: float) -> None:
        steer_norm = clamp(
            steer_norm, -self.steering_limit_norm, self.steering_limit_norm
        )
        self.ctl.send(
            speed_target_ms=self.speed_target_ms,
            steering_norm=steer_norm
        )
        if self.speed_target_ms is None:
            print(f"[lateral_id] cmd steering={steer_norm:+.3f}")
        else:
            print(f"[lateral_id] cmd vref={self.speed_target_ms:.2f} m/s, steering={steer_norm:+.3f}")

    def hold(self, *, steer_norm: float, duration: float) -> None:
        print(f"[lateral_id] hold: steer={steer_norm:+.3f} for {duration:.2f}s")
        t0 = time.time()
        while not self._stop:
            if (time.time() - t0) >= duration:
                break
            self._send(steer_norm)
            self._apply_gates()
            self._sleep_step(time.time() + self.dt)

    def play(self, program: Sequence[Tuple[float, float]], *, name: str) -> None:
        program_list = list(program)
        program_list.sort(key=lambda x: x[0])

        print(f"[lateral_id] warmup {self.warmup:.2f}s")
        self.hold(steer_norm=0.0, duration=self.warmup)

        t0 = time.time()
        print(f"[lateral_id] program start ({name})")

        for i, (t_rel, steer_norm) in enumerate(program_list):
            if self._stop:
                break

            self._sleep_step(t0 + t_rel)
            print(f"[lateral_id] t={t_rel:.2f}s -> steer={steer_norm:+.3f}")

            t_next = program_list[i + 1][0] if (i + 1) < len(program_list) else (t_rel + self.dt)
            while not self._stop:
                now_rel = time.time() - t0
                if now_rel >= t_next:
                    break
                self._send(steer_norm)
                self._apply_gates()
                self._sleep_step(time.time() + self.dt)

        print(f"[lateral_id] cooldown {self.cooldown:.2f}s")
        self.hold(steer_norm=0.0, duration=self.cooldown)


# ---------------------------
# Program generators (time, steering)
# ---------------------------

def prog_step_train(amp: float, hold: float, n_steps: int, *, t_start: float = 0.0) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    t = float(t_start)
    pts.append((t, 0.0))
    for k in range(n_steps):
        sign = 1.0 if (k % 2 == 0) else -1.0
        t += hold
        pts.append((t, sign * amp))
        t += hold
        pts.append((t, 0.0))
    return pts


def prog_prbs(amp: float, bit_time: float, n_bits: int, *, seed: int, t_start: float = 0.0) -> List[Tuple[float, float]]:
    rng = random.Random(int(seed))
    pts: List[Tuple[float, float]] = []
    t = float(t_start)
    pts.append((t, 0.0))
    for _ in range(int(n_bits)):
        t += float(bit_time)
        pts.append((t, (amp if rng.random() > 0.5 else -amp)))
    return pts


def prog_multisine(amp: float, freqs_hz: Sequence[float], duration: float, sample_dt: float, *, t_start: float = 0.0) -> List[Tuple[float, float]]:
    freqs = [float(f) for f in freqs_hz if float(f) > 0.0]
    if not freqs:
        return [(float(t_start), 0.0), (float(t_start) + float(duration), 0.0)]

    pts: List[Tuple[float, float]] = []
    n = max(1, int(float(duration) / float(sample_dt)))
    for k in range(n + 1):
        t = float(t_start) + k * float(sample_dt)
        s = 0.0
        for f in freqs:
            s += math.sin(2.0 * math.pi * f * (t - float(t_start)))
        s /= float(len(freqs))
        pts.append((t, amp * s))
    return pts


def prog_chirp_sine(amp: float, f0_hz: float, f1_hz: float, duration: float, sample_dt: float, *, t_start: float = 0.0) -> List[Tuple[float, float]]:
    # Linear frequency chirp: f(t)=f0 + (f1-f0)*t/T, phase=2π∫f dt
    T = float(duration)
    f0 = float(f0_hz)
    f1 = float(f1_hz)
    if T <= 0.0:
        return [(float(t_start), 0.0)]

    pts: List[Tuple[float, float]] = []
    n = max(1, int(T / float(sample_dt)))
    for k in range(n + 1):
        tau = k * float(sample_dt)
        tau = min(tau, T)
        phase = 2.0 * math.pi * (f0 * tau + 0.5 * (f1 - f0) * (tau * tau) / T)
        pts.append((float(t_start) + tau, amp * math.sin(phase)))
    return pts


def _ramp_segment(t0: float, u0: float, u1: float, duration: float, *, min_points: int = 2) -> List[Tuple[float, float]]:
    if duration <= 0.0:
        return [(t0, u1)]
    n = max(min_points, int(math.ceil(duration / 0.02)))
    pts: List[Tuple[float, float]] = []
    for i in range(1, n + 1):
        frac = i / n
        t = t0 + frac * duration
        u = u0 + (u1 - u0) * frac
        pts.append((t, u))
    return pts


def apply_ramps(program: List[Tuple[float, float]], ramp_up: float, ramp_down: float) -> List[Tuple[float, float]]:
    if not program:
        return []

    program = sorted(program, key=lambda x: x[0])
    out: List[Tuple[float, float]] = []

    last_t = program[0][0]
    last_u = 0.0

    for t, u in program:
        if t < last_t:
            continue

        if u != last_u:
            ramp = ramp_down if abs(u) < abs(last_u) else ramp_up
            ramp_start_t = max(last_t, t - ramp)
            out.extend(_ramp_segment(ramp_start_t, last_u, u, ramp))
        else:
            out.append((t, u))

        last_u = u
        last_t = t

    # End at zero (ramp-down)
    end_t = out[-1][0] if out else program[-1][0]
    if last_u != 0.0:
        out.extend(_ramp_segment(end_t, last_u, 0.0, ramp_down))

    return out


# ---------------------------
# Amplitude selection helpers
# ---------------------------

def compute_delta_amp_rad_from_ay(*, ay_target: float, wheelbase_m: float, speed_ms: float) -> float:
    # Just pure kinematics and pure rolling (no slip): ay = v^2 / R, delta = atan(wheelbase / R) ~ wheelbase / R for small angles.
    v = max(0.5, float(speed_ms))
    return float(ay_target) * float(wheelbase_m) / (v * v)


def rad_to_norm(delta_rad: float, steer_max_rad: float) -> float:
    if steer_max_rad <= 0.0:
        raise ValueError("--steer-max-rad must be > 0")
    return float(delta_rad) / float(steer_max_rad)


def pick_steer_amp_norm(
    *,
    delta_amp: Optional[float],
    delta_amp_rad: Optional[float],
    ay_target: Optional[float],
    wheelbase_m: Optional[float],
    speed_ms: float,
    steer_max_rad: float,
    delta_amp_norm_max: float,
) -> float:
    if delta_amp is not None:
        amp = float(delta_amp)
    elif delta_amp_rad is not None:
        amp = rad_to_norm(float(delta_amp_rad), steer_max_rad)
    elif ay_target is not None and wheelbase_m is not None:
        amp_rad = compute_delta_amp_rad_from_ay(
            ay_target=float(ay_target), wheelbase_m=float(wheelbase_m), speed_ms=float(speed_ms)
        )
        amp = rad_to_norm(amp_rad, steer_max_rad)
    else:
        raise ValueError(
            "Provide one of: --delta-amp, --delta-amp-rad, or (--ay-target and --wheelbase)."
        )

    amp = abs(amp)
    return clamp(amp, 0.0, float(delta_amp_norm_max))


# ---------------------------
# CLI
# ---------------------------

def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--vehicle-name", type=str, default="EGO")

    parser.add_argument("--dt", type=float, default=0.02, help="Command update period [s].")
    parser.add_argument("--warmup", type=float, default=2.0, help="Warmup duration at zero steering [s].")
    parser.add_argument("--cooldown", type=float, default=2.0, help="Cooldown duration at zero steering [s].")

    parser.add_argument("--ramp-up", type=float, default=0.3, help="Ramp duration for increases in |steer| [s].")
    parser.add_argument("--ramp-down", type=float, default=0.3, help="Ramp duration for decreases in |steer| [s].")

    # Speed hold
    parser.add_argument("--speed", type=float, default=None, help="Target speed for this run [m/s].")

    # Steering magnitude selection
    parser.add_argument("--delta-amp", type=float, default=None, help="Steering amplitude (normalized) [0..1].")
    parser.add_argument("--delta-amp-rad", type=float, default=None, help="Steering amplitude (road wheel angle) [rad].")
    parser.add_argument("--steer-max-rad", type=float, default=0.69, help="Max road-wheel angle corresponding to steering=1.0 [rad].")
    parser.add_argument("--ay-target", type=float, default=None, help="Target lateral accel for amplitude scaling [m/s^2].")
    parser.add_argument("--wheelbase", type=float, default=None, help="Wheelbase for amplitude scaling [m].")

    parser.add_argument(
        "--amp-max",
        type=float,
        default=1.0,
        help="Max steering amplitude (normalized) allowed by this tool (extra safety clamp).",
    )

    # Gates
    parser.add_argument("--gates", action="store_true", help="Enable low-slip safety gates.")
    parser.add_argument("--v-min", type=float, default=None, help="Min speed gate [m/s].")
    parser.add_argument("--v-max", type=float, default=None, help="Max speed gate [m/s].")
    parser.add_argument("--ay-max", type=float, default=None, help="Max |accel_y| gate [m/s^2].")
    parser.add_argument("--ax-max", type=float, default=None, help="Max |accel_x| gate [m/s^2].")
    parser.add_argument("--yaw-max", type=float, default=None, help="Max |yaw_rate| gate [rad/s].")
    parser.add_argument("--beta-max", type=float, default=None, help="Max |beta| gate [rad].")
    parser.add_argument("--delta-max-rad", type=float, default=None, help="Max |delta| (measured) [rad].")
    parser.add_argument(
        "--state-age-max",
        type=float,
        default=3.0,
        help="Max allowed reduced_state age before abort [s].",
    )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Send steering excitation signals for linear lateral ID.")
    sub = p.add_subparsers(dest="mode", required=True)

    ps = sub.add_parser("step", help="Alternating step-train about zero steering.")
    add_common_args(ps)
    ps.add_argument("--hold", type=float, default=2.0, help="Hold duration per step half-cycle [s].")
    ps.add_argument("--steps", type=int, default=6, help="Number of +/- step cycles.")

    pp = sub.add_parser("prbs", help="PRBS (random +/-) about zero steering.")
    add_common_args(pp)
    pp.add_argument("--bit", type=float, default=0.5, help="Bit duration [s].")
    pp.add_argument("--bits", type=int, default=80, help="Number of bits.")
    pp.add_argument("--seed", type=int, default=1)

    pm = sub.add_parser("multisine", help="Sum-of-sines about zero steering.")
    add_common_args(pm)
    pm.add_argument("--freqs", type=str, required=True, help="Comma-separated Hz, e.g. 0.2,0.6,1.2")
    pm.add_argument("--dur", type=float, default=60.0, help="Duration [s].")

    pc = sub.add_parser("chirp", help="Sine chirp about zero steering.")
    add_common_args(pc)
    pc.add_argument("--f0", type=float, default=0.2, help="Start frequency [Hz].")
    pc.add_argument("--f1", type=float, default=1.7, help="End frequency [Hz].")
    pc.add_argument("--dur", type=float, default=40.0, help="Duration [s].")

    pq = sub.add_parser("suite", help="Run a default multi-speed identification suite.")
    add_common_args(pq)
    pq.add_argument("--speeds", type=str, required=True, help="Comma-separated target speeds [m/s], e.g. 5,10,15")
    pq.add_argument("--speed-settle", type=float, default=5.0, help="Settling time after changing speed target [s].")
    pq.add_argument("--preset", type=str, default="linear_low_slip", choices=["linear_low_slip"]) 

    return p


def make_controller(args: argparse.Namespace) -> Controller:
    import rclpy

    if not rclpy.ok():
        rclpy.init()

    return Controller(
        vehicle_name=str(args.vehicle_name),
        subscribe_reduced_state=True,
        spin_in_thread=True,
        publish_commands=False,
    )


def build_gates(args: argparse.Namespace) -> Gates:
    return Gates(
        enabled=bool(args.gates),
        v_min=args.v_min,
        v_max=args.v_max,
        ay_max=args.ay_max,
        ax_max=args.ax_max,
        yaw_rate_max=args.yaw_max,
        beta_max=args.beta_max,
        delta_max_rad=args.delta_max_rad,
        state_age_max=args.state_age_max,
    )


def run_one_program(
    *,
    ctl: Controller,
    dt: float,
    warmup: float,
    cooldown: float,
    ramp_up: float,
    ramp_down: float,
    speed_target_ms: Optional[float],
    amp_norm: float,
    amp_norm_max: float,
    gates: Gates,
    name: str,
    program: List[Tuple[float, float]],
) -> None:
    player = SteerPlayer(
        ctl,
        dt=dt,
        warmup=warmup,
        cooldown=cooldown,
        speed_target_ms=speed_target_ms,
        gates=gates,
        steering_limit_norm=float(amp_norm_max),
    )

    program = apply_ramps(program, ramp_up=float(ramp_up), ramp_down=float(ramp_down))
    print(f"[lateral_id] amp_norm={amp_norm:.4f} (clamped to <= {amp_norm_max:.3f})")
    player.play(program, name=name)


def main() -> int:
    args = build_parser().parse_args()

    # Ensure we can stop safely.
    ctl = make_controller(args)
    gates = build_gates(args)

    stop_flag = {"stop": False}

    def _safe_stop() -> None:
        stop_flag["stop"] = True
        try:
            ctl.send(speed_target_ms=args.speed, steering_norm=0.0)
        except Exception:
            pass

    def _handle_sig(_signum, _frame):
        _safe_stop()
        sys.exit(130)

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    def _amp_for_speed(speed_ms: float) -> float:
        return pick_steer_amp_norm(
            delta_amp=args.delta_amp,
            delta_amp_rad=args.delta_amp_rad,
            ay_target=args.ay_target,
            wheelbase_m=args.wheelbase,
            speed_ms=float(speed_ms),
            steer_max_rad=float(args.steer_max_rad),
            delta_amp_norm_max=float(args.amp_max),
        )

    try:
        if args.mode in ("step", "prbs", "multisine", "chirp"):
            if args.speed is None:
                raise ValueError("--speed is required for this mode (use suite for multiple speeds)")

            speed = float(args.speed)
            amp_norm = _amp_for_speed(speed)

            if args.mode == "step":
                program = prog_step_train(amp=amp_norm, hold=float(args.hold), n_steps=int(args.steps))
                name = f"step_v{speed:.1f}"
            elif args.mode == "prbs":
                program = prog_prbs(amp=amp_norm, bit_time=float(args.bit), n_bits=int(args.bits), seed=int(args.seed))
                name = f"prbs_v{speed:.1f}"
            elif args.mode == "multisine":
                freqs = parse_csv_floats(str(args.freqs))
                if not freqs:
                    raise ValueError("Provide at least one frequency in --freqs")
                program = prog_multisine(
                    amp=amp_norm,
                    freqs_hz=freqs,
                    duration=float(args.dur),
                    sample_dt=float(args.dt),
                )
                name = f"multisine_v{speed:.1f}"
            elif args.mode == "chirp":
                program = prog_chirp_sine(
                    amp=amp_norm,
                    f0_hz=float(args.f0),
                    f1_hz=float(args.f1),
                    duration=float(args.dur),
                    sample_dt=float(args.dt),
                )
                name = f"chirp_v{speed:.1f}"
            else:
                raise ValueError(f"Unknown mode: {args.mode}")

            print(f"[lateral_id] Running mode={args.mode} at speed={speed:.2f} m/s (Ctrl+C to stop).")
            run_one_program(
                ctl=ctl,
                dt=float(args.dt),
                warmup=float(args.warmup),
                cooldown=float(args.cooldown),
                ramp_up=float(args.ramp_up),
                ramp_down=float(args.ramp_down),
                speed_target_ms=speed,
                amp_norm=amp_norm,
                amp_norm_max=float(args.amp_max),
                gates=gates,
                name=name,
                program=program,
            )

        elif args.mode == "suite":
            speeds = parse_csv_floats(str(args.speeds))
            if not speeds:
                raise ValueError("Provide at least one speed in --speeds")

            preset = str(args.preset)
            if preset != "linear_low_slip":
                raise ValueError(f"Unknown preset: {preset}")

            # A compact suite per speed:
            # 1) multisine (fitting)
            # 2) chirp (validation)
            # 3) step-train (sanity/validation)
            freqs = [0.2, 0.6, 1.2]
            multisine_dur = 70.0
            chirp_dur = 40.0

            for v in speeds:
                if stop_flag["stop"]:
                    break

                v = float(v)
                amp_norm = _amp_for_speed(v)

                # settle at new speed target with zero steering
                settle = float(args.speed_settle)
                if settle > 0.0:
                    print(f"[lateral_id] settle at v={v:.2f} m/s for {settle:.1f}s")
                    settle_program = [(0.0, 0.0), (settle, 0.0)]
                    run_one_program(
                        ctl=ctl,
                        dt=float(args.dt),
                        warmup=0.0,
                        cooldown=0.0,
                        ramp_up=float(args.ramp_up),
                        ramp_down=float(args.ramp_down),
                        speed_target_ms=v,
                        amp_norm=amp_norm,
                        amp_norm_max=float(args.amp_max),
                        gates=gates,
                        name=f"settle_v{v:.1f}",
                        program=settle_program,
                    )

                # multisine
                run_one_program(
                    ctl=ctl,
                    dt=float(args.dt),
                    warmup=float(args.warmup),
                    cooldown=float(args.cooldown),
                    ramp_up=float(args.ramp_up),
                    ramp_down=float(args.ramp_down),
                    speed_target_ms=v,
                    amp_norm=amp_norm,
                    amp_norm_max=float(args.amp_max),
                    gates=gates,
                    name=f"multisine_v{v:.1f}",
                    program=prog_multisine(amp=amp_norm, freqs_hz=freqs, duration=multisine_dur, sample_dt=float(args.dt)),
                )

                # chirp
                run_one_program(
                    ctl=ctl,
                    dt=float(args.dt),
                    warmup=float(args.warmup),
                    cooldown=float(args.cooldown),
                    ramp_up=float(args.ramp_up),
                    ramp_down=float(args.ramp_down),
                    speed_target_ms=v,
                    amp_norm=amp_norm,
                    amp_norm_max=float(args.amp_max),
                    gates=gates,
                    name=f"chirp_v{v:.1f}",
                    program=prog_chirp_sine(amp=amp_norm, f0_hz=0.2, f1_hz=1.7, duration=chirp_dur, sample_dt=float(args.dt)),
                )

                # step-train
                run_one_program(
                    ctl=ctl,
                    dt=float(args.dt),
                    warmup=float(args.warmup),
                    cooldown=float(args.cooldown),
                    ramp_up=float(args.ramp_up),
                    ramp_down=float(args.ramp_down),
                    speed_target_ms=v,
                    amp_norm=amp_norm,
                    amp_norm_max=float(args.amp_max),
                    gates=gates,
                    name=f"step_v{v:.1f}",
                    program=prog_step_train(amp=amp_norm, hold=2.0, n_steps=6),
                )

        else:
            raise ValueError(f"Unknown mode: {args.mode}")

    finally:
        # Always return steering to zero (keep speed target if user requested)
        try:
            ctl.send(speed_target_ms=getattr(args, "speed", None), steering_norm=0.0)
        except Exception:
            pass
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    print("[lateral_id] Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
