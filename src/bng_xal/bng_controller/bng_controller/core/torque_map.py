#!/usr/bin/env python3
"""
throttle_sweep.py

CLI tool to send throttle excitation signals for wheel-torque identification.

Assumes you have a Helper class that provides:
  - helper.set_throttle(u: float)   # u in [0, 1]
Optionally (for safety gating):
  - helper.get_speed() -> float     # m/s
  - helper.get_yaw_rate() -> float  # rad/s
  - helper.get_ax() -> float        # m/s^2
  - helper.get_slip_est() -> float  # unitless (optional)

No logging here; integrate with your own logger elsewhere.


python torque_map.py throt_brake --dur_t 6.0 --dur_b 3.0 --dur_coast 3.0 --ramp_throt 0.6 --ramp_brake 0.5
  
Recommended runs (warmup 3-5s, post-coast after ramp blocks):
  python torque_map.py ramp --u0 0.08 --u1 0.22 --ramp-time 10 --hold 4 --pre-ramp 0.8 --post-coast 12 --warmup 4 --ramp-up 0.2 --ramp-down 0.2
  python torque_map.py ramp --u0 0.18 --u1 0.35 --ramp-time 10 --hold 4 --pre-ramp 0.8 --post-coast 12 --warmup 4 --ramp-up 0.2 --ramp-down 0.2
  python torque_map.py ramp --u0 0.30 --u1 0.50 --ramp-time 12 --hold 4 --pre-ramp 0.8 --post-coast 15 --warmup 4 --ramp-up 0.2 --ramp-down 0.2
  python torque_map.py step --u0 0.25 --du 0.05 --hold 2.0 --steps 8 --warmup 3 --ramp-up 0.2 --ramp-down 0.2
  python torque_map.py step --u0 0.40 --du 0.06 --hold 2.0 --steps 6 --warmup 3 --ramp-up 0.2 --ramp-down 0.2
  python torque_map.py burst --u 0.30 --burst 1.5 --coast 4.0 --cycles 10 --warmup 3 --ramp-up 0.2 --ramp-down 0.2
  python torque_map.py prbs --u0 0.28 --du 0.04 --bit 0.6 --bits 60 --warmup 3 --ramp-up 0.2 --ramp-down 0.2
  python torque_map.py multisine --u0 0.30 --du 0.03 --freqs 0.3,0.7,1.2 --dur 50 --warmup 3 --ramp-up 0.2 --ramp-down 0.2
  python torque_map.py ramp --u0 0.55 --u1 0.85 --ramp-time 10 --hold 5 --pre-ramp 1.0 --post-coast 18 --warmup 4 --ramp-up 0.2 --ramp-down 0.2
  python torque_map.py step --u0 0.75 --du 0.08 --hold 2.0 --steps 6 --warmup 3 --ramp-up 0.2 --ramp-down 0.2
"""

from __future__ import annotations

import argparse
import math
import random
import signal
import sys
import time
from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Tuple
from bng_controller import TorqueSpeedController

# ---------------------------
# Replace this with your real helper import/constructor.
# ---------------------------

class Helper:
    """Stub. Replace with your actual interface."""
    def set_throttle(self, u: float) -> None:
        # Send to your low-level system
        pass
    
    def set_command(self, u: dict) -> None:
        # Send low-level command in the form of a dict
        pass

    # Optional safety signals:
    def get_speed(self) -> float:
        raise NotImplementedError

    def get_yaw_rate(self) -> float:
        raise NotImplementedError

    def get_ax(self) -> float:
        raise NotImplementedError

    def get_slip_est(self) -> float:
        raise NotImplementedError

class Controller(Helper):
    def __init__(self):
        super().__init__()
        # Initialize your controller and any state here
        self._ctl = TorqueSpeedController(
            vehicle_name="EGO", subscribe_reduced_state=True,
            spin_in_thread=True, publish_commands=False
        )

    def set_throttle(self, u: float) -> None:
        self._ctl.send_command(throttle=u, steering=0)

    def set_command(self, u): 
        self._ctl.send_command(**u)

    def get_speed(self) -> float:
        state = self._ctl.get_latest_state_msg()
        if state is None:
            return 0.0
        return state.vx

    def get_yaw_rate(self) -> float:
        state = self._ctl.get_latest_state_msg()
        if state is None:
            return 0.0
        return state.r

    def get_ax(self) -> float:
        state = self._ctl.get_latest_state_msg()
        if state is None:
            return 0.0
        return state.accel_x

    def get_slip_est(self) -> float:
        state = self._ctl.get_latest_state_msg()
        if state is None:
            return 0.0
        vel_x = state.vx
        wr = state.wr
        slip_est = (wr - vel_x) / max(vel_x, 1.0)
        return slip_est


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


CommandValue = float | dict[str, float]
ProgramPoint = Tuple[float, CommandValue]
Program = List[ProgramPoint]


@dataclass
class Gates:
    enabled: bool = False
    v_min: Optional[float] = None
    v_max: Optional[float] = None
    yaw_rate_max: Optional[float] = None
    ax_max: Optional[float] = None
    slip_max: Optional[float] = None

    def check(self, helper: Helper) -> Optional[str]:
        if not self.enabled:
            return None

        # Each access is optional; if not implemented, we skip that check.
        def safe_get(fn_name: str):
            fn = getattr(helper, fn_name, None)
            if fn is None:
                return None
            try:
                return fn()
            except NotImplementedError:
                return None
            except Exception:
                return None

        v = safe_get("get_speed")
        if v is not None:
            if self.v_min is not None and v < self.v_min:
                return f"speed_below_min ({v:.2f} < {self.v_min:.2f})"
            if self.v_max is not None and v > self.v_max:
                return f"speed_above_max ({v:.2f} > {self.v_max:.2f})"

        r = safe_get("get_yaw_rate")
        if r is not None and self.yaw_rate_max is not None and abs(r) > self.yaw_rate_max:
            return f"yaw_rate_limit (|{r:.3f}| > {self.yaw_rate_max:.3f})"

        ax = safe_get("get_ax")
        if ax is not None and self.ax_max is not None and abs(ax) > self.ax_max:
            return f"ax_limit (|{ax:.2f}| > {self.ax_max:.2f})"

        slip = safe_get("get_slip_est")
        if slip is not None and self.slip_max is not None and abs(slip) > self.slip_max:
            return f"slip_limit (|{slip:.3f}| > {self.slip_max:.3f})"

        return None


class ThrottlePlayer:
    def __init__(self, helper: Helper, dt: float, u_min: float, u_max: float,
                 warmup: float, cooldown: float, gates: Gates):
        self.helper = helper
        self.dt = dt
        self.u_min = u_min
        self.u_max = u_max
        self.warmup = warmup
        self.cooldown = cooldown
        self.gates = gates
        self._stop = False

    def request_stop(self):
        self._stop = True

    def _send(self, u: CommandValue):
        if isinstance(u, dict):
            self.helper.set_command(u)
            # print(f"[throttle_sweep] cmd {u}")
        else:
            u_cmd = clamp(u, self.u_min, self.u_max)
            self.helper.set_throttle(u_cmd)
            # print(f"[throttle_sweep] cmd throttle={u_cmd:.3f}")

    def _sleep_step(self, t_next: float):
        # Sleep in small chunks so Ctrl+C responds quickly
        while True:
            now = time.time()
            if now >= t_next or self._stop:
                break
            time.sleep(min(0.02, t_next - now))

    def _apply_gates(self):
        reason = self.gates.check(self.helper)
        if reason is not None:
            raise RuntimeError(f"Gate violated: {reason}")

    def hold(self, u: float, duration: float):
        print(f"[throttle_sweep] hold: u={u:.3f} for {duration:.2f}s")
        t0 = time.time()
        while not self._stop:
            t = time.time() - t0
            if t >= duration:
                break
            self._send(u)
            self._apply_gates()
            self._sleep_step(time.time() + self.dt)

    def play(self, program: Iterable[ProgramPoint], name: str = ""):
        # # Warmup
        # print(f"[throttle_sweep] warmup {self.warmup:.2f}s")
        # self.hold(0.0, self.warmup)

        t0 = time.time()
        print(f"[throttle_sweep] program start ({name})")
        program_list = list(program)
        
        for i, (t_rel, u) in enumerate(program_list):
            if self._stop:
                break
            # Wait until target time
            self._sleep_step(t0 + t_rel)
            print(f"[throttle_sweep] t={t_rel:.2f}s -> u={u}")
            
            # Continuously send commands until next waypoint
            t_next = program_list[i + 1][0] if i + 1 < len(program_list) else t_rel + self.dt
            while not self._stop:
                now = time.time() - t0
                if now >= t_next:
                    break
                self._send(u)
                self._apply_gates()
                self._sleep_step(time.time() + self.dt)

        # Cooldown
        print(f"[throttle_sweep] cooldown {self.cooldown:.2f}s")
        self.hold(0.0, self.cooldown)


# ---------------------------
# Program generators
# ---------------------------

def prog_ramp_hold(u0: float, u1: float, ramp_time: float,
        hold_time: float, dt_points: float = 0.05, t_start: float = 0.0
) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    t = t_start
    pts.append((t, u0))

    n = max(2, int(ramp_time / dt_points))
    for i in range(1, n + 1):
        t = t_start + i * (ramp_time / n)
        u = u0 + (u1 - u0) * (i / n)
        pts.append((t, u))

    # Add hold start point
    if hold_time > 0.0:
        t = t_start + ramp_time
        pts.append((t, u1))
        # Add hold end point
        t = t_start + ramp_time + hold_time
        pts.append((t, u1))
    
    return pts


def prog_step_train(u0: float, du: float, hold: float,
    n_steps: int, t_start: float = 0.0
) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    t = t_start
    pts.append((t, u0))
    for _ in range(n_steps):
        t += hold
        pts.append((t, u0 + du))
        t += hold
        pts.append((t, u0))
    return pts


def prog_burst_coast(u_burst: float, burst_time: float, coast_time: float,
    cycles: int, u_coast: float = 0.0, t_start: float = 0.0
) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    t = t_start
    pts.append((t, u_coast))
    for _ in range(cycles):
        t += 0.01
        pts.append((t, u_burst))
        t += burst_time
        pts.append((t, u_coast))
        t += coast_time
        pts.append((t, u_coast))
    return pts


def prog_prbs(u0: float, du: float, bit_time: float, n_bits: int,
              seed: int = 0, t_start: float = 0.0) -> List[Tuple[float, float]]:
    rng = random.Random(seed)
    pts: List[Tuple[float, float]] = []
    t = t_start
    pts.append((t, u0))
    for _ in range(n_bits):
        t += bit_time
        pts.append((t, u0 + (du if rng.random() > 0.5 else -du)))
    return pts


def prog_multisine(u0: float, du: float, freqs_hz: List[float], duration: float,
                   sample_dt: float, t_start: float = 0.0) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    n = max(1, int(duration / sample_dt))
    for k in range(n + 1):
        t = t_start + k * sample_dt
        s = 0.0
        for f in freqs_hz:
            s += math.sin(2.0 * math.pi * f * (t - t_start))
        s /= max(1, len(freqs_hz))
        pts.append((t, u0 + du * s))
    return pts


def prog_hold(u: float, duration: float, t_start: float = 0.0) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    t = t_start
    pts.append((t, u))
    if duration > 0.0:
        t += duration
        pts.append((t, u))
    return pts


def chain_programs(*programs: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Concatenate programs by shifting later ones in time."""
    out: List[Tuple[float, float]] = []
    t_offset = 0.0
    for prog in programs:
        if not prog:
            continue
        # Find the maximum time in this program
        prog_max_t = max(t for t, u in prog)
        for t, u in prog:
            out.append((t + t_offset, u))
        # Shift next program by this program's duration (not cumulative end time)
        t_offset += prog_max_t
    return out


def _ramp_segment(t0: float, u0: float, u1: float, duration: float,
                  min_points: int = 2) -> List[Tuple[float, float]]:
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


def apply_ramps(program: List[Tuple[float, float]], ramp_up: float,
                ramp_down: float, u_zero: float = 0.0) -> List[Tuple[float, float]]:
    """Insert ramp transitions; downward changes (including to zero) use ramp_down."""
    if not program:
        return []

    program = sorted(program, key=lambda x: x[0])
    out: List[Tuple[float, float]] = []

    last_t = program[0][0]
    last_u = u_zero
    
    for t, u in program:
        if t < last_t:
            continue
        
        if u != last_u:
            # Determine ramp duration and insert ramp BEFORE target time
            ramp = ramp_down if u < last_u else ramp_up
            ramp_start_t = max(last_t, t - ramp)
            out.extend(_ramp_segment(ramp_start_t, last_u, u, ramp))
        else:
            # Same value, just add the point
            out.append((t, u))
        
        last_u = u
        last_t = t

    # Final ramp back to zero
    end_t = out[-1][0] if out else program[-1][0]
    if last_u != u_zero:
        out.extend(_ramp_segment(end_t, last_u, u_zero, ramp_down))
    return out


def suite_low_speed(post_coast: float = 0.0, pre_ramp: float = 0.0) -> List[Tuple[float, float]]:
    """
    A default "good coverage" suite:
    - burst/coast (separates loss vs drive)
    - ramps (quasi-static map + hysteresis)
    - small steps (lag)
    - PRBS (broadband dynamics)
    """
    p1 = prog_burst_coast(u_burst=0.25, burst_time=1.5, coast_time=3.0, cycles=12)
    p2_pre = prog_ramp_hold(u0=0.0, u1=0.15, ramp_time=pre_ramp, hold_time=0.0)
    p2 = prog_ramp_hold(u0=0.15, u1=0.35, ramp_time=8.0, hold_time=3.0, t_start=0.0)
    p2c = prog_hold(u=0.0, duration=post_coast)
    p3_pre = prog_ramp_hold(u0=0.0, u1=0.35, ramp_time=pre_ramp, hold_time=0.0)
    p3 = prog_ramp_hold(u0=0.35, u1=0.15, ramp_time=8.0, hold_time=3.0, t_start=0.0)
    p3c = prog_hold(u=0.0, duration=post_coast)
    p4 = prog_step_train(u0=0.22, du=0.03, hold=1.0, n_steps=10)
    p5 = prog_prbs(u0=0.22, du=0.03, bit_time=0.5, n_bits=80, seed=1)
    # add small gaps at zero between blocks by inserting holds as "ramp_hold" with same u
    gap = prog_ramp_hold(u0=0.0, u1=0.0, ramp_time=0.0, hold_time=2.0)
    return chain_programs(p1, gap, p2_pre, p2, p2c, gap, p3_pre, p3, p3c, gap, p4, gap, p5)

def prog_throt_brake(
    target_throttle: float, target_brake: float, init_throttle: float = 0,
    ramp_duration: float = 0.6, hold: float = 6.0, 
    brake_ramp: float = 0.5, brake_hold: float = 3.0,
    coast_hold: float = 3.0, dt_points: float = 0.05, t_start: float = 0.0
):
    pts: Program = []
    t = t_start
    pts.append((t, {"throttle": init_throttle, "brake": 0.0}))
    
    # First ramp to target throttle
    n_ramp = max(2, int(ramp_duration / dt_points))
    for i in range(1, n_ramp + 1):
        t = t_start + i * (ramp_duration / n_ramp)
        u = init_throttle + (target_throttle - init_throttle) * (i / n_ramp)
        pts.append((t, {"throttle": u, "brake": 0.0}))
    
    # Hold phase
    t_hold_start = t
    n_hold = max(1, int(hold / dt_points))
    for i in range(1, n_hold+1):
        t = t_hold_start + i * (hold / n_hold)
        pts.append(
            (t, {"throttle": target_throttle, "brake": 0.0})
        )
    
    # Coast phase. Ramp first to 0 then hold zero for coast_hold duration
    t_coast_start = t
    n_coast_ramp = max(2, int(ramp_duration / dt_points))
    for i in range(1, n_coast_ramp + 1):
        t = t_coast_start + i * (ramp_duration / n_coast_ramp)
        u = target_throttle * (1 - i / n_coast_ramp)
        pts.append((t, {"throttle": u, "brake": 0.0}))

    t_coast_hold_start = t
    n_coast_hold = max(1, int(coast_hold / dt_points))
    for i in range(1, n_coast_hold + 1):
        t = t_coast_hold_start + i * (coast_hold / n_coast_hold)
        pts.append((t, {"throttle": 0.0, "brake": 0.0}))
    
    # Brake phase. Ramp up to brake
    t_brake_start = t
    n_brake_ramp = max(2, int(brake_ramp / dt_points))
    for i in range(1, n_brake_ramp + 1):
        t = t_brake_start + i * (brake_ramp / n_brake_ramp)
        b = target_brake * (i / n_brake_ramp)
        pts.append((t, {"throttle": 0.0, "brake": b}))

    t_brake_hold_start = t
    n_brake_hold = max(1, int(brake_hold / dt_points))
    for i in range(1, n_brake_hold + 1):
        t = t_brake_hold_start + i * (brake_hold / n_brake_hold)
        pts.append((t, {"throttle": 0.0, "brake": target_brake}))
    
    # Ramp back to zero brake
    t_brake_release_start = t
    n_brake_release = max(2, int(brake_ramp / dt_points))
    for i in range(1, n_brake_release + 1):
        t = t_brake_release_start + i * (brake_ramp / n_brake_release)
        b = target_brake * (1 - i / n_brake_release)
        pts.append((t, {"throttle": 0.0, "brake": b}))
    
    return pts


def prog_throttle_oscillation(
    center_throttle: float,
    amplitude: float,
    frequency_hz: float,
    duration: float,
    ramp_duration: float = 0.6,
    dt_points: float = 0.05,
    t_start: float = 0.0,
) -> Program:
    pts: Program = []
    t = t_start
    current_throttle = 0.0
    pts.append((t, {"throttle": current_throttle, "brake": 0.0}))

    if duration <= 0.0:
        return pts

    if ramp_duration > 0.0:
        n_ramp = max(2, int(ramp_duration / dt_points))
        for i in range(1, n_ramp + 1):
            t = t_start + i * (ramp_duration / n_ramp)
            current_throttle = clamp(center_throttle * (i / n_ramp), 0.0, 1.0)
            pts.append((t, {"throttle": current_throttle, "brake": 0.0}))
    else:
        current_throttle = clamp(center_throttle, 0.0, 1.0)
        pts.append((t, {"throttle": current_throttle, "brake": 0.0}))

    t_osc_start = t
    n_osc = max(1, int(duration / dt_points))
    for i in range(1, n_osc + 1):
        t = t_osc_start + i * (duration / n_osc)
        current_throttle = clamp(
            center_throttle
            + amplitude * math.sin(2.0 * math.pi * frequency_hz * (t - t_osc_start)),
            0.0,
            1.0,
        )
        pts.append((t, {"throttle": current_throttle, "brake": 0.0}))

    if ramp_duration > 0.0:
        t_ramp_down_start = t
        n_ramp_down = max(2, int(ramp_duration / dt_points))
        start_throttle = current_throttle
        for i in range(1, n_ramp_down + 1):
            t = t_ramp_down_start + i * (ramp_duration / n_ramp_down)
            current_throttle = clamp(start_throttle * (1 - i / n_ramp_down), 0.0, 1.0)
            pts.append((t, {"throttle": current_throttle, "brake": 0.0}))

    return pts
    

def suite_throttle_brake_manual_steer(
    ramp_duration: float = 0.6, hold: float = 6.0,
    brake_ramp:float = 0.5, brake_hold: float = 3.0,
    coast_hold: float = 3.0, dt_points: float = 0.05,
    oscillate_throttle: Optional[float] = None,
    oscillate_amplitude: float = 0.05,
    oscillate_frequency: float = 0.5,
    oscillate_duration: float = 0.0,
) -> Program:
    """
    A suite for allowing brake and throttle to create
    proper data for longitidunal motion. Steering can be input
    by the user during the process.
    """
    throttle_target = [0.1, 0.2, 0.3, 0.4, 0.6, 0.8, 1.0]
    brake_target = [0.1, 0.2, 0.3, 0.4, 0.6, 0.8, 1.0]
    suite: Program = []
    t = 0.0
    for throt, brake in zip(throttle_target, brake_target):
        suite.extend(prog_throt_brake(
            target_throttle=throt, target_brake=brake, init_throttle=0.0,
            ramp_duration=ramp_duration, hold=hold, brake_ramp=brake_ramp,
            brake_hold=brake_hold, coast_hold=coast_hold, dt_points=dt_points,
            t_start=t
        ))
        # Update t to the end of this block
        if suite:
            t = suite[-1][0] + dt_points

    if (
        oscillate_throttle is not None
        and oscillate_duration > 0.0
        and oscillate_frequency > 0.0
        and oscillate_amplitude > 0.0
    ):
        suite.extend(
            prog_throttle_oscillation(
                center_throttle=oscillate_throttle,
                amplitude=oscillate_amplitude,
                frequency_hz=oscillate_frequency,
                duration=oscillate_duration,
                ramp_duration=ramp_duration,
                dt_points=dt_points,
                t_start=t,
            )
        )

    return suite

# ---------------------------
# CLI

def parse_freqs(s: str) -> List[float]:
    if not s.strip():
        return []
    return [float(x) for x in s.split(",")]

def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--dt", type=float, default=0.02, help="Command update period [s].")
    parser.add_argument("--u-min", type=float, default=0.0, help="Min throttle.")
    parser.add_argument("--u-max", type=float, default=1.0, help="Max throttle.")
    parser.add_argument("--warmup", type=float, default=1.0, help="Warmup duration at 0 throttle [s].")
    parser.add_argument("--cooldown", type=float, default=2.0, help="Cooldown duration at 0 throttle [s].")
    parser.add_argument(
        "--ramp-up",
        type=float,
        default=0.2,
        help="Ramp duration to reach each new command [s].",
    )
    parser.add_argument(
        "--ramp-down",
        type=float,
        default=0.2,
        help="Ramp duration back to zero after program blocks [s].",
    )
    parser.add_argument(
        "--post-coast",
        type=float,
        default=0.0,
        help="Append a zero-throttle coast after any ramp blocks [s].",
    )
    parser.add_argument(
        "--pre-ramp",
        type=float,
        default=0.0,
        help="Pre-ramp from 0 to u0 before ramp blocks [s].",
    )

    # Optional gates
    parser.add_argument("--gates", action="store_true", help="Enable safety gates (if helper supports state access).")
    parser.add_argument("--v-min", type=float, default=None, help="Min speed gate [m/s].")
    parser.add_argument("--v-max", type=float, default=None, help="Max speed gate [m/s].")
    parser.add_argument("--yaw-max", type=float, default=None, help="Max yaw rate gate [rad/s].")
    parser.add_argument("--ax-max", type=float, default=None, help="Max |ax| gate [m/s^2].")
    parser.add_argument("--slip-max", type=float, default=None, help="Max |slip_est| gate.")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Send throttle excitation signals (no logging).")

    sub = p.add_subparsers(dest="mode", required=True)

    pr = sub.add_parser("ramp", help="Ramp u0->u1 then hold.")
    add_common_args(pr)
    pr.add_argument("--u0", type=float, required=True)
    pr.add_argument("--u1", type=float, required=True)
    pr.add_argument("--ramp-time", type=float, default=8.0)
    pr.add_argument("--hold", type=float, default=3.0)

    ps = sub.add_parser("step", help="Small step train around u0.")
    add_common_args(ps)
    ps.add_argument("--u0", type=float, required=True)
    ps.add_argument("--du", type=float, required=True)
    ps.add_argument("--hold", type=float, default=1.0)
    ps.add_argument("--steps", type=int, default=10)

    pb = sub.add_parser("burst", help="Burst then coast cycles.")
    add_common_args(pb)
    pb.add_argument("--u", type=float, required=True)
    pb.add_argument("--burst", type=float, default=1.5)
    pb.add_argument("--coast", type=float, default=3.0)
    pb.add_argument("--cycles", type=int, default=12)

    pp = sub.add_parser("prbs", help="PRBS around u0.")
    add_common_args(pp)
    pp.add_argument("--u0", type=float, required=True)
    pp.add_argument("--du", type=float, required=True)
    pp.add_argument("--bit", type=float, default=0.5)
    pp.add_argument("--bits", type=int, default=80)
    pp.add_argument("--seed", type=int, default=1)

    pm = sub.add_parser("multisine", help="Sum of sines around u0.")
    add_common_args(pm)
    pm.add_argument("--u0", type=float, required=True)
    pm.add_argument("--du", type=float, required=True)
    pm.add_argument("--freqs", type=str, required=True, help="Comma-separated Hz, e.g. 0.3,0.7,1.3")
    pm.add_argument("--dur", type=float, default=60.0)

    pq = sub.add_parser("suite", help="Run a default multi-block suite.")
    add_common_args(pq)
    pq.add_argument("--preset", type=str, default="low_speed", choices=["low_speed"])
    
    ptb = sub.add_parser("throt_brake", help="Throttle then brake sequence.")
    add_common_args(ptb)
    ptb.add_argument("--dur_t", type=float, default=6.0, help="Hold duration at target throttle.")
    ptb.add_argument("--dur_b", type=float, default=3.0, help="Hold duration at target brake.")
    ptb.add_argument("--dur_coast", type=float, default=3.0, help="Hold duration at coast (zero throttle/brake) between throttle and brake phases.")
    ptb.add_argument("--ramp_throt", type=float, default=0.6, help="Ramp duration for throttle transitions.")
    ptb.add_argument("--ramp_brake", type=float, default=0.5, help="Ramp duration for brake transitions.")
    ptb.add_argument("--osc-throttle", type=float, default=None, help="Append a final throttle oscillation around this center throttle.")
    ptb.add_argument("--osc-amp", type=float, default=0.05, help="Amplitude of the final throttle oscillation.")
    ptb.add_argument("--osc-freq", type=float, default=0.5, help="Frequency [Hz] of the final throttle oscillation.")
    ptb.add_argument("--osc-dur", type=float, default=0.0, help="Duration [s] of the final throttle oscillation segment.")

    return p

def make_helper() -> Helper:
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    return Controller()

def main() -> int:
    args = build_parser().parse_args()
    helper = make_helper()

    gates = Gates(
        enabled=bool(args.gates),
        v_min=args.v_min,
        v_max=args.v_max,
        yaw_rate_max=args.yaw_max,
        ax_max=args.ax_max,
        slip_max=args.slip_max,
    )

    player = ThrottlePlayer(
        helper=helper,
        dt=args.dt,
        u_min=args.u_min,
        u_max=args.u_max,
        warmup=args.warmup,
        cooldown=args.cooldown,
        gates=gates,
    )

    # Ensure throttle returns to zero on Ctrl+C / SIGTERM
    def _handle_sig(_signum, _frame):
        player.request_stop()
        try:
            helper.set_throttle(0.0)
        except Exception:
            pass
        sys.exit(130)

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    try:
        if args.mode == "ramp":
            ramp = prog_ramp_hold(args.u0, args.u1, args.ramp_time, args.hold)
            coast = prog_hold(u=0.0, duration=args.post_coast)
            if args.pre_ramp > 0.0:
                pre = prog_ramp_hold(0.0, args.u0, args.pre_ramp, hold_time=0.0)
                prog = chain_programs(pre, ramp, coast)
            else:
                prog = chain_programs(ramp, coast)
        elif args.mode == "step":
            prog = prog_step_train(args.u0, args.du, args.hold, args.steps)
        elif args.mode == "burst":
            prog = prog_burst_coast(args.u, args.burst, args.coast, args.cycles)
        elif args.mode == "prbs":
            prog = prog_prbs(args.u0, args.du, args.bit, args.bits, args.seed)
        elif args.mode == "multisine":
            freqs = parse_freqs(args.freqs)
            if not freqs:
                raise ValueError("Provide at least one frequency in --freqs")
            prog = prog_multisine(args.u0, args.du, freqs, args.dur, sample_dt=args.dt)
        elif args.mode == "suite":
            prog = suite_low_speed(post_coast=args.post_coast, pre_ramp=args.pre_ramp)
        elif args.mode == "throt_brake":
            prog = suite_throttle_brake_manual_steer(
                ramp_duration=args.ramp_throt, hold=args.dur_t,
                brake_ramp=args.ramp_brake, brake_hold=args.dur_b,
                coast_hold=args.dur_coast, dt_points=args.dt,
                oscillate_throttle=args.osc_throttle,
                oscillate_amplitude=args.osc_amp,
                oscillate_frequency=args.osc_freq,
                oscillate_duration=args.osc_dur,
            )
        else:
            raise ValueError(f"Unknown mode: {args.mode}")

        # prog = apply_ramps(
        #     prog,
        #     ramp_up=args.ramp_up,
        #     ramp_down=args.ramp_down,
        #     u_zero=0.0,
        # )

        print(f"[throttle_sweep] Running mode={args.mode} (Ctrl+C to stop).")
        player.play(prog, name=args.mode)

    finally:
        # Always return throttle to zero
        try:
            helper.set_throttle(0.0)
        except Exception:
            pass
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    print("[throttle_sweep] Done.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
