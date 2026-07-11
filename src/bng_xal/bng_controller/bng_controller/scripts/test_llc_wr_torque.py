#!/usr/bin/env python3
"""Hold LLC scalar commands and print rear wheel speed vs torque estimate.

Sends torque and/or wheel_speed with steering=0, then samples control_send
(reduced gtState) for wr, rear_wheel_torque_est, and LLC target_* fields.

Requires sim + xlab LLC running (e.g. gridworld.yaml). Do not bind control_send
elsewhere (only one process may bind each UDP port).

Usage::

    ros2 run bng_controller test_llc_wr_torque -- \\
        --command-port 64257 --state-port 64258 \\
        --torque 50 --wheel-speed 5 --duration 15

    # torque feedforward only (no wheel-speed PI target)
    ros2 run bng_controller test_llc_wr_torque -- \\
        --command-port 64257 --state-port 64258 --torque 80 --no-wheel-speed
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any, Dict, Optional

from bng_controller.vehicle_io import VehicleIoClient


def _fmt(v: Any, places: int = 4) -> str:
    if v is None:
        return "n/a"
    try:
        return f"{float(v):{places + 3}.{places}f}"
    except (TypeError, ValueError):
        return str(v)


def _build_command(args: argparse.Namespace) -> Dict[str, float]:
    cmd: Dict[str, float] = {"steering": float(args.steering)}
    if args.torque is not None:
        cmd["torque"] = float(args.torque)
    if args.wheel_speed is not None:
        cmd["wheel_speed"] = float(args.wheel_speed)
    return cmd


def main() -> int:
    parser = argparse.ArgumentParser(
        description="LLC hold command: monitor wr and rear_wheel_torque_est"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, required=True)
    parser.add_argument("--state-port", type=int, required=True)
    parser.add_argument("--torque", type=float, default=50.0, help="Torque command [Nm]")
    parser.add_argument(
        "--wheel-speed",
        type=float,
        default=5.0,
        help="Rear wheel speed setpoint [m/s]",
    )
    parser.add_argument(
        "--no-wheel-speed",
        action="store_true",
        help="Omit wheel_speed from command (torque-only / FF path)",
    )
    parser.add_argument(
        "--no-torque",
        action="store_true",
        help="Omit torque from command (wheel-speed tracking only)",
    )
    parser.add_argument("--steering", type=float, default=0.0, help="Roadwheel [rad]")
    parser.add_argument("--duration", type=float, default=15.0, help="Hold time [s]")
    parser.add_argument(
        "--command-period",
        type=float,
        default=1.0,
        help="Resend cmd interval [s] (keep below LLC command_timeout)",
    )
    parser.add_argument(
        "--print-period",
        type=float,
        default=0.25,
        help="Min seconds between printed samples",
    )
    args = parser.parse_args()

    if args.no_wheel_speed:
        args.wheel_speed = None
    if args.no_torque:
        args.torque = None
    if args.torque is None and args.wheel_speed is None:
        print("Need --torque and/or --wheel-speed (or drop --no-* flags)", file=sys.stderr)
        return 2

    cmd = _build_command(args)
    command_addr = (args.host, args.command_port)
    state_bind = (args.host, args.state_port)

    print(f"Command addr (sendto): {command_addr[0]}:{command_addr[1]}")
    print(f"State bind:            {state_bind[0]}:{state_bind[1]}")
    print(f"Hold payload:          {cmd}")
    print(f"Duration:              {args.duration}s")
    print()
    print(
        f"{'t':>8}  {'wr':>10}  {'tgt_wr':>10}  "
        f"{'rear_T_est':>12}  {'tgt_T':>10}  {'thr':>8}  {'brk':>8}"
    )
    print("-" * 76)

    deadline = time.time() + args.duration
    next_cmd = 0.0
    next_print = 0.0
    samples = 0
    last_state: Optional[Dict[str, Any]] = None

    with VehicleIoClient(command_addr=command_addr, control_state_bind_addr=state_bind) as io:
        while time.time() < deadline:
            now = time.time()
            if now >= next_cmd:
                io.send_command(cmd)
                next_cmd = now + args.command_period

            state = io.recv_control_state(timeout=0.05)
            if state is not None:
                last_state = state

            if last_state is None or last_state.get("t", -1) < 0:
                continue

            if now < next_print:
                continue
            next_print = now + args.print_period
            samples += 1

            print(
                f"{_fmt(last_state.get('t'), 3):>8}  "
                f"{_fmt(last_state.get('wr')):>10}  "
                f"{_fmt(last_state.get('target_wr')):>10}  "
                f"{_fmt(last_state.get('rear_wheel_torque_est'), 2):>12}  "
                f"{_fmt(last_state.get('target_torque')):>10}  "
                f"{_fmt(last_state.get('throttle'), 3):>8}  "
                f"{_fmt(last_state.get('brake'), 3):>8}"
            )

    print("-" * 76)
    if samples == 0:
        print("No valid control state received (check ports / gtState / controlStateRate)", file=sys.stderr)
        return 1

    if last_state:
        print("Last sample:")
        print(f"  wr                  = {last_state.get('wr')}")
        print(f"  target_wr           = {last_state.get('target_wr')}")
        print(f"  rear_wheel_torque_est = {last_state.get('rear_wheel_torque_est')}")
        print(f"  target_torque       = {last_state.get('target_torque')}")
        print(f"  throttle            = {last_state.get('throttle')}")
        print(f"  brake               = {last_state.get('brake')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
