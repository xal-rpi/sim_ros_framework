#!/usr/bin/env python3
"""Calibrate steering_to_input: sweep steering_input, read delta_l/delta_r.

Uses direct BeamNG input (FIELD_STEERING_INPUT). Requires sim + dispatcher running.
Binds control_state_send (dispatcher does not use that port).
"""

from __future__ import annotations

import argparse
import time

from bng_msgs.msg import BngControlCmd
from bng_controller.vehicle_session import VehicleSession


def roadwheel_rad(state: dict) -> float:
    return 0.5 * (float(state["delta_l"]) + float(state["delta_r"]))


def sample_roadwheel(io, samples: int, interval: float) -> tuple[float, float, float] | None:
    last = None
    for _ in range(samples):
        state = io.recv_control_state(timeout=interval + 0.5)
        if state is not None:
            last = state
        time.sleep(interval)
    if last is None:
        return None
    dl = float(last["delta_l"])
    dr = float(last["delta_r"])
    return dl, dr, 0.5 * (dl + dr)


def main() -> None:
    parser = argparse.ArgumentParser(description="steering_input → roadwheel calibration")
    parser.add_argument("--vehicle", default="EGO")
    parser.add_argument("--torque", type=float, default=40.0, help="hold torque [N·m]")
    parser.add_argument("--hold", type=float, default=3.0, help="seconds per step")
    parser.add_argument(
        "--inputs",
        type=float,
        nargs="+",
        default=[-1.0, 0.0, 1.0],
        help="steering_input values to try",
    )
    args = parser.parse_args()

    cmd = BngControlCmd()
    cmd.valid_fields = BngControlCmd.FIELD_TORQUE | BngControlCmd.FIELD_STEERING_INPUT
    cmd.torque = args.torque

    with VehicleSession.from_vehicle_name(
        args.vehicle,
        recreate=True,
        bind_control_state=True,
    ) as session:
        io = session.io
        estimates: list[float] = []
        print(f"{'input':>8}  {'delta_l':>9}  {'delta_r':>9}  {'avg':>9}")
        for u in args.inputs:
            cmd.steering_input = u
            session.send_control_cmd(cmd)
            time.sleep(args.hold * 0.5)
            row = sample_roadwheel(io, samples=8, interval=0.1)
            if row is None:
                print(f"{u:+8.3f}  (no control_state)")
                continue
            dl, dr, avg = row
            print(f"{u:+8.3f}  {dl:+9.4f}  {dr:+9.4f}  {avg:+9.4f}")
            if abs(u) > 1e-6:
                estimates.append(avg / u)

        cmd.valid_fields = BngControlCmd.FIELD_TORQUE | BngControlCmd.FIELD_STEERING_INPUT
        cmd.torque = 0.0
        cmd.steering_input = 0.0
        session.send_control_cmd(cmd)

    print()
    if estimates:
        k = sum(estimates) / len(estimates)
        print(f"steering_to_input estimate (rad / input): {k:+.6f}")
        print("  (copy into vehicle_catalog.yaml → catalog.<vehicle>.steering_to_input)")
    else:
        print("No nonzero steering_input samples — cannot estimate scale.")


if __name__ == "__main__":
    main()
