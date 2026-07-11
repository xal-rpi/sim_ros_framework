#!/usr/bin/env python3
"""Smoke-test vehicle UDP I/O against a running BeamNG sim + xlab controller.

Binds control_state_send (exclusive). Sends commands via sendto to vehicle control_listen.

Usage::

    ros2 run bng_controller test_vehicle_udp -- --command-port 64257 --state-port 64258
    ros2 run bng_controller test_vehicle_udp -- --command-port 64257 --state-port 64258 --cmd-case torque_and_steering
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from bng_controller.llc_scalar_commands import LLC_SCALAR_CMD_CASES
from bng_controller.vehicle_io import VehicleIoClient


def main() -> int:
    parser = argparse.ArgumentParser(description="Test xlab control_listen / control_state_send UDP path")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, required=True, help="Vehicle control_listen port")
    parser.add_argument("--state-port", type=int, required=True, help="Companion control_state_send bind port")
    parser.add_argument("--timeout", type=float, default=5.0, help="Seconds to wait for state")
    parser.add_argument("--no-send-zero-cmd", action="store_true", help="Skip sending a command envelope")
    parser.add_argument(
        "--cmd-case",
        choices=sorted(LLC_SCALAR_CMD_CASES.keys()),
        default="torque_and_steering",
        help="Scalar LLC cmd payload to send (see bng_controller.llc_scalar_commands)",
    )
    parser.add_argument("--send-tune", action="store_true", help="Send a typed tune envelope after cmd")
    parser.add_argument("--list-cmd-cases", action="store_true", help="Print scalar cmd cases and exit")
    args = parser.parse_args()

    if args.list_cmd_cases:
        for name, payload in sorted(LLC_SCALAR_CMD_CASES.items()):
            print(f"{name}: {json.dumps(payload)}")
        return 0

    command_addr = (args.host, args.command_port)
    state_bind = (args.host, args.state_port)

    print(f"Binding state receiver on {state_bind[0]}:{state_bind[1]}")
    print(f"Sending commands to {command_addr[0]}:{command_addr[1]}")

    deadline = time.time() + args.timeout
    state = None
    cmd_payload = LLC_SCALAR_CMD_CASES[args.cmd_case]

    with VehicleIoClient(command_addr=command_addr, control_state_bind_addr=state_bind) as io:
        if not args.no_send_zero_cmd:
            io.send_command(cmd_payload)
            print(f"Sent cmd case '{args.cmd_case}': {json.dumps(cmd_payload)}")

        if args.send_tune:
            io.send_tune({"TorqueKp": 0.1})
            print("Sent tune envelope")

        while time.time() < deadline:
            remaining = max(0.05, deadline - time.time())
            state = io.recv_control_state(timeout=remaining)
            if state is not None:
                break

    if state is None:
        print("No control state received before timeout", file=sys.stderr)
        return 1

    print("Received control state:")
    print(json.dumps(state, indent=2))
    if state.get("t", -1) < 0:
        print("Warning: state.t < 0 — gtState may not be connected yet", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
