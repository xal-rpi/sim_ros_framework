#!/usr/bin/env python3
"""Smoke-test vehicle sensor_send UDP path against a running BeamNG sim.

Binds sensor_send (exclusive — stop sensor_dispatcher / other receivers first).
Does not use VehicleIoClient command/state binds; vehicle Lua sendto's here.

Collects observations until every expected sensor type tag is seen or timeout.
Vehicle sends map batches ``{ sent_t, <name>: observation }``; ``recv_sensor`` flattens them.

Usage::

    ros2 run bng_controller test_vehicle_sensor_udp -- --sensor-port 64259
    ros2 run bng_controller test_vehicle_sensor_udp -- --sensor-port 64259 --expect gtstate
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any, Dict, List

from bng_controller.vehicle_io import VehicleIoClient


def _parse_expect_list(value: str) -> List[str]:
    return [part.strip() for part in value.split(",") if part.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description="Test xlab sensor_send UDP path")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--sensor-port", type=int, required=True, help="Client sensor_send bind port")
    parser.add_argument("--timeout", type=float, default=5.0, help="Seconds to wait for sensor data")
    parser.add_argument(
        "--expect",
        default="gtstate,imu",
        help="Comma-separated sensor type tags required before success (default: gtstate,imu)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print full JSON for the first observation of each sensor type",
    )
    args = parser.parse_args()

    expected = _parse_expect_list(args.expect)
    if not expected:
        print("No sensor types listed in --expect", file=sys.stderr)
        return 2

    sensor_bind = (args.host, args.sensor_port)
    print(f"Binding sensor receiver on {sensor_bind[0]}:{sensor_bind[1]}")
    print(f"Waiting for sensor types: {expected}")

    deadline = time.time() + args.timeout
    seen: Dict[str, Dict[str, Any]] = {}

    with VehicleIoClient(
        command_addr=(args.host, 64257),
        sensor_bind_addr=sensor_bind,
    ) as io:
        while time.time() < deadline:
            remaining = max(0.05, deadline - time.time())
            observations = io.recv_sensor(timeout=remaining)
            if not observations:
                continue

            for observation in observations:
                sensor_type = observation.get("sensor")
                if not sensor_type:
                    print("Warning: observation missing 'sensor' field", file=sys.stderr)
                    continue

                if sensor_type not in seen:
                    seen[sensor_type] = observation
                    print(
                        f"NEW: {sensor_type} "
                        f"name={observation.get('name')!r} "
                        f"t={observation.get('t')}"
                    )
                    if args.verbose:
                        print(json.dumps(observation, indent=2))

            if all(sensor in seen for sensor in expected):
                break

    print(f"Seen types: {sorted(seen)}")

    missing = [sensor for sensor in expected if sensor not in seen]
    if missing:
        print(f"Missing required sensor types: {missing}", file=sys.stderr)
        return 1

    for sensor in expected:
        t = seen[sensor].get("t", -1)
        if t is None or t < 0:
            print(
                f"Warning: {sensor!r} observation.t < 0 — sensor may not be ready yet",
                file=sys.stderr,
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
