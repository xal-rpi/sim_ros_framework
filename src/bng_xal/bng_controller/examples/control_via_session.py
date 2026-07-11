#!/usr/bin/env python3
"""Example: scalar control via VehicleSession (no ROS publisher loop).

Requires sim running::

    ros2 launch bng_bringup simulator.launch.py config:=gridworld.yaml

Run (another terminal, after source install/setup.bash)::

    python3 examples/control_via_session.py

Uses ``recreate=True`` so this script does not steal the dispatcher's UDP binds.
"""

from __future__ import annotations

import time

from bng_msgs.msg import BngControlCmd
from bng_controller.vehicle_session import VehicleSession


def main() -> None:
    vehicle = "EGO"
    with VehicleSession.from_vehicle_name(vehicle, recreate=True) as session:
        cmd = BngControlCmd()
        cmd.valid_fields = (
            BngControlCmd.FIELD_TORQUE | BngControlCmd.FIELD_STEERING_INPUT
        )
        cmd.torque = 60.0
        for steer in (0.0, 0.2, -0.2, 0.0):
            cmd.steering_input = steer
            session.send_control_cmd(cmd)
            print(f"sent torque={cmd.torque} steering_input={steer}")
            time.sleep(2.0)

        # Hold straight, coast torque
        cmd.valid_fields = BngControlCmd.FIELD_TORQUE
        cmd.torque = 0.0
        session.send_control_cmd(cmd)
        print("sent torque=0")


if __name__ == "__main__":
    main()
