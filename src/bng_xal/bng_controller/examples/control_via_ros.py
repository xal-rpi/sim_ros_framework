#!/usr/bin/env python3
"""Example: publish BngControlCmd on the ROS topic the dispatcher listens to.

Requires sim + sensor_dispatcher running::

    ros2 launch bng_bringup simulator.launch.py config:=gridworld.yaml

Run::

    python3 examples/control_via_ros.py

Topic: /<vehicle>/control/cmd
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from bng_msgs.msg import BngControlCmd


class CmdPublisher(Node):
    def __init__(self, vehicle: str) -> None:
        super().__init__("example_control_publisher")
        self._pub = self.create_publisher(
            BngControlCmd,
            f"/{vehicle}/control/cmd",
            10,
        )

    def publish_torque_and_steer(self, torque: float, steering_rad: float) -> None:
        msg = BngControlCmd()
        msg.valid_fields = (
            BngControlCmd.FIELD_TORQUE | BngControlCmd.FIELD_STEERING
        )
        msg.torque = torque
        msg.steering = steering_rad
        self._pub.publish(msg)
        self.get_logger().info(
            f"published torque={torque} steering={steering_rad} rad"
        )


def main() -> None:
    rclpy.init()
    node = CmdPublisher("EGO")
    try:
        node.publish_torque_and_steer(50.0, 0.05)
        time.sleep(2.0)
        node.publish_torque_and_steer(50.0, -0.05)
        time.sleep(2.0)
        node.publish_torque_and_steer(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
