#!/usr/bin/env python3
"""Run sim_manager_node with an explicit config path (dev helper).

Prefer the launch file for normal use::

    ros2 launch bng_bringup simulator.launch.py config:=gridworld.yaml
"""

import argparse
import sys

import rclpy

from bng_simulator.sim_manager_node import SimulationManagerNode
from bng_simulator.utils.scenario_compose import resolve_config_path


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="BeamNG Simulation Manager")
    parser.add_argument(
        "--config",
        type=str,
        default="gridworld.yaml",
        help="Run config name or path (see bng_bringup/config/runs/)",
    )
    parsed_args, _remaining = parser.parse_known_args(args=args)

    config_path = resolve_config_path(parsed_args.config)
    node = SimulationManagerNode(config_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
