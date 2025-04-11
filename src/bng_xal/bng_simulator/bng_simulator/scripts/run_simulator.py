#!/usr/bin/env python3

"""
Main script to run the BeamNG simulation manager node
"""

import rclpy
from rclpy.node import Node
import sys
import os
import argparse
from bng_simulator.sim_manager_node import SimulationManagerNode


def main(args=None):
    """
    Main function to run the simulation manager node
    """
    rclpy.init(args=args)

    # Parse arguments
    parser = argparse.ArgumentParser(description="BeamNG Simulation Manager")
    parser.add_argument(
        "--config",
        type=str,
        default="basic_scenario.yaml",
        help="The configuration file to use",
    )
    parsed_args, remaining = parser.parse_known_args(args=args)

    # Find config file - check if it's a path or just a filename
    config_path = parsed_args.config
    if not os.path.isfile(config_path):
        # Try to find in the package's config directory
        from ament_index_python.packages import get_package_share_directory

        package_config_dir = os.path.join(
            get_package_share_directory("bng_simulator"), "config", "scenarios"
        )
        possible_path = os.path.join(package_config_dir, config_path)
        if os.path.isfile(possible_path):
            config_path = possible_path

    # Create and spin the node
    node = SimulationManagerNode(config_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown ROS and cleanup
        if node.logger_process:
            node.logger_process.stop()
            node.logger_process.join()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
