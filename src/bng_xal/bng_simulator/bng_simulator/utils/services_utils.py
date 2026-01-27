"""
Some utility functions for services
"""

import os
import rclpy

from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from bng_simulator.utils.io_dict_utils import convert_dict_to_str, convert_str_to_dict

# Services imports
from bng_msgs.srv import ExecuteRequest


def convert_sim_time_to_ros_time(sim_time):
    """
    Convert simulation time in seconds to ROS time.

    Args:
        sim_time (float): The simulation time in seconds

    Returns:
        Time: The ROS time.
    """
    return Time(sec=int(sim_time), nanosec=int((sim_time - int(sim_time)) * 1e9))


def convert_time_to_header(time: float, frame_id: str = "map") -> Header:
    """
    Convert time to a ROS header.

    Args:
        time (float): The time in seconds.
        frame_id (str): The frame ID.

    Returns:
        Header: The ROS header.
    """
    return Header(stamp=convert_sim_time_to_ros_time(time), frame_id=frame_id)


def create_unique_node_name(base_name: str):
    return f"{base_name}_{os.getpid()}"


def send_request(
    function_name: str,
    function_args: dict | None = None,
    timeout_sec: float = 2.0,
    node_ros=None,
):
    """
    Send a request to the simulation manager node.

    Args:
        function_name (str): Name of the function to execute
        function_args (dict): Arguments for the function
    """
    if function_args is None:
        function_args = {}

    started_rclpy = False
    created_node = False

    if node_ros is None:
        if not rclpy.ok():
            rclpy.init()
            started_rclpy = True
        node = rclpy.create_node(create_unique_node_name("send_request"))
        created_node = True
    else:
        node = node_ros

    try:
        client = node.create_client(ExecuteRequest, "execute_request")
        # Wait for service to be available
        max_attempts = 5
        num_attempts = 0
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info("service not available, waiting again...")
            num_attempts += 1
            if num_attempts >= max_attempts:
                node.get_logger().info("service not available, exiting...")
                return
        # Create the request
        request = ExecuteRequest.Request()
        # Let's create the string to send
        function_args_str = convert_dict_to_str(function_args)

        # Set the function name and arguments
        request.function_name = function_name
        request.arguments = function_args_str

        # Send the request asynchronously
        future = client.call_async(request)
        # Request time out
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        response_dict = None
        if future.result() is not None:
            response = future.result()
            response_dict = convert_str_to_dict(response.result)
        else:
            node.get_logger().info("Service call failed")

        return response_dict
    finally:
        # Clean up only what we created/started here.
        if created_node:
            node.destroy_node()
        if started_rclpy:
            rclpy.shutdown()
