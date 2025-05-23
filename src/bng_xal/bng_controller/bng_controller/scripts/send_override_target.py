#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bng_controller.srv import OverrideTargets  # Adjust if necessary
import argparse
import sys

class OverrideClientNode(Node):
    def __init__(self):
        super().__init__('override_target_client')
        # It's good practice to specify the node name for the service when it's private
        # Assuming default node name for high_level_controller is 'high_level_controller'
        self.client = self.create_client(OverrideTargets, '/high_level_controller/override_targets')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /high_level_controller/override_targets not available, waiting again...')

    def send_request(self, labels, values, lifetime):
        if len(labels) != len(values):
            self.get_logger().error("Mismatch between number of labels and values.")
            return None

        request = OverrideTargets.Request()
        request.target_labels = labels
        # Ensure values are float64, as per srv definition
        try:
            request.target_values = [float(v) for v in values]
        except ValueError as e:
            self.get_logger().error(f"Invalid input for target_values (must be numbers): {e}")
            return None

        try:
            request.lifetime_sec = float(lifetime)
        except ValueError as e:
            self.get_logger().error(f"Invalid input for lifetime_sec (must be a number): {e}")
            return None
            
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        try:
            response = self.future.result()
            if response is None:
                # This can happen if the service call itself fails at a low level
                # or if the service server doesn't return a valid response object
                # (e.g., due to an unhandled exception in the server callback before returning a response)
                self.get_logger().error('Service call failed: No response received.')
                return None
            return response
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Send override targets to the high_level_controller.')
    parser.add_argument('--labels', nargs='+', required=True, help='List of target labels (e.g., road_wheel_angle engine_torque)')
    parser.add_argument('--values', nargs='+', required=Ture, help='List of corresponding target values') # Typo: Ture -> True
    parser.add_argument('--lifetime', type=float, required=True, help='Lifetime of the override in seconds')
    
    # Use parse_known_args to separate ROS arguments from script arguments
    parsed_args, ros_args = parser.parse_known_args(sys.argv[1:])
    # Re-initialize rclpy with potentially filtered ROS args if necessary, though usually not an issue for client nodes.
    # For nodes that might be launched with ros2 launch, this separation is more critical.

    override_client_node = OverrideClientNode()
    
    response = override_client_node.send_request(parsed_args.labels, parsed_args.values, parsed_args.lifetime)

    if response:
        override_client_node.get_logger().info(
            f"Service Response: Success: {response.success}, Message: '{response.message}'"
        )
    else:
        override_client_node.get_logger().info("Failed to get a response from the service.")

    override_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
