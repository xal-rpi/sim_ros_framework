#!/usr/bin/env python3

"""
Interactive shell for controlling BeamNG simulation through ROS services.
"""

import sys
import cmd
import yaml
import rclpy
from rclpy.node import Node
from bng_msgs.srv import ExecuteRequest, StartLogger, StopLogger


class SimulationShell(cmd.Cmd, Node):
    intro = """
    ==============================================
    BeamNG Simulation Interactive Control Shell
    ==============================================
    Type 'help' or '?' to list commands.
    Type 'exit' or 'quit' to exit the shell.
    """
    prompt = "beamng> "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rclpy.init()
        Node.__init__(self, "simulation_shell")

        # Create service clients
        self.execute_client = self.create_client(ExecuteRequest, "execute_request")
        self.start_logger_client = self.create_client(StartLogger, "start_logger")
        self.stop_logger_client = self.create_client(StopLogger, "stop_logger")

        # Wait for services to be available
        self.wait_for_services()

        # Keep track of available vehicles and their sensors
        self.vehicles = []
        self.sensors = {}
        self.default_vehicle = None

        # Get available vehicles and sensors
        self.get_simulation_info()

    def wait_for_services(self):
        """Wait for all required services to become available."""
        self.get_logger().info("Waiting for services...")

        self.execute_client.wait_for_service()
        self.get_logger().info("- 'execute_request' service available")

        self.start_logger_client.wait_for_service()
        self.get_logger().info("- 'start_logger' service available")

        self.stop_logger_client.wait_for_service()
        self.get_logger().info("- 'stop_logger' service available")

    def get_simulation_info(self):
        """Get information about the simulation, vehicles, and sensors."""
        # Get available vehicles
        resp = self.execute_request("get_available_vehicles")
        if resp and "result" in resp:
            self.vehicles = resp["result"]
            self.get_logger().info(f"Available vehicles: {self.vehicles}")

            # Get default vehicle
            resp_default = self.execute_request("get_sim_config")
            if resp_default:
                self.default_vehicle = resp_default.get(
                    "default_vehicle", self.vehicles[0] if self.vehicles else None
                )
                self.get_logger().info(f"Default vehicle: {self.default_vehicle}")

                # Get sensors for each vehicle
                for vehicle in self.vehicles:
                    # We'd need to have a function to get sensors for each vehicle
                    # For now, let's assume we'll discover them as we use the simulation
                    self.sensors[vehicle] = []

    def parse_args(self, arg):
        """Convert arguments string to Python dict."""
        if not arg.strip():
            return {}

        args = {}

        # Split the argument string and process each key=value pair
        for param in arg.split():
            if "=" in param:
                key, value = param.split("=", 1)
                # Try to parse as YAML for complex values like lists
                try:
                    parsed_value = yaml.safe_load(value)
                    args[key] = parsed_value
                except yaml.YAMLError:
                    # If YAML parsing fails, keep as string
                    args[key] = value

        return args

    def execute_request(self, function_name, **kwargs):
        """Execute a request through the simulation manager."""
        request = ExecuteRequest.Request()
        request.function_name = function_name

        # Convert kwargs to YAML string
        if kwargs:
            request.arguments = yaml.dump(kwargs, sort_keys=False).strip()
        else:
            request.arguments = ""

        self.get_logger().info(f"Executing: {function_name} with args: {kwargs}")

        future = self.execute_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result().result
            # Print the raw result to help with debugging
            print(f"Raw result from {function_name}: {result}")

            if result.strip():  # Check if result is not empty
                try:
                    # Parse the result from YAML string
                    parsed_result = yaml.safe_load(result)
                    return parsed_result
                except yaml.YAMLError as e:
                    self.get_logger().error(f"Failed to parse result as YAML: {e}")

                    # Try a more forgiving approach - treat as JSON if possible
                    try:
                        import json

                        # Try to clean up the string before parsing
                        cleaned_result = result.replace("'", '"')
                        parsed_result = json.loads(cleaned_result)
                        return parsed_result
                    except json.JSONDecodeError:
                        # Return the raw result if all parsing fails
                        return {"raw_result": result}
            else:
                # Handle empty result case
                self.get_logger().warning(f"Empty result from {function_name}")
                return {"success": True}  # Assume success for empty responses
        else:
            self.get_logger().error(f"Service call failed: {function_name}")
            return None

    def do_help(self, arg):
        """List available commands with help text."""
        cmd.Cmd.do_help(self, arg)

    def do_exit(self, arg):
        """Exit the shell."""
        print("Exiting simulation shell...")
        return True

    def do_EOF(self, arg):
        """Exit the shell."""
        return self.do_exit(arg)

    def do_quit(self, arg):
        """Exit the shell."""
        return self.do_exit(arg)

    def do_vehicles(self, arg):
        """
        List available vehicles in the simulation.

        Usage: vehicles
        """
        response = self.execute_request("get_available_vehicles")
        if response and "result" in response:
            self.vehicles = response["result"]
            print(f"Available vehicles: {self.vehicles}")
            print(f"Default vehicle: {self.default_vehicle}")
        else:
            print("Failed to retrieve vehicles")

    def do_teleport(self, arg):
        """
        Teleport a vehicle to a specified position and orientation.

        Usage: teleport [vehicle_name=<name>] pos=[x,y,z] [rot_quat=[x,y,z,w]]
               or teleport [vehicle_name=<name>] pos=[x,y,z] [yaw_angle=<degrees>] [pitch_angle=<degrees>] [roll_angle=<degrees>]

        Example: teleport vehicle_name=ego_vehicle pos=[0,0,0] yaw_angle=90
        """
        args = self.parse_args(arg)
        if args is None:
            return

        response = self.execute_request("teleport_vehicle", **args)
        if response and response.get("success", False):
            print("Vehicle successfully teleported")
        else:
            print(f"Teleport response: {response}")
            print("Failed to teleport vehicle")

    def do_control(self, arg):
        """
        Send control inputs to a vehicle.

        Usage: control [vehicle_name=<name>] steering=<float> throttle=<float> brake=<float> [clutch=<float>] [parkingbrake=<bool>] [gear=<int>]

        Example: control vehicle_name=ego_vehicle steering=0.5 throttle=0.7 brake=0
        """
        if not arg:
            print("Error: At least one control parameter is required")
            return

        # Parse arguments manually for this command
        args = {}
        vehicle_name = self.default_vehicle

        # Split the argument string and process each key=value pair
        for param in arg.split():
            if "=" in param:
                key, value = param.split("=", 1)
                if key == "vehicle_name":
                    vehicle_name = value
                else:
                    # Convert numeric values appropriately
                    try:
                        if value.lower() == "true":
                            args[key] = True
                        elif value.lower() == "false":
                            args[key] = False
                        elif "." in value:
                            args[key] = float(value)
                        else:
                            args[key] = int(value)
                    except ValueError:
                        # If conversion fails, keep as string
                        args[key] = value

        # Call the control_vehicle function
        print(f"Sending control command to {vehicle_name}: {args}")
        response = self.execute_request(
            "control_vehicle", vehicle_name=vehicle_name, **args
        )

        # Consider any response (even empty dict) as success unless it's None
        if response is not None:
            print(f"Control command sent successfully")
        else:
            print("Failed to send control command")

    def do_logger(self, arg):
        """
        Start or stop the logger.

        Usage: logger start <save_location> [max_queue_size=<int>] [flush_interval=<float>]
               logger stop

        Example: logger start /tmp/simulation_logs max_queue_size=1000 flush_interval=0.5
        """
        args = arg.split()
        if not args:
            print("Error: Must specify 'start' or 'stop'")
            return

        command = args[0]

        if command == "start":
            if len(args) < 2:
                print("Error: Save location is required")
                return

            save_location = args[1]
            max_queue_size = 100  # Default
            flush_interval = 0.5  # Default

            # Parse optional parameters
            for i in range(2, len(args)):
                param = args[i].split("=")
                if len(param) == 2:
                    if param[0] == "max_queue_size":
                        try:
                            max_queue_size = int(param[1])
                        except ValueError:
                            print(f"Invalid value for max_queue_size: {param[1]}")
                            return
                    elif param[0] == "flush_interval":
                        try:
                            flush_interval = float(param[1])
                        except ValueError:
                            print(f"Invalid value for flush_interval: {param[1]}")
                            return

            # Create request
            request = StartLogger.Request()
            request.save_location = save_location
            request.max_queue_size = max_queue_size
            request.flush_interval = flush_interval

            # Call service
            future = self.start_logger_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None and future.result().success:
                print(f"Logger started. Saving to {save_location}")
            else:
                print("Failed to start logger")

        elif command == "stop":
            # Create request
            request = StopLogger.Request()

            # Call service
            future = self.stop_logger_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None and future.result().success:
                print("Logger stopped")
            else:
                print("Failed to stop logger")

        else:
            print(f"Unknown logger command: {command}")
            print("Use 'start' or 'stop'")

    def do_exec(self, arg):
        """
        Execute a custom request to the simulation manager.

        Usage: exec <function_name> [param1=value1] [param2=value2] ...

        Example: exec get_sim_config
                 exec teleport_vehicle vehicle_name=ego_vehicle pos=[0,0,0] yaw_angle=90
        """
        args = arg.split(maxsplit=1)
        if not args:
            print("Error: Function name is required")
            return

        function_name = args[0]
        args_str = args[1] if len(args) > 1 else ""

        kwargs = self.parse_args(args_str)
        if kwargs is None:
            return

        response = self.execute_request(function_name, **kwargs)
        if response:
            print("Response:")
            yaml.dump(response, sys.stdout, default_flow_style=False)
        else:
            print(f"Failed to execute {function_name}")


def main():
    shell = SimulationShell()
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        print("\nExiting simulation shell...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
