"""
Vehicle management class for BeamNG simulation with ROS integration.
"""

from typing import Dict, Any, Optional
from copy import deepcopy

from beamngpy import Vehicle, BeamNGpy
from bng_simulator.vehicle.sensors import SensorBase, SensorRegistry
from bng_simulator.vehicle.controllers.base import ControllerRegistry
from bng_simulator.utils.math_op import convert_euler_to_quaternion

from rclpy.node import get_logger


class VehicleManager:
    """
    Manages a vehicle instance and its sensors in a BeamNG simulation.

    Handles vehicle, sensor initialization, and data collection with
    proper lifecycle management.

    Args:
        name (str): Vehicle name
        beamng (BeamNGpy): Active BeamNG simulation instance
        config (dict): Vehicle configuration dictionary
    """

    def __init__(self, name: str, beamng: BeamNGpy, config: Dict[str, Any]):
        self._name = name
        self._beamng = beamng
        self._config = deepcopy(config)
        self._sensors: Dict[str, SensorBase] = {}
        self._logger = get_logger(__name__)
        self._vehicle: Vehicle = self.create_vehicle_instance()
        self._controllers = {}  # Dictionary to store controllers

    def create_vehicle_instance(self) -> Vehicle:
        """
        Create a vehicle instance based on the configuration.

        Returns:
            Vehicle: The created vehicle instance
        """
        # Let's extract the configuration for the sensors
        self._sensors_config = self._config.pop("sensors", {})

        # Let's extract scenario-based arguments
        self._scenario_args = self._config.pop("scenario_args", {})

        # Now log the remaining configuration
        self._logger.debug(f"Vehicle --{self._name}-- configuration: \n{self._config}")

        # Let's construct the vehicle instance
        return Vehicle(self._name, **self._config, extensions=["xlab/xlabCore"])

    def get_scenario_args(self):
        """
        Get scenario spawn parameters from vehicle config.

        Returns:
            Dict[str, Any]: Spawn arguments for Scenario.add_vehicle()
            Example: {'pos': (x,y,z), 'rot_quat': (x,y,z,w), ...}
        """
        if "pos" not in self._scenario_args:
            self._scenario_args["pos"] = [0, 0, 0]
        if "rot_quat" not in self._scenario_args:
            self._scenario_args["rot_quat"] = [0, 0, 0, 1]
        if (
            "yaw_angle" in self._scenario_args
            or "pitch_angle" in self._scenario_args
            or "roll_angle" in self._scenario_args
        ):
            yaw_rad = self._scenario_args.get("yaw_angle", 0) * (3.14159 / 180)
            pitch_rad = self._scenario_args.get("pitch_angle", 0) * (3.14159 / 180)
            roll_rad = self._scenario_args.get("roll_angle", 0) * (3.14159 / 180)
            quat_vehicle = convert_euler_to_quaternion((roll_rad, pitch_rad, yaw_rad))
            quat_vehicle = [float(q) for q in quat_vehicle]
            self._scenario_args["rot_quat"] = quat_vehicle
            # Pop the Euler angles
            self._scenario_args.pop("yaw_angle", None)
            self._scenario_args.pop("pitch_angle", None)
            self._scenario_args.pop("roll_angle", None)
        self._logger.debug(
            f"Scenario arguments for vehicle --{self._name}--: \n{self._scenario_args}"
        )
        return self._scenario_args

    @property
    def controllers(self) -> Dict:
        """
        Get the controllers.

        Returns:
            Controllers: The controllers
        """
        return self._controllers

    @property
    def vehicle(self) -> Vehicle:
        """
        Get the vehicle instance.

        Returns:
            Vehicle: The vehicle instance
        """
        return self._vehicle

    def setup_all_sensors(self):
        """
        Setup all the sensors for the vehicle.
        """
        for name, config in self._sensors_config.items():
            # Log the sensor setup
            self.setup_sensor(name, config)
            self._logger.debug(f"Sensor --{name}-- set up for vehicle --{self._name}--")

    def setup_sensor(self, name: str, config: Dict[str, Any]):
        """
        Setup a sensor for the vehicle.

        Args:
            name (str): Sensor name
            config (dict): Sensor configuration dictionary
        """
        sensor_type = config.pop("type")
        try:
            sensor_class = SensorRegistry.get_class(sensor_type)
            sensor = sensor_class(name, self._vehicle, self._beamng, config)
            self._sensors[name] = sensor
        except ValueError:
            self._logger.error(f"Sensor type --{sensor_type}-- not found in registry")

    def poll_sensor(self, name: str):
        """
        Poll a sensor for the latest data.

        Args:
            name (str): Sensor name
        """
        if name in self._sensors:
            self._sensors[name].poll()
        else:
            self._logger.error(
                f"Sensor --{name}-- not found in vehicle --{self._name}--"
            )

    def extract_sensor_ros_msg_type(self, name: str) -> Optional[Any]:
        """
        Extract the ROS message type for a sensor.

        Args:
            name (str): Sensor name

        Returns:
            Any: The ROS message type
        """
        if name in self._sensors:
            return self._sensors[name].ros_msg_type()
        else:
            self._logger.error(
                f"Sensor --{name}-- not found in vehicle --{self._name}--"
            )
            return None

    def extract_sensor_ros_msg(self, name: str) -> Optional[Any]:
        """
        Extract the ROS message for a sensor.

        Args:
            name (str): Sensor name

        Returns:
            Any: The ROS message
        """
        if name in self._sensors:
            return self._sensors[name].to_ros_msg()
        else:
            self._logger.error(
                f"Sensor --{name}-- not found in vehicle --{self._name}--"
            )
            return None

    def get_sensor(self, name: str) -> Optional[SensorBase]:
        """
        Get a sensor instance.

        Args:
            name (str): Sensor name

        Returns:
            SensorBase: The sensor instance
        """
        return self._sensors.get(name, None)

    def setup_controllers(self):
        """Set up controllers for the vehicle."""
        controllers_config = self._config.get("controllers", {})
        for controller_name, controller_config in controllers_config.items():
            self.setup_controller(controller_name, controller_config)

    def setup_controller(self, controller_name: str, controller_config: dict):
        """Set up a specific controller for the vehicle."""
        controller_type = controller_config.get("controllerType")
        if not controller_type:
            self._logger.error(f"Controller type not specified for {controller_name}")
            return

        self._logger.debug(f"{controller_config}")

        try:
            controller_class = ControllerRegistry.get_class(controller_name)
            controller = controller_class(
                controller_name, self._vehicle, self._beamng, controller_config
            )
            self._controllers[controller_name] = controller
            self._logger.info(
                f"Controller {controller_name} of type {controller_type} set up"
            )
        except ValueError as e:
            self._logger.error(f"Failed to set up controller {controller_name}: {e}")

    def start_controller(self, controller_name: str):
        """Start a specific controller."""
        controller = self._controllers.get(controller_name)
        if controller:
            controller.start()
            self._logger.info(f"Controller {controller_name} started")
        else:
            self._logger.error(f"Controller {controller_name} not found")

    def stop_controller(self, controller_name: str):
        """Stop a specific controller."""
        controller = self._controllers.get(controller_name)
        if controller:
            controller.stop()
            self._logger.info(f"Controller {controller_name} stopped")
        else:
            self._logger.error(f"Controller {controller_name} not found")

    def start_all_controllers(self):
        """Start all controllers."""
        for controller_name in self._controllers:
            self.start_controller(controller_name)

    def stop_all_controllers(self):
        """Stop all controllers."""
        for controller_name in self._controllers:
            self.stop_controller(controller_name)
