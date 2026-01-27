"""
Vehicle management class for BeamNG simulation with ROS integration.
"""

from typing import Dict, Any, Optional
from copy import deepcopy

from beamngpy import Vehicle, BeamNGpy
from bng_simulator.vehicle.sensors import SensorBase, SensorRegistry
from bng_simulator.vehicle.controllers.base import ControllerRegistry

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
    
    @classmethod
    def from_existing_vehicle(cls, name: str, beamng: BeamNGpy,
                              existing_vehicle: Vehicle, config: Dict[str, Any]):
        """
        Create VehicleManager from an existing BeamNG vehicle (attach mode).
        
        Args:
            name: Internal name for this vehicle manager
            beamng: BeamNG connection
            existing_vehicle: Vehicle object from scenario.get_current(connect=True)
            config: Partial YAML config (sensors, controllers only)
        
        Returns:
            VehicleManager instance
        """
        logger = get_logger(__name__)
        logger.info(f"Creating VehicleManager from existing vehicle: {name}")
        
        # Create instance manually (bypass normal __init__)
        instance = cls.__new__(cls)
        instance._name = name
        instance._beamng = beamng
        instance._config = deepcopy(config)
        instance._sensors = {}
        instance._controllers = {}
        instance._logger = logger
        
        # Use existing vehicle instead of creating new one
        instance._vehicle = existing_vehicle
        logger.info(f"  Using existing vehicle object: {existing_vehicle.vid}")
        logger.info(f"  Vehicle connected: {existing_vehicle.is_connected()}")
        
        # Extract sensor/controller configs (spawn args not used in attach mode)
        # NOTE: Keep this explicit; attach mode must NOT try to reconstruct Vehicle() args.
        instance._sensors_config = instance._config.pop("sensors", {})
        instance._controllers_config = instance._config.pop("controllers", {})
        instance._spawn_args = {}  # Not used in attach mode
        
        logger.info(f"  ✓ VehicleManager created for existing vehicle")
        logger.info(f"  Sensors to attach: {list(instance._sensors_config.keys())}")
        logger.info(f"  Controllers to attach: {list(instance._controllers_config.keys())}")
        
        return instance

    def create_vehicle_instance(self) -> Vehicle:
        """
        Create a vehicle instance based on the configuration.

        Returns:
            Vehicle: The created vehicle instance
        """
        # Extract sub-configs that are NOT part of the BeamNGpy Vehicle constructor.
        # This is required so we can use VehicleManager in CREATE mode (scenario-based).
        self._sensors_config = self._config.pop("sensors", {})
        self._controllers_config = self._config.pop("controllers", {})
        self._spawn_args = self._config.pop("spawn", {})

        # Ensure xlab/xlabCore is always enabled for our custom sensors/controllers.
        # If the user provided extensions, merge them rather than overwriting.
        model_args = self._config["model_args"]
        extensions = list(model_args.get("extensions", []) or [])
        if "xlab/xlabCore" not in extensions:
            extensions.append("xlab/xlabCore")
        model_args["extensions"] = extensions

        # Now log the remaining configuration (Vehicle args only)
        self._logger.debug(f"Vehicle --{self._name}-- configuration: \n{self._config}")
        self._logger.debug(f"Vehicle --{self._name}-- extensions: {extensions}")

        # Construct the BeamNGpy Vehicle instance
        return Vehicle(self._name, **self._config["model_args"])

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
        controllers_config = getattr(self, "_controllers_config", {})
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
