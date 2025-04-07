"""
Manages BeamNG simulation lifecycle, scenarios, and vehicles.
"""

import logging
import traceback

from typing import Dict, List, Optional, Any
from copy import deepcopy

from beamngpy import BeamNGpy, Scenario
from beamngpy.logging import BNGValueError

from bng_simulator.utils.io_dict_utils import load_yaml

from bng_simulator.vehicle.manager import VehicleManager
from bng_simulator.vehicle.sensors import SensorBase
from bng_simulator.utils.math_op import convert_euler_to_quaternion
import bng_simulator.core.vehicle_properties as vehicle_queries

PI = 3.14159


class SimulationManager:
    """
    Manages the entire lifecycle of a BeamNG simulation on a scenario
    with multiple vehicles.
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the simulation manager with a configuration.

        Args:
            config (Dict[str, Any]): Simulation configuration dictionary
        """
        self.logger = logging.getLogger("SimulationManager")
        self.config = deepcopy(config)

        self.beamng: Optional[BeamNGpy] = None
        self.scenario: Optional[Scenario] = None

        self.vehicles: Dict[str, VehicleManager] = {}

        # Connect to the BeamNG simulation
        self.connect()

        # Check for existing scenario and close it if any
        self.close_existing_scenario_if_any()

        # Create the scenario
        self.create_scenario()

        # Apply post scenario configuration
        self.post_scenario_configuration()

    @classmethod
    def from_file(cls, config_path: str):
        """
        Create a SimulationManager instance from a configuration file.

        Args:
            config_path (str): Path to the YAML configuration file

        Returns:
            SimulationManager: Initialized simulation manager
        """
        config = load_yaml(config_path)
        return cls(config)

    def connect(self):
        """Establish connection with BeamNG simulation."""
        beamng_info = deepcopy(self.config.get("beamng", {}))
        launch_info = beamng_info.pop("open_args", {})
        setup_funcs = beamng_info.pop("setup_funcs", {})

        self.logger.info("Connecting to BeamNG")
        self.logger.info(f"Connection parameters: \n{beamng_info}")
        self.logger.info(f"Launch arguments: \n{launch_info}")

        self.beamng = BeamNGpy(**beamng_info)
        self.beamng.open(**launch_info)

        # Apply simulator configuration functions
        for func_name, func_args in setup_funcs.items():
            self.execute_request(func_name, **func_args)

    def create_scenario(self):
        """
        Create and load the scenario based on configuration.
        """
        scenario_config = self.config.get("scenario", {})
        self.logger.info(f"Creating scenario: \n{scenario_config}")

        # Create new scenario
        self.scenario = Scenario(**scenario_config)

        # Add vehicles
        vehicles_config = self.config.get("vehicles", {})
        for vehicle_name, vehicle_config in vehicles_config.items():
            self.add_vehicle(vehicle_name, vehicle_config)

        # Make and load scenario
        self.scenario.make(self.beamng)
        self.beamng.load_scenario(self.scenario)
        self.beamng.scenario.start()

        self.logger.info("Scenario created and loaded successfully")

        # Intialize the sensors for each vehicle
        for vehicle_name in self.vehicles:
            vehicle_manager = self.vehicles[vehicle_name]
            vehicle_manager.setup_all_sensors()

    def add_vehicle(self, vehicle_name: str, vehicle_config: Dict[str, Any]):
        """
        Add a vehicle to the scenario.

        Args:
            vehicle_name (str): Name of the vehicle
            vehicle_config (Dict[str, Any]): Vehicle configuration
        """
        vehicle = VehicleManager(vehicle_name, self.beamng, vehicle_config)
        self.vehicles[vehicle_name] = vehicle
        # Get scenario spawn parameters from vehicle config
        spawn_args = vehicle.get_scenario_args()
        self.scenario.add_vehicle(vehicle.vehicle, **spawn_args)

    def close_existing_scenario_if_any(self):
        """Close any existing scenario if any."""
        try:
            old_scenario = self.beamng.get_current_scenario()
            if old_scenario:
                self.logger.info("Stopping the existing scenario...")
                data = dict(type="StopScenario")
                self.beamng.scenario._send(data).ack("ScenarioStopped")
                self.logger.info("Existing scenario stopped.\n")
            else:
                self.logger.info("No existing scenario found.\n")
        except BNGValueError:
            self.logger.info("No existing scenario found.\n")

    def proxy_for_vehicle_properties(
        self, property_name: str, vehicle_name: Optional[str] = None, **kwargs
    ) -> Dict[str, Any]:
        """
        Proxy for vehicle properties query functions.

        Args:
            vehicle_name (str): Name of the vehicle
            property_name (str): Name of the property query function
            **kwargs: Additional arguments for the query function

        Returns:
            Dict[str, Any]: Result of the query function
        """
        if vehicle_name is None:
            vehicle_name = self.default_vehicle_name
        vehicle = self.vehicles[vehicle_name].vehicle
        query_func = getattr(vehicle_queries, property_name, None)

        if query_func is None:
            err_msg = f"Query function not found: {property_name}\n"
            self.logger.error(err_msg)
            return {"error": err_msg}

        # Execute the query function, need to catch invalid keyword arguments
        try:
            self.logger.info(f"Attempting {property_name} for vehicle: {vehicle_name}")
            query_output = query_func(vehicle, **kwargs)
        except TypeError:
            self.logger.error(f"Error executing query function: {property_name}")
            err_msg = "Error:\n" + traceback.format_exc()
            self.logger.error(err_msg)
            return {"error": err_msg}
        self.logger.info("Query executed successfully.")
        # log the query started and the output
        self.logger.debug(f"Query output: \n{query_output}")
        return query_output if query_output is not None else {}

    def teleport_vehicle(
        self,
        vehicle_name: Optional[str] = None,
        pos: Optional[List[float]] = None,
        rot_quat: Optional[List[float]] = None,
        reset: bool = True,
        **kwargs,
    ):
        """
        Teleport a vehicle to a new position and rotation.
        """
        if vehicle_name is None:
            vehicle_name = self.default_vehicle_name
        vehicle_manager = self.vehicles[vehicle_name]
        vehicle = vehicle_manager.vehicle
        scenario_args = vehicle_manager.get_scenario_args()
        if pos is None:
            pos = scenario_args.get("pos", [0, 0, 0])
        if rot_quat is None:
            rot_quat = scenario_args.get("rot_quat", [0, 0, 0, 1.0])
        # Check if euler angles are provided
        if "yaw_angle" in kwargs or "pitch_angle" in kwargs or "roll_angle" in kwargs:
            yaw_rad = kwargs.get("yaw_angle", 0) * (PI / 180)
            pitch_rad = kwargs.get("pitch_angle", 0) * (PI / 180)
            roll_rad = kwargs.get("roll_angle", 0) * (PI / 180)
            rot_quat = convert_euler_to_quaternion(
                (roll_rad, pitch_rad, yaw_rad)
            )  # TODO : fix type mistmatch
            rot_quat = [float(q) for q in rot_quat]
        # Teleport the vehicle
        succeed = vehicle.teleport(pos, rot_quat, reset=reset)
        return {"success": succeed}

    def post_scenario_configuration(self):
        """
        A set of function to apply after the scenario is created.
        """
        config = self.config.get("post_scenario", {})
        for func_name, func_args in config.items():
            # Execute the request
            self.execute_request(func_name, **func_args)

    def execute_request(self, func_name: str, **func_args):
        """
        Execute a request on the simulation manager.
        The function name could be either a method of the SimulationManager (other than execute_request).
        Either a proxy_for_vehicle_properties or a method of the beamng instance starting with beamng.

        Returns:
            Any: The result of the function call. Mostly a dictionary.
        """
        # Log the request
        self.logger.info(f"Executing request: {func_name} with args: \n{func_args}")
        # Check if the function is a method of the SimulationManager
        # other than execute_request or proxy_for_vehicle_properties
        if hasattr(self, func_name) and func_name not in [
            "execute_request",
            "proxy_for_vehicle_properties",
        ]:
            method = getattr(self, func_name)
            out = method(**func_args)
            self.logger.info(f"Request executed successfully.")
            out_mod = {} if out is None else out
            return out_mod if isinstance(out_mod, dict) else {"result": out_mod}

        # If a beamng method is requested
        if func_name.startswith("beamng."):
            path_components = func_name.split(".")
            current_obj = self.beamng
            for component in path_components[1:]:
                current_obj = getattr(current_obj, component)
            out = current_obj(**func_args)
            out_mod = {} if out is None else out
            self.logger.info(f"Request executed successfully.")
            return out_mod if isinstance(out_mod, dict) else {"result": out_mod}

        # We assume it is a proxy for vehicle properties
        # this will return an error if the function is not found
        return self.proxy_for_vehicle_properties(func_name, **func_args)

    @property
    def default_vehicle_name(self) -> str:
        """
        Get the default vehicle name.

        Returns:
            str: Default vehicle name
        """
        first_vehicle_name = next(iter(self.vehicles), None)
        assert first_vehicle_name is not None, "No vehicles found in the scenario"
        return self.config.get("default_vehicle", first_vehicle_name)

    def get_available_vehicles(self) -> List[str]:
        """
        Get the list of available vehicles.

        Returns:
            List[str]: List of vehicle names
        """
        return list(self.vehicles.keys())

    def get_vehicle_part_config(self, vehicle_name: str = None) -> str:
        """
        Get the vehicle part configuration name.

        Args:
            vehicle_name (str): Name of the vehicle

        Returns:
            str: Vehicle part configuration name
        """
        if vehicle_name is None:
            vehicle_name = self.default_vehicle_name
        vehicle_manager = self.vehicles[vehicle_name]
        vehicle = vehicle_manager.vehicle
        return vehicle.get_part_config()

    def get_sim_config(
        self,
    ) -> Dict[str, str]:
        """
        Get the simulation configuration.

        Returns:
            Dict[str, str]: Simulation configuration
        """
        _config = deepcopy(self.config)
        # Let's get the part config for each vehicle
        _config["vehicles_part"] = {}
        for vehicle_name in self.vehicles:
            part_config = self.get_vehicle_part_config(vehicle_name)
            # This is of the form "vehicles/vehicleName/model_name.pc"
            config_name = part_config["partConfigFilename"]
            veh_model_name = config_name.split("/")[-2:]
            veh_model_name = "_".join(veh_model_name)
            veh_model_name = veh_model_name.replace(".pc", "")
            _config["vehicles_part"][vehicle_name] = veh_model_name
        return _config

    def poll_sensor(self, sensor_name: str, vehicle_name: Optional[str] = None):
        """
        Poll a sensor for the latest data.

        Args:
            sensor_name (str): Name of the sensor
            vehicle_name (str): Name of the vehicle
        """
        if vehicle_name is None:
            vehicle_name = self.default_vehicle_name
        vehicle_manager = self.vehicles[vehicle_name]
        vehicle_manager.poll_sensor(sensor_name)

    def extract_sensor_ros_msg_type(
        self, sensor_name: str, vehicle_name: Optional[str] = None
    ) -> Optional[Any]:
        """
        Extract the ROS message type for a sensor.

        Args:
            sensor_name (str): Name of the sensor
            vehicle_name (str): Name of the vehicle

        Returns:
            Any: The ROS message type
        """
        if vehicle_name is None:
            vehicle_name = self.default_vehicle_name
        vehicle_manager = self.vehicles[vehicle_name]
        return vehicle_manager.extract_sensor_ros_msg_type(sensor_name)

    # def control_vehicle(
    #     self,
    #     vehicle_name: str = None,
    #     steering: Optional[float] = None,
    #     throttle: Optional[float] = None,
    #     brake: Optional[float] = None,
    #     parkingbrake: Optional[float] = None,
    #     clutch: Optional[float] = None,
    #     gear: Optional[int] = None,
    # ):
    #     """
    #     Utility function to send desired input to the vehicle.
    #     """
    #     if vehicle_name is None:
    #         vehicle_name = self.default_vehicle_name
    #     vehicle: Vehicle = self.vehicles[vehicle_name].vehicle
    #     vehicle.control(
    #         steering=steering,
    #         throttle=throttle,
    #         brake=brake,
    #         parkingbrake=parkingbrake,
    #         clutch=clutch,
    #         gear=gear,
    #     )
    #     return {"success": True}

    def extract_sensor_ros_msg(
        self, sensor_name: str, vehicle_name: Optional[str] = None
    ) -> Optional[Any]:
        """
        Extract the ROS message for a sensor.

        Args:
            sensor_name (str): Name of the sensor
            vehicle_name (str): Name of the vehicle

        Returns:
            Any: The ROS message
        """
        if vehicle_name is None:
            vehicle_name = self.default_vehicle_name
        vehicle_manager = self.vehicles[vehicle_name]
        return vehicle_manager.extract_sensor_ros_msg(sensor_name)

    def get_sensor(
        self, sensor_name: str, vehicle_name: Optional[str] = None
    ) -> Optional[SensorBase]:
        """
        Get a sensor instance.

        Args:
            sensor_name (str): Name of the sensor
            vehicle_name (str): Name of the vehicle

        Returns:
            Any: The sensor instance
        """
        if vehicle_name is None:
            vehicle_name = self.default_vehicle_name
        vehicle_manager = self.vehicles[vehicle_name]
        return vehicle_manager.get_sensor(sensor_name)


# # Let's create a quick main for testing
# if __name__ == "__main__":
#     import sys
#     import logging
#     logging.basicConfig(stream=sys.stdout, level=logging.INFO)


#     # Load the simulation configuration
#     config_path = "../../config/scenarios/basic_scenario.yaml"
#     sim_manager = SimulationManager.from_file(config_path)
#     print("Simulation manager created successfully.")
#     print(f"Scenario: {sim_manager.scenario}")
#     print(f"Vehicles: {sim_manager.vehicles}")
#     print("Simulation manager ready for use.")
