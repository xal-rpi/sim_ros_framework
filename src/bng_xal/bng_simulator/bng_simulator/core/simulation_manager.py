"""
Manages BeamNG simulation lifecycle, levels, and vehicles.
"""

import traceback

from typing import Dict, List, Optional, Any
from copy import deepcopy
import rclpy
from rclpy.impl.rcutils_logger import RcutilsLogger

import beamngpy
from beamngpy import BeamNGpy, Scenario

from bng_simulator.vehicle.manager import VehicleManager
from bng_simulator.vehicle.sensors import SensorBase
from bng_simulator.core.attach_manager import AttachManager
from bng_simulator.core.request_handler import SimulationRequestHandler
from bng_simulator.core.scenario_builder import ScenarioBuilder
from bng_simulator.utils.config_manager import ConfigManager


class SimulationManager:
    """
    Manages the entire lifecycle of a BeamNG simulation with level-based architecture.
    
    Supports two modes:
    - CREATE: Load level and spawn vehicles
    - ATTACH: Attach to existing level and vehicles
    
    The xlab/xlabCore extension is required and automatically loaded.
    """

    def __init__(self, config: Dict[str, Any], logger: RcutilsLogger):
        """
        Initialize the simulation manager with a configuration.

        Args:
            config (Dict[str, Any]): Simulation configuration dictionary
            logger: ROS logger instance
        """
        self.logger = logger
        
        self.config = deepcopy(config)

        self.beamng: Optional[BeamNGpy] = None
        self.level_name: Optional[str] = None
        self.scenario: Optional[Scenario] = None

        self.vehicles: Dict[str, VehicleManager] = {}

        # Helpers for lifecycle, attach, and request routing
        self.request_handler = SimulationRequestHandler(self)
        self.attach_manager = AttachManager(self)
        self.scenario_builder = ScenarioBuilder(self)

        # Connect to the BeamNG simulation
        self.connect()

        # Get scenario mode from config and initialize
        scenario_mode = self.config.get("scenario_mode", "create").lower()
        
        self.logger.info("=" * 70)
        self.logger.info(f"SCENARIO MODE: {scenario_mode}")
        self.logger.info("=" * 70)
        
        if scenario_mode == "create":
            self.scenario_builder.initialize_create_mode()
        else:
            self.attach_manager.initialize_by_mode(scenario_mode)

        # Setup sensors and controllers for all vehicles after initialization
        self.setup_vehicle_runtime()

        # Apply post-scenario configuration
        self.post_scenario_configuration()

    @classmethod
    def from_file(cls, config: str, logger):
        """
        Create a SimulationManager instance from a configuration file.

        Args:
            config (str): Path to the YAML configuration file

        Returns:
            SimulationManager: Initialized simulation manager
        """
        cfg = ConfigManager.get_config(config)
        if cfg is None:
            raise RuntimeError("Config manager returned an empty config")

        return cls(cfg, logger)
    
    def connect(self):
        """Establish connection with BeamNG simulation."""
        beamng_info = deepcopy(self.config.get("beamng", {}))
        launch_info = beamng_info.pop("open_args", {})
        setup_funcs = beamng_info.pop("setup_funcs", {})

        # ENSURE xlab/xlabCore extension is in extensions list
        extensions = launch_info.get("extensions", [])
        if "xlab/xlabCore" not in extensions:
            extensions.append("xlab/xlabCore")
            self.logger.info("Adding required xlab/xlabCore extension")
        launch_info["extensions"] = extensions

        self.logger.info("Connecting to BeamNG")
        self.logger.debug(f"Connection parameters: \n{beamng_info}")
        self.logger.debug(f"Launch arguments: \n{launch_info}")
        self.logger.debug(f"Extensions: {extensions}")
        self.logger.debug(f"Setup arguments: \n{setup_funcs}")

        self.beamng = BeamNGpy(**beamng_info)
        self.beamng.open(**launch_info)
        if self.beamng is None:
            raise RuntimeError("BeamNG instance didn't connect")

        # Verify xlab extension loaded
        from bng_simulator.core.xlab_api import XlabApi
        if not XlabApi.is_available(self.beamng):
            raise RuntimeError(
                "xlab/xlabCore extension failed to load! "
                "This extension is required for the simulator to function."
            )
        self.logger.info("✓ xlab/xlabCore extension verified")

        # Apply simulator configuration functions
        for func_name, func_args in setup_funcs.items():
            self.request_handler.execute_request(func_name, **func_args)

    def setup_vehicle_runtime(self,):
        """
        Set up sensors and controllers for a vehicle.
        
        Called by managers after vehicle creation/attachment.
        """
        for vehicle_name, vehicle_manager in self.vehicles.items():
            self.logger.info(f"Setting up sensors/controllers for: {vehicle_name}")
            vehicle_manager.setup_all_sensors()
            vehicle_manager.setup_controllers()
        self.logger.info(f"✓ {vehicle_name} setup complete")

    def post_scenario_configuration(self):
        """Functions to apply after scenario is loaded/attached."""
        config = self.config.get("post_scenario", {})
        for func_name, func_args in config.items():
            self.request_handler.execute_request(func_name, **func_args)
    
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

    def get_controller_config(
        self, controller_name: str, vehicle_name: Optional[str] = None
    ) -> Dict[str, Any]:
        if vehicle_name is None:
            vehicle_name = self.default_vehicle_name

        # Get controller config directly from the main config
        vehicles_config = self.config.get("vehicles", {})
        vehicle_config = vehicles_config.get(vehicle_name, {})
        controllers = vehicle_config.get("controllers", {})
        return controllers.get(controller_name, {})

    def replace_vehicle(
        self, 
        vehicle_manager: VehicleManager, 
        connect: bool = True
    ) -> bool:
        """
        Replace an existing vehicle (workaround for color spawn issue).
        
        BeamNGPy has an issue where vehicle colors don't apply correctly
        on initial spawn under a Scenario. This method respawns the vehicle with
        replace=True to ensure colors are rendered properly.
        
        Args:
            vehicle_manager: VehicleManager instance to replace
            connect: If True, reconnect the vehicle after replacement
        
        Returns:
            True if replacement succeeded, False otherwise
        
        Notes:
            - Disconnects vehicle before replacement
            - Preserves all vehicle options and spawn arguments
            - Converts color formats to BeamNG RGBA strings
        """
        from beamngpy.misc.colors import coerce_color, rgba_to_str
        
        vehicle = vehicle_manager.vehicle
        cling = vehicle_manager._spawn_args.get("cling", True)
        
        # Build spawn data packet
        data = dict(type="SpawnVehicle", cling=cling)
        data.update(vehicle.options)
        data["name"] = vehicle.vid
        data["model"] = vehicle.options["model"]
        data["replace"] = True  # Critical: replace existing instead of spawning new
        
        # Convert color formats to BeamNG RGBA string format
        for color in ("color", "color2", "color3"):
            if data.get(color) is not None:
                data[color] = rgba_to_str(coerce_color(data[color]))

        # Send replacement command and wait for confirmation
        resp = self.beamng.vehicles._send(data).recv("VehicleSpawned")
        
        if resp["success"]:
            # Disconnect old connection and optionally reconnect
            if connect:
                vehicle.disconnect()
                vehicle.port = None
                vehicle.connect(self.beamng)
        
        return resp["success"]

    @staticmethod
    def _poll_and_publish(
        node, vehicle_name, sensor_name, sensor, publisher, publish_type
    ):
        """
        Poll sensor and publish data to ROS topic.
        
        Args:
            node: ROS node instance
            vehicle_name: Name of the vehicle
            sensor_name: Name of the sensor
            sensor: Sensor device instance
            publisher: ROS publisher (None if not publishing)
            publish_type: 0=no publish, 1=publish aggregated, >1=publish each record
        """
        try:
            # Poll sensor for latest data
            sensor.poll()
            all_data = sensor.get_all_data()

            # Queue data for logging if logger is configured
            logger_queue = getattr(node, "logger_queue", None)
            if logger_queue and all_data:
                try:
                    logger_queue.put({
                        "vehicle_name": vehicle_name,
                        "sensor_name": sensor_name,
                        "data": all_data,
                    })
                except Exception as e:
                    node.get_logger().error(
                        f"Failed to enqueue logger data for {sensor_name}: {e}"
                    )

            # Publish to ROS topic if publisher exists
            if publisher:
                if publish_type > 1:
                    # Publish each record as separate message (streaming mode)
                    for record in all_data:
                        msg = sensor.to_ros_msg(record)
                        if msg:
                            publisher.publish(msg)
                else:
                    # Publish only the latest message
                    msg = sensor.to_ros_msg()
                    if msg:
                        publisher.publish(msg)
                        
        except KeyboardInterrupt:
            node.get_logger().warn("User interrupt.")
        except Exception as e:
            node.get_logger().error(
                f"Error polling/publishing {vehicle_name}/{sensor_name}: {e}"
            )

    def _validate_poll_time(
        self, node: rclpy.node.Node, veh_name: str, sensor_name: str, poll_time: Any
    ) -> float:
        """
        Validate and sanitize poll_time value.
        
        Args:
            node: ROS node for logging
            veh_name: Vehicle name (for error messages)
            sensor_name: Sensor name (for error messages)
            poll_time: Raw poll_time value from config
            
        Returns:
            Valid poll_time as float (minimum 0.001s, default 0.2s)
        """
        try:
            poll_time = float(poll_time)
        except (ValueError, TypeError):
            node.get_logger().warn(
                f"Invalid poll_time for {veh_name}/{sensor_name} ({poll_time}); "
                f"using default 0.2s"
            )
            return 0.2

        if poll_time <= 0:
            node.get_logger().warn(
                f"Non-positive poll_time for {veh_name}/{sensor_name} ({poll_time}); "
                f"using minimum 0.001s"
            )
            return 0.001

        return poll_time

    def _setup_sensors_for_vehicle(
        self, node: rclpy.node.Node, veh_name: str, sensor_cfg: Dict[str, Any]
    ):
        """
        Configure polling and publishing for all sensors of a specific vehicle.
        
        Args:
            node: ROS node to attach publishers and timers to
            veh_name: Vehicle name
            sensor_cfg: Dict mapping sensor_name -> {topic, poll_time, publish}
        """
        # Validate sensor config format
        if not isinstance(sensor_cfg, dict):
            node.get_logger().error(
                f"Invalid ros_poll_config for vehicle '{veh_name}': "
                f"expected dict, got {type(sensor_cfg).__name__}"
            )
            return

        # Get or create vehicle publisher dict
        veh_pub = node.sensor_publishers.get(veh_name, {})

        for sensor_name, sensor_info in sensor_cfg.items():
            # Clean up existing handles before re-registering
            if sensor_name in veh_pub:
                self._destroy_ros_polling_handles(node, veh_name, sensor_name)

            # Get sensor device instance
            sensor_device = self.get_sensor(sensor_name, veh_name)
            if sensor_device is None:
                node.get_logger().error(
                    f"Sensor '{sensor_name}' not found for vehicle '{veh_name}'"
                )
                continue

            # Validate sensor info format
            if not isinstance(sensor_info, dict):
                node.get_logger().error(
                    f"Invalid config for {veh_name}/{sensor_name}: "
                    f"expected dict, got {type(sensor_info).__name__}"
                )
                continue

            # Extract and validate polling configuration
            topic = sensor_info.get("topic", f"/{veh_name}/{sensor_name}")
            poll_time = self._validate_poll_time(
                node, veh_name, sensor_name, sensor_info.get("poll_time", 0.2)
            )
            publish_mode = sensor_info.get("publish", 0)

            # Create publisher if publishing is enabled
            publisher = None
            if publish_mode > 0:
                msg_type = sensor_device.ros_msg_type()
                if msg_type is None:
                    node.get_logger().error(
                        f"Sensor {veh_name}/{sensor_name} has no ros_msg_type(); "
                        f"cannot create publisher for topic '{topic}'"
                    )
                else:
                    publisher = node.create_publisher(msg_type, topic, 10)

            # Create timer with callback using lambda closure to capture variables
            timer = node.create_timer(
                poll_time,
                lambda v=veh_name, s=sensor_name, sd=sensor_device, 
                       p=publisher, pm=publish_mode: 
                    SimulationManager._poll_and_publish(node, v, s, sd, p, pm),
            )

            # Store handles for cleanup
            veh_pub[sensor_name] = {"pub": publisher, "timer": timer}

        # Update node's sensor publishers dict
        if veh_pub:
            node.sensor_publishers[veh_name] = veh_pub

    def _destroy_ros_polling_handles(
        self, node: rclpy.node.Node, veh_name: str, sensor_name: str
    ) -> None:
        """
        Clean up timer and publisher resources for a sensor.
        
        Safely destroys ROS timer and publisher objects, removing them from tracking dict.
        
        Args:
            node: ROS node containing the resources
            veh_name: Vehicle name
            sensor_name: Sensor name
        """
        sensor_publishers = getattr(node, "sensor_publishers", None)
        if not sensor_publishers:
            return

        veh_pub = sensor_publishers.get(veh_name, {})
        handles = veh_pub.get(sensor_name)
        if not handles:
            return

        # Destroy timer (cancel first, then destroy if available)
        timer = handles.get("timer")
        if timer:
            try:
                timer.cancel()
                node.destroy_timer(timer)  # May not exist in all rclpy versions
            except (AttributeError, Exception):
                pass  # Timer cancelled is sufficient

        # Destroy publisher
        publisher = handles.get("pub")
        if publisher:
            try:
                node.destroy_publisher(publisher)
            except Exception:
                pass

        # Remove from tracking dict
        veh_pub.pop(sensor_name, None)
        if not veh_pub:
            sensor_publishers.pop(veh_name, None)

    def register_ros_polling(self, node: rclpy.node.Node):
        """
        Register ROS publishers and polling timers for all configured sensors.
        
        Reads 'ros_poll_config' from simulation config and sets up:
        - ROS publishers for each sensor
        - Timers to poll sensors at specified rates
        - Optional data logging queue
        
        Config format:
            ros_poll_config:
                vehicle_name:  # or "*" for all vehicles
                    sensor_name:
                        topic: "/topic/name"  # optional, default: /{vehicle}/{sensor}
                        poll_time: 0.1  # seconds between polls
                        publish: 1  # 0=no publish, 1=batch, >1=streaming
        
        Args:
            node: ROS node to attach publishers and timers to
        """
        node.get_logger().debug("Registering ROS polling for sensors")
        
        # Clean up existing resources to prevent leaks on re-registration
        if getattr(node, "sensor_publishers", None):
            for veh_name, veh_pub in list(node.sensor_publishers.items()):
                if isinstance(veh_pub, dict):
                    for sensor_name in list(veh_pub.keys()):
                        self._destroy_ros_polling_handles(node, veh_name, sensor_name)
        
        # Initialize fresh tracking dict
        node.sensor_publishers = {}
        
        # Get polling configuration
        pub_config = self.config.get("ros_poll_config", {})
        if not isinstance(pub_config, dict):
            node.get_logger().error(
                f"Invalid ros_poll_config: expected dict, got {type(pub_config).__name__}"
            )
            return
        
        # Setup sensors for each vehicle
        for veh_name, sensor_cfg in pub_config.items():
            if veh_name == "*":
                # Wildcard: apply configuration to all vehicles
                for actual_veh_name in self.vehicles:
                    self._setup_sensors_for_vehicle(node, actual_veh_name, sensor_cfg)
            else:
                # Specific vehicle
                self._setup_sensors_for_vehicle(node, veh_name, sensor_cfg)
