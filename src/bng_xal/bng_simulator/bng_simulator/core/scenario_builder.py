"""
Scenario creation flow for SimulationManager.

This module handles CREATE mode initialization, which involves:
- Creating and configuring scenarios
- Spawning vehicles with proper settings
- Adding level objects (cones, barriers, etc.)
- Handling vehicle color spawn workarounds
"""

import traceback
from copy import deepcopy
from typing import Any, Dict, Optional, TYPE_CHECKING

import beamngpy
from beamngpy import Scenario

from bng_simulator.core.environment_assets import load_environment_assets
from bng_simulator.core.xlab_api import XlabApi
from bng_simulator.vehicle.manager import VehicleManager
from bng_simulator.utils.math_op import process_euler_to_quat

if TYPE_CHECKING:
    from bng_simulator.core.simulation_manager import SimulationManager


class ScenarioBuilder:
    """
    Build and launch scenarios in CREATE mode.
    
    Responsibilities:
    - Scenario lifecycle management (create, configure, start)
    - Vehicle spawning and configuration
    - Level object placement
    - Workarounds for BeamNG quirks (color spawn issues)
    """

    def __init__(self, sim_manager: "SimulationManager") -> None:
        """
        Initialize scenario builder.
        
        Args:
            sim_manager: Parent SimulationManager instance providing access to
                        BeamNG connection, config, and vehicle registry
        """
        self._sim = sim_manager
        self._logger = sim_manager.logger

    def close_existing_scenario_if_any(self) -> None:
        """Return to the main menu only when a level is loaded.

        The level query also handles Freeroam and an empty main menu. A query or
        stop failure propagates; it must not be reported as an absent scenario.
        """
        assert self._sim.beamng is not None
        if not XlabApi.get_current_level(self._sim.beamng):
            self._logger.info("No existing scenario found.")
            return
        self._logger.info("Stopping the existing scenario...")
        self._sim.beamng.scenario._send({"type": "StopScenario"}).ack("ScenarioStopped")
        self._logger.info("Existing scenario stopped.")

    def add_level_objects(
        self, 
        objects_config: Dict[str, Any], 
        scenario: Optional[Scenario] = None
    ) -> None:
        """
        Add objects to the current level (cones, barriers, procedural objects, etc).
        
        Args:
            objects_config: Dictionary mapping object names to their configuration.
                           Each config must include a 'type' key matching a beamngpy class.
            scenario: Scenario instance to add objects to. If None, objects are skipped.
        
        Notes:
            - Supports Euler angle rotation (automatically converted to quaternions)
            - Validates object types against beamngpy module
            - Uses deepcopy to prevent config mutations
        """
        if not objects_config:
            return
        
        self._logger.info(f"Adding {len(objects_config)} object(s) to level")
        
        for obj_name, obj_cfg in objects_config.items():
            # Deep copy to avoid mutating original config
            obj_cfg = deepcopy(obj_cfg)
            
            # Extract and validate object type
            obj_type = obj_cfg.pop("type", None)
            if obj_type is None:
                self._logger.error(f"Object type not specified for {obj_name}")
                continue
            if not hasattr(beamngpy, obj_type):
                self._logger.error(f"Object type not found in beamngpy: {obj_type}")
                continue
            
            # Get the class constructor from beamngpy module
            obj_class = getattr(beamngpy, obj_type)
            
            # Handle rotation: convert Euler angles to quaternions if provided
            rot_args = obj_cfg.pop("rot", {})
            process_euler_to_quat(rot_args, self._sim._PI / 180)
            if "rot_quat" in rot_args:
                obj_cfg["rot_quat"] = rot_args["rot_quat"]
            
            # Set required object properties with defaults
            obj_cfg["name"] = obj_name
            obj_cfg["pos"] = tuple(obj_cfg.get("pos", (0, 0, 0)))
            obj_cfg["scale"] = tuple(obj_cfg.get("scale", (1, 1, 1)))
            
            # Create and add object instance
            obj = obj_class(**obj_cfg)
            self._logger.debug(f"Created level object: {obj.__dict__}")
            
            if scenario is not None:
                scenario.add_object(obj)
                self._logger.info(f"Added object '{obj_name}' to scenario")

    def initialize_create_mode(self) -> None:
        """
        Initialize in CREATE mode (scenario-based).
        
        CREATE mode workflow:
        1. Validate scenario configuration
        2. Close any existing scenarios
        3. Create new scenario instance
        4. Add vehicles with spawn configurations
        5. Add level objects (cones, barriers, etc.)
        6. Make and load scenario
        7. Respawn vehicles to fix color rendering issue
        
        Raises:
            RuntimeError: If scenario configuration is missing
            AssertionError: If BeamNG connection not established
        
        Notes:
            - Requires 'scenario' section in config with level and name
            - All vehicles must have 'spawn' configuration
            - Vehicle colors are fixed via replace workaround
        """
        self._logger.info("Initializing in CREATE mode...")

        # === STEP 1: Validate Configuration ===
        # CREATE mode is scenario-based and requires explicit scenario config
        scenario_config = deepcopy(self._sim.config.get("scenario", {}))
        if not scenario_config:
            raise RuntimeError(
                "CREATE mode requires a 'scenario' section in the config. "
                "Example: scenario: {level: tech_ground, name: my_scenario}"
            )

        if self._sim.config.get("environment", {}).get("trackbuilder_tracks"):
            # Spawning assets into an existing scene is deliberately CREATE-only.
            # Every fresh scenario load owns a clean set of additions.
            self._logger.info("Environment additions will load before simulation starts")

        # === STEP 2: Clean Slate ===
        # Stop any existing scenario to avoid inconsistent simulation state
        self.close_existing_scenario_if_any()

        # === STEP 3: Create Scenario ===
        self._logger.info(f"Creating scenario: \n{scenario_config}")
        self._sim.scenario = Scenario(**scenario_config)

        # === STEP 4: Add Vehicles ===
        # Vehicles must be added to scenario before making/loading it
        vehicles_config = self._sim.config.get("vehicles", {})
        for vehicle_name, vehicle_config in vehicles_config.items():
            # Create vehicle manager instance
            vehicle_manager = VehicleManager(vehicle_name, self._sim.beamng, vehicle_config)
            self._sim.vehicles[vehicle_name] = vehicle_manager

            # Process spawn arguments (Euler angles → quaternions)
            spawn_args = deepcopy(vehicle_config["spawn"])
            process_euler_to_quat(spawn_args)

            # Add vehicle to scenario (not yet spawned)
            self._sim.scenario.add_vehicle(vehicle_manager.vehicle, **spawn_args)
            self._logger.info(f"Added vehicle '{vehicle_name}' to scenario")

        # === STEP 5: Add Level Objects ===
        # Optional: cones, barriers, procedural meshes, etc.
        extra_objects_config = self._sim.config.get("extra_objects", {})
        self.add_level_objects(extra_objects_config, scenario=self._sim.scenario)

        # === STEP 6: Make and Load Scenario ===
        # This writes scenario files and loads them into BeamNG
        assert self._sim.beamng is not None, "BeamNG connection not established"
        assert self._sim.scenario is not None, "Scenario instance not created"
        self._sim.scenario.make(self._sim.beamng)  # Write scenario files
        self._sim.beamng.load_scenario(self._sim.scenario)  # Load into simulation
        load_environment_assets(self._sim.beamng, self._sim.config, self._logger)
        self._sim.beamng.scenario.start()  # Start after track collision is ready

        self._logger.info("Scenario created and started successfully")

        # === STEP 7: Fix Color Spawn Issue ===
        # BeamNG bug: colors don't apply on first spawn, must replace vehicles
        for vehicle_name, vehicle_manager in self._sim.vehicles.items():
            self._sim.replace_vehicle(vehicle_manager)
            self.restore_explicit_spawn(vehicle_manager)

        # Settle + sensors happen in setup_vehicle_runtime (after replace, before GtState).
        self._logger.info("✓ CREATE mode initialization complete")

    def restore_explicit_spawn(self, vehicle_manager: VehicleManager) -> None:
        """Restore an airborne/elevated spawn after scene loading and replacement.

        BeamNGpy does not serialize cling=False into the initial vehicle prefab.
        Replacement preserves the current pose, which may already be on terrain.
        Teleport after environment geometry is loaded, before settling/sensors,
        so an explicit Z remains meaningful for tracks above the base terrain.
        """
        spawn = vehicle_manager.get_spawn_args()
        if spawn.get("cling", True):
            return
        process_euler_to_quat(spawn)
        pos = tuple(spawn["pos"])
        rot = tuple(spawn.get("rot_quat", (0, 0, 0, 1)))
        if not vehicle_manager.vehicle.teleport(pos, rot_quat=rot, reset=True):
            raise RuntimeError(f"Could not restore explicit spawn for {vehicle_manager.vehicle.vid}")
        self._logger.info(
            f"Restored explicit spawn for {vehicle_manager.vehicle.vid}: pos={pos}"
        )
