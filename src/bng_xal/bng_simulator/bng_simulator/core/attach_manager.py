"""
Attach and discovery logic for SimulationManager.

This module handles ATTACH and AUTO modes, which allow connecting to
existing BeamNG levels and vehicles rather than creating new scenarios.

Key features:
- Level discovery via xlab API
- Vehicle discovery and connection
- Flexible vehicle matching (exact name, wildcard, or index-based)
- Mode coordination (create/attach/auto)
"""

import traceback
from typing import Any, Dict, Optional, TYPE_CHECKING

from beamngpy.logging import BNGValueError

from bng_simulator.vehicle.manager import VehicleManager

if TYPE_CHECKING:
    from bng_simulator.core.simulation_manager import SimulationManager


class AttachManager:
    """
    Discover existing levels/vehicles and attach to them.
    Also coordinates initialization across all scenario modes (create/attach/auto).
    
    Responsibilities:
    - Mode orchestration: Route initialization to correct mode handler
    - Level discovery: Query BeamNG for current loaded level
    - Vehicle discovery: Find and connect to existing vehicles
    - Vehicle matching: Map YAML configs to discovered vehicles
    - Extension loading: Ensure xlab/xlabCore is loaded on all vehicles
    
    Matching Strategies:
    1. Exact name match: YAML vehicle name == BeamNG vehicle ID
    2. Wildcard match: Use '*' in YAML to attach all vehicles with same config
    3. Index-based match: Use 'attach_index' to match by discovery order
    """

    def __init__(self, sim_manager: "SimulationManager") -> None:
        """
        Initialize attach manager.
        
        Args:
            sim_manager: Parent SimulationManager instance providing access to
                        BeamNG connection, config, and vehicle registry
        """
        self._sim = sim_manager
        self._logger = sim_manager.logger

    def initialize_by_mode(self, scenario_mode: str) -> None:
        """
        Initialize simulation based on scenario mode.
        
        This is the entry point for mode-based initialization. It routes
        to the appropriate mode handler or delegates to ScenarioBuilder
        for CREATE mode.
        
        Args:
            scenario_mode: One of 'attach', or 'auto'
                - 'attach': Discover and attach to existing level/vehicles
                - 'auto': Try attach first, fallback to create if nothing found
        
        Raises:
            ValueError: If scenario_mode is not recognized
            RuntimeError: If attach mode fails and no fallback configured
        """
        if scenario_mode == "attach":
            self._initialize_attach_mode()
        elif scenario_mode == "auto":
            self._initialize_auto_mode()
        else:
            raise ValueError(
                f"Unknown scenario_mode: {scenario_mode}. "
                f"Expected 'create', 'attach', or 'auto'"
            )

    def discover_level_and_vehicles(self) -> Optional[Dict[str, Any]]:
        """
        Discover current level and vehicles (scenario or freeroam).
        
        This method queries BeamNG to find what's currently loaded:
        1. Current scenario (if any)
        2. Current level via xlab API
        3. All vehicles in the level
        4. Vehicle configurations and properties
        
        Returns:
            Dictionary with 'level' and 'vehicles' keys if discovery succeeds,
            None if no level or vehicles found.
            Format: {"level": str, "vehicles": Dict[str, Vehicle]}
        
        Raises:
            AssertionError: If BeamNG connection not established
            Exception: If discovery process fails unexpectedly
        
        Notes:
            - Automatically loads xlab/xlabCore extension on all vehicles
            - Connects vehicles if not already connected
            - Returns None (not error) if level/vehicles not found
        """
        self._logger.info("-" * 60)
        self._logger.info("STEP 1: DISCOVERING LEVEL & VEHICLES")
        self._logger.info("-" * 60)

        from bng_simulator.core.xlab_api import XlabApi

        try:
            assert self._sim.beamng is not None, "BeamNG connection not established"

            # === Track current scenario instance if any ===
            # try:
            #     self._sim.scenario = self._sim.beamng.get_current_scenario()
            #     if self._sim.scenario:
            #         self._logger.debug(f"Current scenario instance: {self._sim.scenario}")
            # except BNGValueError:
            #     # No scenario loaded (freeroam mode)
            #     self._sim.scenario = None
            self._sim.scenario = None

            # === Get current level via xlab ===
            self._logger.debug("Querying xlab.GetCurrentLevel()...")
            level_name = XlabApi.get_current_level(self._sim.beamng)

            if level_name is None:
                self._logger.info("✗ No level loaded")
                self._logger.info("   Hint: Load a level in BeamNG (freeroam or scenario)")
                return None

            self._logger.info(f"✓ Level detected: {level_name}")

            # === Get info ===
            self._logger.debug("Querying beamng.get_current_vehicles()...")
            vehicles_dict = self._sim.beamng.get_current_vehicles(include_config=True)

            if not vehicles_dict:
                self._logger.info("✗ No vehicles found")
                self._logger.info("   Hint: Spawn a vehicle in BeamNG first")
                return None

            self._logger.info(f"✓ Found {len(vehicles_dict)} vehicle(s):")
            for vid in vehicles_dict.keys():
                self._logger.info(f"   - {vid}")

            # === Connect vehicles + load xlab extension ===
            for vid, veh in vehicles_dict.items():
                self._logger.info(f"Processing vehicle: {vid}")

                # Ensure xlab extension is in the extensions list
                if veh.extensions is None:
                    veh.extensions = []
                # Use set to avoid duplicates
                veh.extensions = list(set(list(veh.extensions) + ["xlab/xlabCore"]))
                self._logger.info(f"  Connecting vehicle: {vid}")
                veh.connect(self._sim.beamng)
                self._logger.info(f"  ✓ Vehicle connected with xlab extension")
                
                # Let's add other relevant variables to the vehicle instance
                partConfig = veh.get_part_config()
                veh.options["partConfig"] = partConfig["partConfigFilename"]

            return {
                "level": level_name,
                "vehicles": vehicles_dict,
            }

        except Exception as e:
            self._logger.error(f"✗ Failed to discover level/vehicles: {e}")
            self._logger.debug(f"   Exception details: {traceback.format_exc()}")
            raise

    def _initialize_attach_mode(self) -> None:
        """
        Initialize in ATTACH mode: discover level + vehicles.
        
        ATTACH mode expects a level and vehicles to already exist in BeamNG.
        If nothing is found and attach_fallback=True, falls back to CREATE mode.
        
        Raises:
            RuntimeError: If no level/vehicles found and attach_fallback=False
        
        Notes:
            - This is a strict mode: expects BeamNG to be in a ready state
            - Use AUTO mode for more flexible initialization
        """
        self._logger.info("Initializing in ATTACH mode...")
        
        level_info = self.discover_level_and_vehicles()
        
        if level_info is None:
            # No level or vehicles found
            fallback = self._sim.config.get("attach_fallback", False)
            if fallback:
                self._logger.warn("No level/vehicles found, falling back to CREATE mode")
                return self._sim.scenario_builder.initialize_create_mode()
            else:
                raise RuntimeError(
                    "ATTACH mode requires a loaded level with vehicles. "
                    "Load a level and spawn vehicles in BeamNG first, or set attach_fallback=True"
                )
        
        # Attach to discovered level and vehicles
        self.attach_to_level(level_info)
        
        self._logger.info("✓ ATTACH mode initialization complete")
    
    def _initialize_auto_mode(self) -> None:
        """
        Initialize in AUTO mode: try attach, fallback to create.
        
        AUTO mode is the most flexible initialization strategy:
        1. Try to discover existing level and vehicles
        2. If found, attach to them (like ATTACH mode)
        3. If not found, create new scenario (like CREATE mode)
        
        This is ideal for development workflows where you might or might
        not have a level pre-loaded in BeamNG.
        
        Notes:
            - Never fails due to missing level/vehicles
            - Always results in a usable simulation state
            - Logs which strategy was used
        """
        self._logger.info("Initializing in AUTO mode...")
        
        level_info = self.discover_level_and_vehicles()
        
        if level_info is not None:
            # Found existing level/vehicles, use ATTACH strategy
            self._logger.info("Level/vehicles detected, using ATTACH mode")
            self.attach_to_level(level_info)
        else:
            # Nothing found, use CREATE strategy
            self._logger.info("No level/vehicles detected, using CREATE mode")
            self._sim.scenario_builder.initialize_create_mode()
        
        self._logger.info("✓ AUTO mode initialization complete")

    def attach_to_level(self, level_info: Dict[str, Any]) -> None:
        """
        Attach to existing level and vehicles.
        
        This method:
        1. Stores the level name
        2. Extracts vehicle information (model, part config)
        3. Matches discovered vehicles to YAML configurations
        4. Creates VehicleManager instances for matched vehicles
        
        Args:
            level_info: Discovery result from discover_level_and_vehicles()
                       Must contain 'level' (str) and 'vehicles' (dict) keys
        
        Notes:
            - Gracefully handles missing vehicle configs in YAML
            - Logs warnings for vehicles that can't be queried
            - No exception if no vehicles match (warns instead)
        """
        self._logger.info("-" * 60)
        self._logger.info("STEP 2: ATTACHING TO LEVEL")
        self._logger.info("-" * 60)

        # === Store level name ===
        self._sim.level_name = level_info["level"]
        self._logger.info(f"Level: {self._sim.level_name}")

        # === Get discovered vehicles (already connected with xlab loaded) ===
        discovered_vehicles_dict = level_info.get("vehicles", {})

        if not discovered_vehicles_dict:
            self._logger.warn("⚠ No vehicles to attach to")
            return

        # === Get vehicle configs from YAML ===
        yaml_vehicles_config = self._sim.config.get("vehicles", {})

        if not yaml_vehicles_config:
            self._logger.warn("⚠ No vehicle configurations in YAML, skipping sensor/controller setup")
            return

        # === Convert to format expected by match_and_attach_vehicles ===
        # Extract model and part config for each discovered vehicle
        discovered_vehicles = {}
        for vid, vehicle_obj in discovered_vehicles_dict.items():
            try:
                # Query vehicle properties from BeamNG
                part_config = vehicle_obj.get_part_config()
                model = part_config.get("partConfigFilename", "Unknown")

                self._logger.debug(f"   Vehicle ID: {vid}, Model: {model}")

                discovered_vehicles[vid] = {
                    "vehicle": vehicle_obj,
                    "model": model,
                    "part_config": part_config,
                }
            except Exception as e:
                # Vehicle exists but can't query properties (shouldn't happen)
                self._logger.warn(f"   ⚠ Failed to query properties for {vid}: {e}")
                discovered_vehicles[vid] = {
                    "vehicle": vehicle_obj,
                    "model": "Unknown",
                    "part_config": {},
                }

        # === Match and attach vehicles ===
        self._logger.info("-" * 60)
        self._logger.info("STEP 3: MATCHING & ATTACHING VEHICLES")
        self._logger.info("-" * 60)

        self._match_and_attach_vehicles(discovered_vehicles, yaml_vehicles_config)

    def _match_and_attach_vehicles(
        self, 
        discovered: Dict[str, Any], 
        yaml_config: Dict[str, Any]
    ) -> None:
        """
        Match YAML vehicle configs to discovered vehicles and attach.
        
        Matching strategies (in order of priority):
        1. Wildcard ('*'): Apply same config to all discovered vehicles
        2. Exact name: Match YAML key to vehicle ID in BeamNG
        3. Index-based: Use 'attach_index' in config to match by order
        
        Args:
            discovered: Dict mapping vehicle IDs to their info dicts
                       (vehicle, model, part_config)
            yaml_config: Dict mapping YAML vehicle names to their configs
        
        Notes:
            - Logs warnings for unmatched vehicles
            - Provides hints for common matching issues
            - Wildcard match applies to ALL vehicles (no other matches processed)
        """
        self._logger.debug(f"Discovered vehicles: {list(discovered.keys())}")
        self._logger.debug(f"YAML vehicle configs: {list(yaml_config.keys())}")

        # === Strategy 1: Wildcard matching ===
        # If '*' present, apply same config to all vehicles
        if "*" in yaml_config:
            self._logger.info("Wildcard match detected: attaching to ALL vehicles")
            wildcard_config = yaml_config["*"]
            for vid in discovered:
                self._attach_single_vehicle(vid, vid, discovered[vid], wildcard_config)
            return

        # === Strategy 2 & 3: Name-based and index-based matching ===
        for yaml_name, yaml_veh_config in yaml_config.items():
            self._logger.info(f"Matching YAML vehicle: {yaml_name}")

            # Try exact name match first
            if yaml_name in discovered:
                self._logger.info(f"  ✓ Exact match found: {yaml_name}")
                self._attach_single_vehicle(
                    yaml_name, yaml_name, discovered[yaml_name], yaml_veh_config
                )
                continue

            # Try index-based match if specified
            if "attach_index" in yaml_veh_config:
                idx = yaml_veh_config["attach_index"]
                discovered_list = list(discovered.keys())

                if idx < len(discovered_list):
                    actual_vid = discovered_list[idx]
                    self._logger.info(f"  ✓ Index match found: {yaml_name} -> {actual_vid}")
                    self._attach_single_vehicle(
                        yaml_name, actual_vid, discovered[actual_vid], yaml_veh_config
                    )
                    continue
                else:
                    self._logger.warn(
                        f"  ✗ Index {idx} out of range (only {len(discovered_list)} vehicles)"
                    )

            # No match found - provide helpful feedback
            self._logger.warn(f"  ✗ No match found for {yaml_name}")
            self._logger.info(f"     Available vehicles: {list(discovered.keys())}")
            self._logger.info(
                f"     Hint: Check vehicle name in BeamNG or use attach_index"
            )

    def _attach_single_vehicle(
        self,
        yaml_name: str,
        actual_vid: str,
        discovered_info: Dict[str, Any],
        yaml_config: Dict[str, Any],
    ) -> None:
        """
        Attach to a single discovered vehicle.
        
        Creates a VehicleManager instance from the existing BeamNG vehicle
        and registers it in the simulation manager's vehicle registry.
        
        Args:
            yaml_name: Name to use in the vehicle registry (from YAML config)
            actual_vid: Actual vehicle ID in BeamNG
            discovered_info: Vehicle info dict with 'vehicle', 'model', 'part_config'
            yaml_config: YAML configuration for sensors, controllers, etc.
        
        Raises:
            Exception: If vehicle attachment fails (logged and re-raised)
        
        Notes:
            - yaml_name may differ from actual_vid (for index-based matching)
            - VehicleManager.from_existing_vehicle handles connection details
            - Sensors and controllers setup happens later in setup_vehicle_runtime()
        """
        self._logger.info(f"Attaching to vehicle: {actual_vid} (as '{yaml_name}')")

        try:
            vehicle = discovered_info["vehicle"]
            
            # Create VehicleManager from existing vehicle
            # This preserves the existing connection and adds our config
            vehicle_manager = VehicleManager.from_existing_vehicle(
                name=yaml_name,
                beamng=self._sim.beamng,
                existing_vehicle=vehicle,
                config=yaml_config,
            )

            # Register in simulation manager's vehicle registry
            self._sim.vehicles[yaml_name] = vehicle_manager
            self._logger.info(f"  ✓ Successfully attached to {actual_vid}")
            
            # Replace vehicle at current position (workaround for proper initialization)
            self._logger.info(f"  Replacing vehicle at current position...")
            
            success = self._sim.replace_vehicle(vehicle_manager)
            if success:
                self._logger.info(f"  ✓ Vehicle replaced successfully")
            else:
                self._logger.warn(f"  ⚠ Vehicle replacement failed, continuing anyway")

        except Exception as e:
            self._logger.error(f"  ✗ Failed to attach to {actual_vid}: {e}")
            self._logger.debug(f"     Exception details: {traceback.format_exc()}")
            raise
