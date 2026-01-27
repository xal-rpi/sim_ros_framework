"""
Request routing for SimulationManager.
"""

import traceback
from typing import Any, Dict, Optional, TYPE_CHECKING

import bng_simulator.core.vehicle_properties as vehicle_queries

if TYPE_CHECKING:
    from bng_simulator.core.simulation_manager import SimulationManager

class SimulationRequestHandler:
    """
    Handle request dispatch for SimulationManager.

    Keeps request routing isolated from lifecycle management.
    """

    def __init__(self, sim_manager: "SimulationManager"):
        self._sim = sim_manager
        self._logger = sim_manager.logger
        self._PI = 3.14159

    def teleport_vehicle(
        self,
        vehicle_name: str = None,
        pos: list = None,
        rot_quat: list = None,
        reset: bool = True,
        **kwargs,
    ):
        """
        Teleport a vehicle to a new position and rotation.
        
        Args:
            vehicle_name: Name of the vehicle (uses default if None)
            pos: Position [x, y, z] (uses spawn position if None)
            rot_quat: Rotation quaternion [x, y, z, w] (uses spawn rotation if None)
            reset: Whether to reset vehicle state on teleport
            **kwargs: Optional euler angles (yaw_angle, pitch_angle, roll_angle in degrees)
        
        Returns:
            dict: {"success": bool}
        """
        from bng_simulator.utils.math_op import process_euler_to_quat
        
        if vehicle_name is None:
            vehicle_name = self._sim.default_vehicle_name
            
        vehicle_manager = self._sim.vehicles[vehicle_name]
        vehicle = vehicle_manager.vehicle
        spawn_args = vehicle_manager.get_spawn_args()
        
        # Use spawn position/rotation as defaults
        if pos is None:
            pos = spawn_args.get("pos", [0, 0, 0])
        if rot_quat is None:
            rot_quat = spawn_args.get("rot_quat", [0, 0, 0, 1.0])
            
        # Convert euler angles to quaternion if provided using utility function
        process_euler_to_quat(kwargs, deg_to_rad_factor=self._PI / 180)
        if "rot_quat" in kwargs:
            rot_quat = kwargs["rot_quat"]
            
        # Execute teleport
        succeed = vehicle.teleport(pos, rot_quat, reset=reset)
        return {"success": succeed}

    def get_vehicle_part_config(self, vehicle_name: str = None):
        """
        Get the vehicle part configuration.
        
        Args:
            vehicle_name: Name of the vehicle (uses default if None)
        
        Returns:
            dict: Vehicle part configuration from BeamNG
        """
        if vehicle_name is None:
            vehicle_name = self._sim.default_vehicle_name
            
        vehicle_manager = self._sim.vehicles[vehicle_name]
        vehicle = vehicle_manager.vehicle
        return {**vehicle.get_part_config(), "model": vehicle.options.get("model")}

    def get_sim_config(self):
        """
        Get the complete simulation configuration including vehicle part configs.
        
        Returns:
            dict: Simulation configuration with vehicles_part mapping
        """
        from copy import deepcopy
        
        _config = deepcopy(self._sim.config)
        # Add part config for each vehicle
        _config["vehicles_part"] = {}
        _config["vehicles_status"] = {}
        for vehicle_name in self._sim.vehicles:
            part_config = self.get_vehicle_part_config(vehicle_name)
            # Extract model name from "vehicles/vehicleName/model_name.pc"
            config_name = part_config["partConfigFilename"]
            veh_model_name = config_name.split("/")[-2:]
            veh_model_name = "_".join(veh_model_name)
            veh_model_name = veh_model_name.replace(".pc", "")
            _config["vehicles_part"][vehicle_name] = veh_model_name

            # Best-effort snapshot of vehicle safety/drivetrain state
            try:
                _config["vehicles_status"][vehicle_name] = self.get_vehicle_safety_and_drivetrain_status(
                    vehicle_name=vehicle_name
                )
            except Exception as e:
                _config["vehicles_status"][vehicle_name] = {"error": str(e)}
        return _config

    def get_vehicle_config(self, vehicle_name: Optional[str] = None) -> Dict[str, Any]:
        """Return the raw YAML config section for a single vehicle.

        This is intended for controller-side nodes (LLC/HLC) to discover their
        runtime configuration without re-parsing the scenario YAML.
        """
        from copy import deepcopy

        if vehicle_name is None:
            vehicle_name = self._sim.default_vehicle_name

        vehicles_cfg = (self._sim.config or {}).get("vehicles", {})
        return {"vehicle_name": vehicle_name, "config": deepcopy(vehicles_cfg.get(vehicle_name, {}))}

    def get_manager_config(self) -> Dict[str, Any]:
        """Return the full config for this sim manager."""
        from copy import deepcopy
        
        return deepcopy(self._sim.config)

    def is_sim_ready(
        self,
    ) -> Dict[str, Any]:
        """Best-effort readiness check for controller-side synchronization.
        """
        has_beamng = getattr(self._sim, "beamng", None) is not None
        vehicles_present = getattr(self._sim, "vehicles", None) is not None and len(self._sim.vehicles) > 0
        if not has_beamng or not vehicles_present:
            self._logger.debug("Sim not ready: BeamNG or vehicles missing")
            return {"ready": False}
        
        # Iterate through vehicles and try to poll states
        for v_name, v_manager in self._sim.vehicles.items():
            vehicle = v_manager.vehicle
            if vehicle is None:
                self._logger.debug(f"Sim not ready: Vehicle {v_name} is None")
                return {"ready": False}
            
            try:
                vehicle.poll_sensors("state")
                state_dict = vehicle.sensors["state"]
            except Exception as e:
                self._logger.debug(f"Sim not ready: Failed to poll {v_name} state: {e}")
                return {"ready": False}
            
            if not state_dict or not isinstance(state_dict, dict):
                self._logger.debug(f"Sim not ready: Vehicle {v_name} state sensor missing or empty")
                return {"ready": False}
        
        return {"ready": True}

    def get_vehicle_safety_and_drivetrain_status(self, vehicle_name: Optional[str] = None) -> Dict[str, Any]:
        """Return 2WD/4WD, diff, and safety features status in one request.

        Returns a YAML-serializable dict with best-effort probing. Vehicles without
        a given feature should return `present/is4wdCapable/hasAbs/hasESC` falsey
        fields or an `error` entry for that subsection.
        """
        if vehicle_name is None:
            vehicle_name = self._sim.default_vehicle_name

        vehicle_manager = self._sim.vehicles[vehicle_name]
        vehicle = vehicle_manager.vehicle

        out: Dict[str, Any] = {
            "vehicle_name": vehicle_name,
            "drivetrain": {},
            "differential": {"front": {}, "rear": {}, "has_differential": None},
            "safety": {},
        }

        # 4WD / range box
        try:
            out["drivetrain"]["4wd_mode"] = vehicle_queries.get_4WD_mode(vehicle)
        except Exception as e:
            out["drivetrain"]["4wd_mode"] = {"error": str(e)}

        # Diff lock state (front/rear)
        for which in ("front", "rear"):
            try:
                state = vehicle_queries.get_diff_lock_state(vehicle, diff=which)
                out["differential"][which] = {"present": True, **(state or {})}
            except Exception as e:
                out["differential"][which] = {"present": False, "error": str(e)}

        # Powertrain scan to detect differentials (best-effort)
        try:
            powertrain = vehicle_queries.get_powertrain_properties(vehicle) or {}
            out["drivetrain"]["powertrain_properties"] = powertrain

            def _has_diff(obj: Any) -> bool:
                if isinstance(obj, dict):
                    t = obj.get("type")
                    if isinstance(t, str) and "diff" in t.lower():
                        return True
                    return any(_has_diff(v) for v in obj.values())
                if isinstance(obj, list):
                    return any(_has_diff(v) for v in obj)
                return False

            out["differential"]["has_differential"] = _has_diff(powertrain)
        except Exception as e:
            out["drivetrain"]["powertrain_properties"] = {"error": str(e)}
            out["differential"]["has_differential"] = None

        # Safety features
        try:
            out["safety"]["abs"] = vehicle_queries.get_ABS(vehicle)
        except Exception as e:
            out["safety"]["abs"] = {"error": str(e)}

        try:
            out["safety"]["esc"] = vehicle_queries.get_ESC(vehicle)
        except Exception as e:
            out["safety"]["esc"] = {"error": str(e)}

        return out

    def disable_safety_features(
        self,
        vehicle_name: Optional[str] = None,
        abs: bool = True,
        esc: bool = True,
        stop_controllers: bool = True,
    ) -> Dict[str, Any]:
        """Disable safety features (ABS/ESC + optional StopSafetyFeatures)."""
        if vehicle_name is None:
            vehicle_name = self._sim.default_vehicle_name

        vehicle = self._sim.vehicles[vehicle_name].vehicle
        return vehicle_queries.disable_safety_features(
            vehicle, abs=abs, esc=esc, stop_controllers=stop_controllers
        )

    def get_advanced_level_info(self, level_name: str) -> Dict[str, Any]:
        """Return advanced level info (spawn points, scenarios, metadata).

        This mirrors the pattern used in the notebooks:
        `beamng._send({type='GetAdvancedLevelInfo', levelName=...}).recv('GetAdvancedLevelInfo')`.

        Requires the xlab/xlabCore extension.
        """
        from bng_simulator.core.xlab_api import XlabApi

        return XlabApi.get_advanced_level_info(self._sim.beamng, level_name)

    def get_level_spawn_points(self, level_name: str) -> Dict[str, Any]:
        """Get spawn points for a level.

        Returns a small, serializable payload:
            {"levelName": str, "spawnPoints": list}
        """
        resp = self.get_advanced_level_info(level_name)
        return {"levelName": level_name, "spawnPoints": resp.get("spawnPoints", [])}

    def get_levels_and_scenarios_info(self) -> Dict[str, Any]:
        """Return levels and scenarios as plain Python data.

        BeamNGpy returns Level/Scenario objects from `get_levels_and_scenarios()`.
        Those are not reliably YAML-serializable for ROS service transport.
        This method converts them into dictionaries/lists.
        """
        levels, scenarios = self._sim.beamng.scenario.get_levels_and_scenarios()

        levels_out: Dict[str, Any] = {}
        for level_key, level in levels.items():
            size = None
            try:
                size = list(level.size) if level.size is not None else None
            except Exception:
                size = None

            props = {}
            try:
                props = dict(level.properties) if getattr(level, "properties", None) else {}
            except Exception:
                props = {}

            levels_out[str(level_key)] = {
                "name": getattr(level, "name", str(level_key)),
                "size": size,
                "path": getattr(level, "path", None),
                **props,
            }

        scenarios_out: Dict[str, Any] = {}
        for level_name, scen_list in scenarios.items():
            scen_payload = []
            for scen in scen_list:
                scen_level = getattr(scen, "level", None)
                if hasattr(scen_level, "name"):
                    scen_level = scen_level.name

                scen_payload.append(
                    {
                        "name": getattr(scen, "name", None),
                        "level": scen_level,
                        "path": getattr(scen, "path", None),
                        "human_name": getattr(scen, "human_name", None),
                        "description": getattr(scen, "description", None),
                        "difficulty": getattr(scen, "difficulty", None),
                        "authors": getattr(scen, "authors", None),
                    }
                )
            scenarios_out[str(level_name)] = scen_payload

        return {"levels": levels_out, "scenarios": scenarios_out}

    def get_vehicle_launch_context(
        self,
        vehicle_name: Optional[str] = None,
        include_spawn_points: bool = True,
        world_space: bool = True,
    ) -> Dict[str, Any]:
        """Return vehicle + level context in one request.

        Intended for CLI / automation to capture everything needed to relaunch:
        - vehicle model
        - vehicle part config (partConfigFilename)
        - current vehicle position
        - current level name (best-effort)
        - spawn points for that level (optional)
        """
        if vehicle_name is None:
            vehicle_name = self._sim.default_vehicle_name

        vehicle_manager = self._sim.vehicles[vehicle_name]
        vehicle = vehicle_manager.vehicle

        model = None
        try:
            model = vehicle.options.get("model")
        except Exception:
            model = None

        part_cfg = self.get_vehicle_part_config(vehicle_name)

        # Current vehicle properties (legacy: keep for compatibility)
        veh_props = vehicle_queries.get_vehicle_properties(vehicle, world_space=world_space)
        curr_pos = veh_props.get("currPos")
        curr_pos_list = None
        if isinstance(curr_pos, dict):
            curr_pos_list = [curr_pos.get("x"), curr_pos.get("y"), curr_pos.get("z")]

        # Ground-truth pose from the built-in BeamNG "state" sensor.
        # This is generally more reliable than currPos and includes quaternion.
        state_payload: Dict[str, Any] = {}
        try:
            # vehicle.poll_sensors("state")
            state = vehicle.sensors["state"]
            if isinstance(state, dict) and state:
                pos = state.get("pos")
                rot = state.get("rotation")
                state_payload = {
                    "time": state.get("time"),
                    "pos": list(pos) if isinstance(pos, (list, tuple)) else pos,
                    "quat": list(rot) if isinstance(rot, (list, tuple)) else rot,
                }
        except Exception:
            state_payload = {}

        # Best-effort level discovery
        level_name = getattr(self._sim, "level_name", None)
        if not level_name:
            for key in ("level", "level_name", "levelName"):
                if key in (self._sim.config or {}):
                    level_name = self._sim.config.get(key)
                    break
        if not level_name:
            scenario_cfg = (self._sim.config or {}).get("scenario", {})
            if isinstance(scenario_cfg, dict):
                level_name = scenario_cfg.get("level") or scenario_cfg.get("level_name")

        gamestate = {}
        try:
            gamestate = self._sim.beamng.control.get_gamestate()
        except Exception:
            gamestate = {}

        scenario_name = None
        try:
            scenario_name = self._sim.beamng.scenario.get_name()
        except Exception:
            scenario_name = None

        spawn_points = None
        if include_spawn_points and level_name:
            try:
                advanced = self.get_advanced_level_info(level_name)
                spawn_points = advanced.get("spawnPoints", [])
            except Exception as e:
                self._logger.error(f"Failed to fetch spawn points for '{level_name}': {e}")
                spawn_points = []

        status = {}
        try:
            status = self.get_vehicle_safety_and_drivetrain_status(vehicle_name=vehicle_name)
        except Exception as e:
            status = {"error": str(e)}

        return {
            "vehicle_name": vehicle_name,
            "vehicle": {
                "model": model,
                "part_config": part_cfg,
                "properties": {
                    "world_space": world_space,
                    "currPos": curr_pos,
                    "currPos_list": curr_pos_list,
                },
                "state": state_payload,
            },
            "status": status,
            "scenario": {
                "level": level_name,
                "scenario_name": scenario_name,
                "gamestate": gamestate,
                "spawnPoints": spawn_points,
            },
        }

    def execute_request(self, func_name: str, **func_args):
        """
        Execute a request on the RequestHandler, SimulationManager, or BeamNG instance.
        
        Priority order:
        1. RequestHandler methods (teleport_vehicle, get_vehicle_part_config, etc.)
        2. SimulationManager methods
        3. BeamNG API calls (beamng.*)
        4. Vehicle API calls (vehicle.*)
        5. Vehicle properties proxy (fallback)
        """
        # Check RequestHandler methods first
        if hasattr(self, func_name) and func_name not in [
            "execute_request",
            "vehicle_api_request",
            "proxy_for_vehicle_properties",
        ]:
            method = getattr(self, func_name)
            out = method(**func_args)
            out_mod = {} if out is None else out
            return out_mod if isinstance(out_mod, dict) else {"result": out_mod}
        
        # Direct SimulationManager method
        if hasattr(self._sim, func_name) and func_name not in [
            "execute_request",
            "vehicle_api_request",
            "proxy_for_vehicle_properties",
        ]:
            method = getattr(self._sim, func_name)
            out = method(**func_args)
            out_mod = {} if out is None else out
            return out_mod if isinstance(out_mod, dict) else {"result": out_mod}

        # BeamNG method call
        if func_name.startswith("beamng."):
            path_components = func_name.split(".")
            current_obj = self._sim.beamng
            for component in path_components[1:]:
                current_obj = getattr(current_obj, component)
            out = current_obj(**func_args)
            out_mod = {} if out is None else out
            return out_mod if isinstance(out_mod, dict) else {"result": out_mod}

        # Vehicle API call
        if func_name.startswith("vehicle."):
            func_handle = func_name.split(".")[1]
            return self.vehicle_api_request(func_handle, **func_args)

        # Fallback to vehicle properties proxy
        return self.proxy_for_vehicle_properties(func_name, **func_args)

    def vehicle_api_request(self, func_name, **func_args):
        """
        Handle vehicle API requests.
        """
        vehicle_name = func_args.pop("vehicle_name", self._sim.default_vehicle_name)
        if vehicle_name not in self._sim.vehicles:
            raise ValueError(f"Vehicle {vehicle_name} not found in the scenario")
        vehicle_manager = self._sim.vehicles[vehicle_name]
        vehicle = vehicle_manager.vehicle
        assert hasattr(vehicle, func_name), f"Vehicle {vehicle_name} has no method {func_name}"
        method = getattr(vehicle, func_name)
        try:
            out = method(**func_args)
            out_mod = {} if out is None else out
            return out_mod if isinstance(out_mod, dict) else {"result": out_mod}
        except Exception as e:
            self._logger.error(f"Error executing {func_name} on vehicle {vehicle_name}: {e}")
            return {"error": str(e)}

    def proxy_for_vehicle_properties(self, property_name: str, vehicle_name=None, **kwargs):
        """
        Proxy for vehicle properties query functions.
        """
        if vehicle_name is None:
            vehicle_name = self._sim.default_vehicle_name
        vehicle = self._sim.vehicles[vehicle_name].vehicle
        query_func = getattr(vehicle_queries, property_name, None)

        if query_func is None:
            err_msg = f"Query function not found: {property_name}"
            self._logger.error(err_msg)
            return {"error": err_msg}

        try:
            self._logger.info(f"Attempting {property_name} for vehicle: {vehicle_name}")
            query_output = query_func(vehicle, **kwargs)
        except TypeError:
            self._logger.error(f"Error executing query function: {property_name}")
            err_msg = "Error:\n" + traceback.format_exc()
            self._logger.error(err_msg)
            return {"error": err_msg}

        self._logger.debug(f"Query output: \n{query_output}")
        return query_output if query_output is not None else {}
