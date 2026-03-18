#!/usr/bin/env python3

"""
Comprehensive command-line tool for BeamNG simulation control.

This script provides a unified interface for common simulation operations:
- Vehicle control (teleport, steering, throttle, brake, etc.)
- Level and spawn point queries
- Vehicle configuration extraction and generation
- Powertrain and gearbox information
- Vehicle part configuration
- One-shot vehicle + level context for relaunching

Usage Examples:
    # Teleport vehicle to position with yaw angle
    sim_control teleport --pos 0 0 0 --yaw 90
    sim_control teleport --vehicle ego --pos 100 50 25 --yaw 45 --pitch 10
    
    # Control vehicle inputs (steering, throttle, brake)
    sim_control control --steering 0.5 --throttle 0.8
    sim_control control --brake 1.0 --parkingbrake 1.0
    
    # Query levels and spawn points
    sim_control list-levels
    sim_control spawn-points --level small_island
    sim_control spawn-points --level west_coast_usa
    
    # Get vehicle + level context (one-shot: model, partConfig, position, spawnpoints)
    sim_control vehicle-context
    sim_control vehicle-context --vehicle ego --no-spawnpoints
    
    # Generate complete vehicle configuration file
    sim_control vehicle-config
    sim_control vehicle-config --vehicle ego
    sim_control vehicle-config --output ../config/vehicles/
    sim_control vehicle-config --vehicle ego --output my_vehicle.yaml
    
    # Query powertrain and gearbox information
    sim_control powertrain-info
    sim_control gearbox-info --scan-all
    sim_control engine-info
    
    # Get vehicle part configuration
    sim_control part-config --vehicle ego

    # Safety features (query + toggle)
    sim_control abs
    sim_control abs --disable
    sim_control esc
    sim_control esc --enable

    # Drivetrain + safety status (one-shot)
    sim_control vehicle-status
    sim_control vehicle-status --vehicle ego

    # Disable safety features (one-shot)
    sim_control safety-off
    sim_control safety-off --vehicle ego
    sim_control safety-off --no-stop-controllers
    
    # Get complete simulation configuration
    sim_control sim-config
    
    # Execute custom requests (advanced)
    sim_control exec get_sim_config
    sim_control exec teleport_vehicle vehicle_name=ego pos=[0,0,0] yaw_angle=90
    sim_control exec vehicle.get_bbox_corners vehicle_name=ego
"""

import os
import sys
import time
import argparse
from typing import Dict, Any, Optional
import yaml

try:
    import rclpy
    from bng_simulator.utils.services_utils import send_request
    from bng_simulator.utils.io_dict_utils import (
        save_yaml,
        round_dict_values,
        build_tree_from_dict,
        convert_tree_into_proper_dict
    )
except ImportError as e:
    print(f"Error: Required ROS2 packages not found: {e}", file=sys.stderr)
    print("Make sure you've sourced your ROS2 workspace:", file=sys.stderr)
    print("  source /opt/ros/<distro>/setup.bash", file=sys.stderr)
    print("  source ~/ros2_ws/install/setup.bash", file=sys.stderr)
    sys.exit(1)


class SimulationController:
    """
    Controller class for managing BeamNG simulation via ROS services.
    
    Provides high-level methods for common simulation operations that internally
    use the send_request() utility to communicate with the simulation manager node.
    """
    
    def __init__(self, timeout: float = 2.0):
        """
        Initialize the simulation controller.
        
        Args:
            timeout: ROS service call timeout in seconds
        """
        self.timeout = timeout
    
    def teleport_vehicle(
        self, 
        vehicle_name: Optional[str] = None,
        pos: Optional[list] = None,
        yaw_angle: Optional[float] = None,
        pitch_angle: Optional[float] = None,
        roll_angle: Optional[float] = None,
        reset: bool = True
    ) -> Dict[str, Any]:
        """
        Teleport vehicle to a specific position and orientation.
        
        Args:
            vehicle_name: Name of the vehicle (default uses first vehicle)
            pos: Position [x, y, z] in meters
            yaw_angle: Yaw angle in degrees
            pitch_angle: Pitch angle in degrees
            roll_angle: Roll angle in degrees
            reset: Whether to reset vehicle physics state
            
        Returns:
            Response dictionary with success status
        """
        args = {}
        if vehicle_name:
            args["vehicle_name"] = vehicle_name
        if pos:
            args["pos"] = pos
        if yaw_angle is not None:
            args["yaw_angle"] = yaw_angle
        if pitch_angle is not None:
            args["pitch_angle"] = pitch_angle
        if roll_angle is not None:
            args["roll_angle"] = roll_angle
        args["reset"] = reset
        
        return send_request("teleport_vehicle", args, timeout_sec=self.timeout)
    
    def control_vehicle(
        self,
        vehicle_name: Optional[str] = None,
        steering: Optional[float] = None,
        throttle: Optional[float] = None,
        brake: Optional[float] = None,
        parkingbrake: Optional[float] = None,
        clutch: Optional[float] = None,
        gear: Optional[int] = None
    ) -> Dict[str, Any]:
        """
        Send control inputs to vehicle.
        
        Args:
            vehicle_name: Name of the vehicle
            steering: Steering input [-1.0, 1.0] (left/right)
            throttle: Throttle input [0.0, 1.0]
            brake: Brake input [0.0, 1.0]
            parkingbrake: Parking brake [0.0, 1.0]
            clutch: Clutch input [0.0, 1.0]
            gear: Gear number (integer)
            
        Returns:
            Response dictionary
        """
        control_args = {}
        
        if vehicle_name:
            control_args["vehicle_name"] = vehicle_name
        if steering is not None:
            control_args["steering"] = steering
        if throttle is not None:
            control_args["throttle"] = throttle
        if brake is not None:
            control_args["brake"] = brake
        if parkingbrake is not None:
            control_args["parkingbrake"] = parkingbrake
        if clutch is not None:
            control_args["clutch"] = clutch
        if gear is not None:
            control_args["gear"] = gear
        
        return send_request("vehicle.control", control_args, timeout_sec=self.timeout)
    
    def get_level_info(self) -> Dict[str, Any]:
        """
        Get information about available levels in BeamNG.
        
        Returns:
            Dictionary with level information
        """
        # Use server-side serializer to avoid returning BeamNGpy Level/Scenario objects.
        return send_request("get_levels_and_scenarios_info", {}, timeout_sec=self.timeout)
    
    def get_spawn_points(self, level_name: str) -> Dict[str, Any]:
        """
        Get spawn points for a specific level.
        
        Args:
            level_name: Name of the level (e.g., 'small_island', 'west_coast_usa')
            
        Returns:
            Dictionary with spawn point information
        """
        # Notebook-compatible: GetAdvancedLevelInfo (xlab extension) contains spawnPoints.
        return send_request(
            "get_level_spawn_points",
            {"level_name": level_name},
            timeout_sec=self.timeout,
        )

    def get_vehicle_launch_context(
        self,
        vehicle_name: Optional[str] = None,
        include_spawn_points: bool = True,
        world_space: bool = True,
    ) -> Dict[str, Any]:
        """Get vehicle + level + spawn context in one request."""
        args: Dict[str, Any] = {
            "include_spawn_points": include_spawn_points,
            "world_space": world_space,
        }
        if vehicle_name:
            args["vehicle_name"] = vehicle_name
        return send_request("get_vehicle_launch_context", args, timeout_sec=self.timeout)
    
    def get_vehicle_part_config(self, vehicle_name: Optional[str] = None) -> Dict[str, Any]:
        """
        Get vehicle part configuration.
        
        Args:
            vehicle_name: Name of the vehicle
            
        Returns:
            Vehicle part configuration dictionary
        """
        args = {}
        if vehicle_name:
            args["vehicle_name"] = vehicle_name
        return send_request("get_vehicle_part_config", args, timeout_sec=self.timeout)

    def get_abs(self, vehicle_name: Optional[str] = None) -> Dict[str, Any]:
        """Get ABS status/capability for a vehicle."""
        args: Dict[str, Any] = {}
        if vehicle_name:
            args["vehicle_name"] = vehicle_name
        return send_request("get_ABS", args, timeout_sec=self.timeout)

    def set_abs(self, enabled: bool, vehicle_name: Optional[str] = None) -> Dict[str, Any]:
        """Enable/disable ABS for a vehicle."""
        args: Dict[str, Any] = {"enabled": bool(enabled)}
        if vehicle_name:
            args["vehicle_name"] = vehicle_name
        return send_request("set_ABS", args, timeout_sec=self.timeout)

    def get_esc(self, vehicle_name: Optional[str] = None) -> Dict[str, Any]:
        """Get ESC status/capability for a vehicle."""
        args: Dict[str, Any] = {}
        if vehicle_name:
            args["vehicle_name"] = vehicle_name
        return send_request("get_ESC", args, timeout_sec=self.timeout)

    def set_esc(self, enabled: bool, vehicle_name: Optional[str] = None) -> Dict[str, Any]:
        """Enable/disable ESC for a vehicle."""
        args: Dict[str, Any] = {"enabled": bool(enabled)}
        if vehicle_name:
            args["vehicle_name"] = vehicle_name
        return send_request("set_ESC", args, timeout_sec=self.timeout)

    def get_vehicle_safety_and_drivetrain_status(
        self, vehicle_name: Optional[str] = None
    ) -> Dict[str, Any]:
        """Get 2WD/4WD, diff, and ABS/ESC status in one request."""
        args: Dict[str, Any] = {}
        if vehicle_name:
            args["vehicle_name"] = vehicle_name
        return send_request(
            "get_vehicle_safety_and_drivetrain_status",
            args,
            timeout_sec=self.timeout,
        )

    def disable_safety_features(
        self,
        vehicle_name: Optional[str] = None,
        abs: bool = True,
        esc: bool = True,
        stop_controllers: bool = True,
    ) -> Dict[str, Any]:
        """Disable safety features (ABS/ESC + optional StopSafetyFeatures)."""
        args: Dict[str, Any] = {
            "abs": bool(abs),
            "esc": bool(esc),
            "stop_controllers": bool(stop_controllers),
        }
        if vehicle_name:
            args["vehicle_name"] = vehicle_name
        return send_request("disable_safety_features", args, timeout_sec=self.timeout)
    
    def extract_gear_infos(self, vehicle_name: Optional[str] = None) -> Dict[str, Any]:
        """
        Extract comprehensive gear information by cycling through all gears.
        
        This function iterates through all available gears to build a complete
        mapping of gear names, ratios, and modes.
        
        Args:
            vehicle_name: Name of the vehicle
            
        Returns:
            Dictionary mapping gear names to their properties
        """
        in_args = {}
        in_args_set = {}
        if vehicle_name:
            in_args["vehicle_name"] = vehicle_name
            in_args_set["vehicle_name"] = vehicle_name
        
        # Get initial gear information
        gear_infos = send_request("get_gearbox_info", in_args, timeout_sec=self.timeout)
        min_gear = int(gear_infos["minGearIndex"])
        max_gear = int(gear_infos["maxGearIndex"])
        
        print(f"Scanning gears: {min_gear} to {max_gear}")
        
        def parse_gear_infos(gear_infos):
            """Parse gear information into structured format."""
            return (
                gear_infos["gearName"],
                {
                    "index": int(gear_infos["gearIndex"]),
                    "mode_index": gear_infos["gearModeIndex"],
                    "gear_ratio": gear_infos["gearRatio"],
                    "mode": gear_infos["mode"]
                }
            )
        
        # Store gear mappings
        mapping_info = {}
        gear_name, gear_info = parse_gear_infos(gear_infos)
        initial_gear_name = gear_name
        print(f"  {gear_name}: {gear_info}")
        
        # Cycle through all gears
        for i in range(-1, max_gear + 3):
            in_args_set["gear_index"] = i
            send_request("set_gearbox_index", in_args_set, timeout_sec=self.timeout)
            time.sleep(0.5)  # Wait for gear change
            
            gear_infos = send_request("get_gearbox_info", in_args, timeout_sec=self.timeout)
            gear_name, gear_info = parse_gear_infos(gear_infos)
            gear_info["indexCmd"] = i
            
            if gear_name not in mapping_info:
                mapping_info[gear_name] = gear_info
                print(f"  {gear_name}: {gear_info}")
        
        # Restore initial gear
        initial_gear_index = mapping_info[initial_gear_name]["indexCmd"]
        in_args_set["gear_index"] = initial_gear_index
        send_request("set_gearbox_index", in_args_set, timeout_sec=self.timeout)
        
        return mapping_info
    
    def get_powertrain_info(self, vehicle_name: Optional[str] = None) -> Dict[str, Any]:
        """
        Get formatted powertrain structure with hierarchical component information.
        
        Args:
            vehicle_name: Name of the vehicle
            
        Returns:
            Formatted powertrain structure dictionary
        """
        args = {}
        if vehicle_name:
            args["vehicle_name"] = vehicle_name
        
        powertrain_prop = send_request("get_powertrain_properties", 
                                       args, timeout_sec=self.timeout)
        
        # Format into tree structure
        roots, tree = build_tree_from_dict(powertrain_prop)
        relevant_keys = ['type', 'gearRatio', 'gearRatios', 
                        'availableModes', 'diffTorqueSplit']
        
        result = {}
        for root in roots:
            result[root] = convert_tree_into_proper_dict(
                root, tree, powertrain_prop, relevant_keys
            )
        
        return round_dict_values(result)
    
    def generate_vehicle_config(
        self, 
        output_path: Optional[str] = None,
        vehicle_name: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate comprehensive vehicle configuration file.
        
        This extracts all relevant vehicle parameters including kinematics,
        engine, gearbox, powertrain, and controller information.
        
        Args:
            output_path: Path to save YAML config (if None, returns dict only)
            vehicle_name: Name of the vehicle
            
        Returns:
            Complete vehicle configuration dictionary
        """
        print("Gathering vehicle configuration...")
        
        # Get simulation config for vehicle part identification
        _vehicle_config = send_request(
            "get_vehicle_part_config", {}, 
            timeout_sec=self.timeout
        )
        
        # Kinematics properties
        print("  - Kinematics properties")
        kin_args = {"world_space": True}
        if vehicle_name:
            kin_args["vehicle_name"] = vehicle_name
        kin_props = send_request("get_vehicle_properties", kin_args, 
                                timeout_sec=self.timeout)
        
        # Engine information
        print("  - Engine information")
        engine_args = {}
        if vehicle_name:
            engine_args["vehicle_name"] = vehicle_name
        engine_infos = send_request("get_engine_infos", engine_args, 
                                    timeout_sec=self.timeout)
        
        # Gearbox information
        print("  - Gearbox information")
        gear_infos = self.extract_gear_infos(vehicle_name)
        
        # Powertrain structure
        print("  - Powertrain structure")
        powertrain_infos = self.get_powertrain_info(vehicle_name)
        
        # Controller information
        print("  - Controller information")
        controller_args = {}
        if vehicle_name:
            controller_args["vehicle_name"] = vehicle_name
        controllers_infos = send_request("get_controller_infos", controller_args,
                                        timeout_sec=self.timeout)
        controllers_infos = dict(sorted(controllers_infos.items()))
        
        # Assemble final configuration
        final_config = {
            **round_dict_values(kin_props),
            "engine": round_dict_values(engine_infos),
            "gearbox": gear_infos,
            "powertrain": powertrain_infos,
            "controllers": controllers_infos,
        }
        
        # Save to file
        # Get vehicle part config info
        vehicle_part = _vehicle_config["partConfigFilename"] #of the form /xx/xxx.pc
        vehicle_model_name = _vehicle_config["model"]
        # extract the pc part
        vehicle_part = vehicle_part.split("/")[-1].replace(".pc", "")
        out_name = f"{vehicle_model_name}_{vehicle_part}.yaml"
        
        if output_path is None or output_path.lower() == "" or len(output_path.strip()) == 0:
            print("\nNo output path provided, returning config as dictionary.")
            return final_config
        
        if output_path is not None:
            if os.path.isdir(output_path):
                output_path = os.path.join(output_path, out_name)
            else:
                output_path = output_path
        else:
            output_path = "../config/vehicles/" + out_name
        
        if output_path:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            save_yaml(final_config, output_path, sort_keys=False)
            print(f"\nConfiguration saved to: {output_path}")
        
        return final_config


def setup_parser() -> argparse.ArgumentParser:
    """
    Setup command-line argument parser with all available commands.
    
    Returns:
        Configured ArgumentParser instance
    """
    parser = argparse.ArgumentParser(
        description="BeamNG Simulation Control CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Teleport vehicle
  %(prog)s teleport --pos 0 0 0 --yaw 90
  
  # Control vehicle (50%% steering right, 80%% throttle)
  %(prog)s control --steering 0.5 --throttle 0.8
  
  # List available levels
  %(prog)s list-levels
  
  # Show spawn points
  %(prog)s spawn-points --level small_island
  
  # Generate vehicle config
  %(prog)s vehicle-config --output ./config/
  
  # Get powertrain info
  %(prog)s powertrain-info
  
  # Execute custom request
  %(prog)s exec get_sim_config

    # Drivetrain + safety status
    %(prog)s vehicle-status

    # Disable safety features
    %(prog)s safety-off
        """
    )
    
    parser.add_argument(
        '--timeout', 
        type=float, 
        default=2.0,
        help='ROS service call timeout in seconds (default: 2.0)'
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Teleport command
    teleport = subparsers.add_parser(
        'teleport', 
        help='Teleport vehicle to position/orientation'
    )
    teleport.add_argument('--vehicle', type=str, help='Vehicle name')
    teleport.add_argument(
        '--pos', 
        nargs=3, 
        type=float, 
        metavar=('X', 'Y', 'Z'),
        help='Position coordinates [x, y, z] in meters'
    )
    teleport.add_argument('--yaw', type=float, help='Yaw angle in degrees')
    teleport.add_argument('--pitch', type=float, help='Pitch angle in degrees')
    teleport.add_argument('--roll', type=float, help='Roll angle in degrees')
    teleport.add_argument(
        '--no-reset', 
        action='store_true',
        help='Do not reset vehicle physics state'
    )
    
    # Control command
    control = subparsers.add_parser('control', help='Send control inputs to vehicle')
    control.add_argument('--vehicle', type=str, help='Vehicle name')
    control.add_argument(
        '--steering', 
        type=float, 
        help='Steering [-1.0 to 1.0] (negative=left, positive=right)'
    )
    control.add_argument(
        '--throttle', 
        type=float,
        help='Throttle [0.0 to 1.0]'
    )
    control.add_argument('--brake', type=float, help='Brake [0.0 to 1.0]')
    control.add_argument(
        '--parkingbrake', 
        type=float,
        help='Parking brake [0.0 to 1.0]'
    )
    control.add_argument('--clutch', type=float, help='Clutch [0.0 to 1.0]')
    control.add_argument('--gear', type=int, help='Gear number')
    
    # List levels command
    subparsers.add_parser('list-levels', help='List all available levels')
    
    # Spawn points command
    spawn_pts = subparsers.add_parser(
        'spawn-points', 
        help='Show spawn points for a level'
    )
    spawn_pts.add_argument(
        '--level', 
        type=str, 
        required=True,
        help='Level name (e.g., small_island, west_coast_usa)'
    )

    # Safety features: ABS
    abs_cmd = subparsers.add_parser('abs', help='Get/Set ABS (anti-lock braking)')
    abs_cmd.add_argument('--vehicle', type=str, help='Vehicle name')
    abs_group = abs_cmd.add_mutually_exclusive_group()
    abs_group.add_argument('--enable', action='store_true', help='Enable ABS')
    abs_group.add_argument('--disable', action='store_true', help='Disable ABS')

    # Safety features: ESC
    esc_cmd = subparsers.add_parser('esc', help='Get/Set ESC (stability control)')
    esc_cmd.add_argument('--vehicle', type=str, help='Vehicle name')
    esc_group = esc_cmd.add_mutually_exclusive_group()
    esc_group.add_argument('--enable', action='store_true', help='Enable ESC')
    esc_group.add_argument('--disable', action='store_true', help='Disable ESC')

    # Combined safety + drivetrain status
    veh_status = subparsers.add_parser(
        'vehicle-status',
        help='Get 2WD/4WD, differential, and ABS/ESC status'
    )
    veh_status.add_argument('--vehicle', type=str, help='Vehicle name')

    # Disable safety features in one shot
    safety_off = subparsers.add_parser(
        'safety-off',
        help='Disable safety features (ABS/ESC + optional controller unload)'
    )
    safety_off.add_argument('--vehicle', type=str, help='Vehicle name')
    safety_off.add_argument('--no-abs', action='store_true', help='Do not touch ABS')
    safety_off.add_argument('--no-esc', action='store_true', help='Do not touch ESC')
    safety_off.add_argument(
        '--no-stop-controllers',
        action='store_true',
        help='Do not call StopSafetyFeatures'
    )

    # Vehicle launch/context command
    veh_ctx = subparsers.add_parser(
        'vehicle-context',
        help='Get current vehicle + level + spawn context (one-shot)'
    )
    veh_ctx.add_argument('--vehicle', type=str, help='Vehicle name')
    veh_ctx.add_argument(
        '--no-spawnpoints',
        action='store_true',
        help='Do not include spawn points for the current level'
    )
    veh_ctx.add_argument(
        '--world-space',
        action='store_true',
        help='Use world-space vehicle properties (default: True)'
    )
    veh_ctx.add_argument(
        '--local-space',
        action='store_true',
        help='Use local-space vehicle properties (overrides --world-space)'
    )
    
    # Vehicle config command
    veh_config = subparsers.add_parser(
        'vehicle-config',
        help='Generate comprehensive vehicle configuration file'
    )
    veh_config.add_argument('--vehicle', type=str, help='Vehicle name')
    veh_config.add_argument(
        '--output',
        default="",
        type=str,
        help='Output file path or directory (default: prints to stdout)'
    )
    
    # Part config command
    part_config = subparsers.add_parser(
        'part-config',
        help='Get vehicle part configuration'
    )
    part_config.add_argument('--vehicle', type=str, help='Vehicle name')
    
    # Powertrain info command
    powertrain = subparsers.add_parser(
        'powertrain-info',
        help='Get powertrain structure and components'
    )
    powertrain.add_argument('--vehicle', type=str, help='Vehicle name')
    
    # Gearbox info command
    gearbox = subparsers.add_parser('gearbox-info', help='Get gearbox information')
    gearbox.add_argument('--vehicle', type=str, help='Vehicle name')
    gearbox.add_argument(
        '--scan-all', 
        action='store_true',
        help='Scan all gears (cycles through gears - takes time)'
    )
    
    # Engine info command
    engine = subparsers.add_parser('engine-info', help='Get engine information')
    engine.add_argument('--vehicle', type=str, help='Vehicle name')
    
    # Sim config command
    subparsers.add_parser('sim-config', help='Get complete simulation configuration')
    
    # Execute custom command
    exec_cmd = subparsers.add_parser(
        'exec',
        help='Execute custom request (advanced users)'
    )
    exec_cmd.add_argument('function_name', help='Function name to execute')
    exec_cmd.add_argument(
        'args',
        nargs='*',
        help='Arguments in key=value format (e.g., vehicle_name=ego pos=[0,0,0])'
    )
    
    return parser


def parse_exec_args(args_list: list) -> Dict[str, Any]:
    """
    Parse command-line arguments for exec command.
    
    Supports formats:
        key=value
        key=[1,2,3]
        key={a:1, b:2}
        
    Args:
        args_list: List of argument strings
        
    Returns:
        Dictionary of parsed arguments
    """
    result = {}
    for arg in args_list:
        if '=' not in arg:
            print(f"Warning: Ignoring invalid argument format: {arg}", file=sys.stderr)
            continue
        
        key, value = arg.split('=', 1)
        key = key.strip()
        value = value.strip()
        
        # Try to parse as YAML
        try:
            result[key] = yaml.safe_load(value)
        except yaml.YAMLError:
            # If YAML parsing fails, use as string
            result[key] = value
    
    return result


def print_yaml(data: Any, indent: int = 0):
    """
    Pretty print data in YAML format.
    
    Args:
        data: Data to print
        indent: Indentation level
    """
    yaml.dump(data, sys.stdout, default_flow_style=False, indent=2, sort_keys=False)


def main():
    """Main entry point for the CLI tool."""
    parser = setup_parser()
    args = parser.parse_args()
    
    if args.command is None:
        parser.print_help()
        return 1
    
    # Create controller
    controller = SimulationController(timeout=args.timeout)
    
    try:
        result = None
        
        # Execute command
        if args.command == 'teleport':
            result = controller.teleport_vehicle(
                vehicle_name=args.vehicle,
                pos=args.pos,
                yaw_angle=args.yaw,
                pitch_angle=args.pitch,
                roll_angle=args.roll,
                reset=not args.no_reset
            )
            
        elif args.command == 'control':
            result = controller.control_vehicle(
                vehicle_name=args.vehicle,
                steering=args.steering,
                throttle=args.throttle,
                brake=args.brake,
                parkingbrake=args.parkingbrake,
                clutch=args.clutch,
                gear=args.gear
            )
            
        elif args.command == 'list-levels':
            result = controller.get_level_info()
            if result:
                print("\n=== Available Levels ===")
                levels = result.get('levels', {})
                for level_name, level_info in levels.items():
                    print(f"\n{level_name}:")
                    title = level_info.get('title') or level_info.get('name') or 'N/A'
                    size = level_info.get('size', 'N/A')
                    path = level_info.get('path', 'N/A')
                    print(f"  Title: {title}")
                    print(f"  Size: {size}")
                    print(f"  Path: {path}")
                result = None  # Don't print raw result
                
        elif args.command == 'spawn-points':
            result = controller.get_spawn_points(args.level)
            if result and 'spawnPoints' in result:
                print(f"\n=== Spawn Points for {args.level} ===")
                for pt in result['spawnPoints']:
                    print("\n" + "="*40)
                    print(f"Name: {pt.get('objectname', 'N/A')}")
                    print(f"Translation ID: {pt.get('translationId', 'N/A')}")
                    pos = pt.get('pos', {})
                    print(f"Position: [{pos.get('x', 0):.2f}, {pos.get('y', 0):.2f}, {pos.get('z', 0):.2f}]")
                    print(f"Rotation: {pt.get('rot', 'N/A')}")
                result = None  # Don't print raw result

        elif args.command == 'abs':
            if getattr(args, 'enable', False):
                controller.set_abs(True, vehicle_name=args.vehicle)
            elif getattr(args, 'disable', False):
                controller.set_abs(False, vehicle_name=args.vehicle)

            result = controller.get_abs(vehicle_name=args.vehicle)

        elif args.command == 'esc':
            if getattr(args, 'enable', False):
                controller.set_esc(True, vehicle_name=args.vehicle)
            elif getattr(args, 'disable', False):
                controller.set_esc(False, vehicle_name=args.vehicle)

            result = controller.get_esc(vehicle_name=args.vehicle)

        elif args.command == 'vehicle-context':
            world_space = True
            if getattr(args, 'local_space', False):
                world_space = False
            elif getattr(args, 'world_space', False):
                world_space = True

            result = controller.get_vehicle_launch_context(
                vehicle_name=args.vehicle,
                include_spawn_points=not args.no_spawnpoints,
                world_space=world_space,
            )

            if result:
                vehicle = (result or {}).get('vehicle', {})
                scenario = (result or {}).get('scenario', {})
                part_cfg = (vehicle.get('part_config') or {})
                props = (vehicle.get('properties') or {})
                state = (vehicle.get('state') or {})

                print("\n=== Vehicle Context ===")
                print(f"Vehicle: {result.get('vehicle_name', 'N/A')}")
                print(f"Model: {vehicle.get('model', 'N/A')}")
                print(f"PartConfig: {part_cfg.get('partConfigFilename', 'N/A')}")
                if isinstance(state, dict) and state:
                    print(f"Position (state.pos): {state.get('pos', 'N/A')}")
                    print(f"Quaternion (state.quat): {state.get('quat', 'N/A')}")
                else:
                    print(f"Position (currPos): {props.get('currPos', 'N/A')}")
                    print(f"Position (list): {props.get('currPos_list', 'N/A')}")

                print("\n=== Scenario Context ===")
                print(f"Level: {scenario.get('level', 'N/A')}")
                print(f"Scenario name: {scenario.get('scenario_name', 'N/A')}")
                gamestate = scenario.get('gamestate', {})
                if isinstance(gamestate, dict) and gamestate:
                    print(f"Game state: {gamestate.get('state', 'N/A')} / {gamestate.get('scenario_state', 'N/A')}")

                spawn_points = scenario.get('spawnPoints')
                if isinstance(spawn_points, list):
                    print(f"Spawn points: {len(spawn_points)}")
                    if spawn_points:
                        print("\n--- Spawn Points (YAML) ---")
                        print_yaml(spawn_points)
                elif spawn_points is None:
                    print("Spawn points: (not requested)")

                status = (result or {}).get('status', {})
                if isinstance(status, dict) and status:
                    drivetrain = status.get('drivetrain', {}) or {}
                    diff = status.get('differential', {}) or {}
                    safety = status.get('safety', {}) or {}

                    print("\n=== Drivetrain / Safety ===")
                    mode_payload = drivetrain.get('4wd_mode')
                    if isinstance(mode_payload, dict) and mode_payload:
                        if 'mode' in mode_payload:
                            print(f"Drive mode: {mode_payload.get('mode', 'N/A')}")
                        if 'range' in mode_payload:
                            print(f"Range: {mode_payload.get('range', 'N/A')}")
                        if 'is4wdCapable' in mode_payload:
                            print(f"4WD capable: {mode_payload.get('is4wdCapable', 'N/A')}")

                    if isinstance(diff, dict) and diff:
                        if 'has_differential' in diff:
                            print(f"Has differential: {diff.get('has_differential', 'N/A')}")
                        for which in ('front', 'rear'):
                            payload = diff.get(which, {})
                            if isinstance(payload, dict) and payload:
                                print(f"Diff {which}: present={payload.get('present', 'N/A')} locked={payload.get('locked', 'N/A')}")

                    if isinstance(safety, dict) and safety:
                        abs_payload = safety.get('abs')
                        if isinstance(abs_payload, dict) and abs_payload:
                            if 'hasAbs' in abs_payload:
                                print(f"ABS capable: {abs_payload.get('hasAbs', 'N/A')}")
                            if 'enabled' in abs_payload:
                                print(f"ABS enabled: {abs_payload.get('enabled', 'N/A')}")
                        esc_payload = safety.get('esc')
                        if isinstance(esc_payload, dict) and esc_payload:
                            if 'hasESC' in esc_payload:
                                print(f"ESC capable: {esc_payload.get('hasESC', 'N/A')}")
                            if 'enabled' in esc_payload:
                                print(f"ESC enabled: {esc_payload.get('enabled', 'N/A')}")

                result = None  # Already formatted

        elif args.command == 'vehicle-status':
            result = controller.get_vehicle_safety_and_drivetrain_status(vehicle_name=args.vehicle)

        elif args.command == 'safety-off':
            result = controller.disable_safety_features(
                vehicle_name=args.vehicle,
                abs=not args.no_abs,
                esc=not args.no_esc,
                stop_controllers=not args.no_stop_controllers,
            )
                
        elif args.command == 'vehicle-config':
            result = controller.generate_vehicle_config(
                output_path=args.output,
                vehicle_name=args.vehicle
            )
            print_yaml(result)
            result = None  # Already handled
            
        elif args.command == 'part-config':
            result = controller.get_vehicle_part_config(args.vehicle)
            
        elif args.command == 'powertrain-info':
            result = controller.get_powertrain_info(args.vehicle)
            
        elif args.command == 'gearbox-info':
            if args.scan_all:
                result = controller.extract_gear_infos(args.vehicle)
            else:
                req_args = {}
                if args.vehicle:
                    req_args["vehicle_name"] = args.vehicle
                result = send_request("get_gearbox_info", req_args, 
                                    timeout_sec=args.timeout)
                
        elif args.command == 'engine-info':
            req_args = {}
            if args.vehicle:
                req_args["vehicle_name"] = args.vehicle
            result = send_request("get_engine_infos", req_args, 
                                timeout_sec=args.timeout)
            
        elif args.command == 'sim-config':
            result = send_request("get_sim_config", {}, timeout_sec=args.timeout)
            
        elif args.command == 'exec':
            exec_args = parse_exec_args(args.args)
            result = send_request(args.function_name, exec_args, 
                                timeout_sec=args.timeout)
        
        # Print result if not already handled
        if result is not None:
            print("\n=== Result ===")
            print_yaml(result)
        
        return 0
        
    except KeyboardInterrupt:
        print("\nInterrupted by user", file=sys.stderr)
        return 130
        
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1
        
    finally:
        pass


if __name__ == '__main__':
    sys.exit(main())
