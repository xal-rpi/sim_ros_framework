#!/usr/bin/env python3
"""
Interactive Logger Starter

This script:
- Ensures that a run folder exists under ~/beamng_log_data/<vehicle>/run_XXX.
- Determines the new run folder and proposes the log file (data.pkl).
- Asks the user to confirm the file name.
- Starts the ROS services (StartLogger and StopLogger) with a provided max_queue_size.
- Waits for the user to type "stop" to end logging.
- Records specified ROS topics as mcap files in the run folder.
- Prompts the user for metadata (map name, additional info) and saves it as metadata.yaml.
- Consolidates all temporary logger files into data.pkl.
- Discards recording outputs on request; managed runs retain session/ diagnostics.
"""

import os
import sys
import time
import argparse
import rclpy
import subprocess
import signal
import atexit
import threading
import json
from pathlib import Path
import yaml

from bng_simulator.utils.log_session import (
    LoggerClient,
    begin_run,
    end_run,
    get_next_run_folder,
    discard_run_folder,
    vehicle_slug_from_config,
)
from bng_simulator.utils.services_utils import send_request
from bng_simulator.utils.mpc_snapshot import DEFAULT_INFO_SERVICE, snapshot_ref_traj_info

_managed_run = False

# Color utilities for terminal output
class Colors:
    """ANSI color codes for terminal output"""
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'  # End formatting

def print_success(msg):
    """Print a success message in green"""
    print(f"{Colors.GREEN}✓ {msg}{Colors.END}")

def print_error(msg):
    """Print an error message in red"""
    print(f"{Colors.RED}✗ {msg}{Colors.END}")

def print_warning(msg):
    """Print a warning message in yellow"""
    print(f"{Colors.YELLOW}⚠ {msg}{Colors.END}")

def print_info(msg):
    """Print an info message in blue"""
    print(f"{Colors.BLUE}ℹ {msg}{Colors.END}")

# Global state variables to track what needs cleanup
run_folder = None
client = None
bag_process = None
logging_started = False
cleanup_done = False  # Flag to prevent cleanup from running multiple times

# MPC configuration variables
mpc_config = None
mpc_config_received = False
mpc_config_thread = None
mpc_config_stop_event = None  # Event to signal the MPC config thread to stop
mpc_info_service = DEFAULT_INFO_SERVICE

# Default topics to record with ROS bag
DEFAULT_TOPICS = [
    "/drive_high_level_ros/controller_info",
    "/drive_high_level_ros/mpc_horizon",
    "/drive_high_level_ros/vehicle_on_surface",
    "/drive_high_level_ros/debug/gt_path",
    "/drive_high_level_ros/mpc_solve",
]

# ------------------------------------------------------------------
# Cleanup handler for graceful shutdown
# ------------------------------------------------------------------
def cleanup_handler(signum=None, frame=None):
    """
    Handle cleanup when the script is interrupted.
    
    This function ensures all resources are properly released, including:
    - Stopping the ROS bag recording process
    - Stopping the logger service
    - Stopping the MPC configuration thread
    - Cleaning up the ROS node
    - Handling the run folder based on user input
    
    Args:
        signum: Signal number that triggered this handler (if any)
        frame: Current stack frame (not used)
    """
    global client, bag_process, run_folder, logging_started, cleanup_done
    global mpc_config_thread, mpc_config_stop_event
    
    # Prevent running twice
    if cleanup_done:
        return
    cleanup_done = True
    
    if signum is not None:
        print(f"\nReceived signal {signum}. Performing cleanup...")
    
    # Stop ROS bag recording if it was started
    if bag_process:
        try:
            print_info("Stopping ROS bag recording...")
            os.killpg(os.getpgid(bag_process.pid), signal.SIGINT)
            bag_process.wait(timeout=5)
            print_success("ROS bag recording stopped")
        except Exception as e:
            print_error(f"Error stopping ROS bag recording: {e}")
            try:
                os.killpg(os.getpgid(bag_process.pid), signal.SIGKILL)
            except:
                pass
    
    # Stop logger service if it was started
    if client and logging_started:
        try:
            print_info("Stopping logger service...")
            stop_resp = client.stop_logging()
            if stop_resp and stop_resp.success:
                print_success("Logger service stopped successfully.")
            else:
                print_error("Failed to stop logger service.")
        except Exception as e:
            print_error(f"Error stopping logger service: {e}")
    
    # Stop MPC configuration thread if it's running
    if mpc_config_thread and mpc_config_thread.is_alive():
        try:
            print_info("Stopping MPC configuration thread...")
            if mpc_config_stop_event:
                mpc_config_stop_event.set()
            # Wait for the thread to terminate with a timeout
            mpc_config_thread.join(timeout=2.0)
            if mpc_config_thread.is_alive():
                print_warning("Warning: MPC configuration thread did not terminate cleanly.")
            else:
                print_success("MPC configuration thread stopped.")
        except Exception as e:
            print_error(f"Error stopping MPC configuration thread: {e}")
    
    # Clean up ROS node
    if client:
        try:
            client.destroy_node()
        except Exception as e:
            print(f"Error destroying ROS node: {e}")
    
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as e:
        print(f"Error shutting down ROS: {e}")
    
    # Ask if the user wants to keep the partial data
    if run_folder and os.path.exists(run_folder):
        try:
            keep = prompt("\nDo you want to keep the partial data? (y/n): ", "y").strip().lower()
            if keep != 'y':
                print(f"Removing run folder {run_folder}...")
                discard_run_folder(run_folder, preserve_session=_managed_run)
                print("Recording discarded; session diagnostics retained." if _managed_run else "Run folder removed.")
            else:
                print(f"Partial data kept in {run_folder}")
        except:
            # If we can't get user input (e.g., in non-interactive mode), keep the data
            print(f"Keeping partial data in {run_folder}")

    print("Cleanup completed.")
    sys.exit()

# ------------------------------------------------------------------
# Capture the current controller configuration without blocking recording.
# ------------------------------------------------------------------
def check_mpc_config():
    """Retry GetRefTrajInfo until captured or logging stops.

    Use a dedicated node/executor so the background service call cannot race
    the main thread's logger start/stop calls on LoggerClient. Controllers that
    start after recording are supported; no MPC is required for a sensor-only run.
    """
    global mpc_config, mpc_config_received
    try:
        from xal_drive_msgs.srv import GetRefTrajInfo
    except ImportError:
        print_warning("MPC capture skipped: xal_drive_msgs is unavailable; source its workspace.")
        return

    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor

    node = Node("logger_mpc_snapshot")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    service = node.create_client(GetRefTrajInfo, mpc_info_service)
    print_info(f"Waiting for MPC identity on {mpc_info_service} (recording continues).")
    try:
        while not mpc_config_stop_event.is_set() and rclpy.ok():
            try:
                if service.wait_for_service(timeout_sec=0.2):
                    future = service.call_async(GetRefTrajInfo.Request())
                    executor.spin_until_future_complete(future, timeout_sec=1.0)
                    if future.done() and not future.cancelled():
                        result = future.result()
                        if result is not None and result.success:
                            captured = snapshot_ref_traj_info(result, run_folder, print_warning)
                            captured["service"] = mpc_info_service
                            mpc_config = captured
                            mpc_config_received = True
                            report = print_warning if captured.get("reference_snapshot_error") else print_success
                            report("MPC identity captured" + (
                                "; reference snapshot failed (see error above)." if captured.get("reference_snapshot_error") else
                                " with reference/ and tracking.yaml saved." if captured.get("tracking_yaml") else " with reference/ saved."
                            ))
                            print(json.dumps(captured, indent=2))
                            return
                    else:
                        service.remove_pending_request(future)
                        future.cancel()
            except Exception as exc:
                print_warning(f"MPC configuration capture failed: {exc}")
            # Interruptible delay keeps shutdown responsive while allowing late startup.
            mpc_config_stop_event.wait(1.0)
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()

# ------------------------------------------------------------------
# Setup signal handlers
# ------------------------------------------------------------------
def setup_signal_handlers():
    """Register signal handlers for graceful termination."""
    signal.signal(signal.SIGINT, cleanup_handler)
    signal.signal(signal.SIGTERM, cleanup_handler)
    # Register the cleanup function to be called on normal exit
    atexit.register(cleanup_handler)

# ------------------------------------------------------------------
# Main entry point of the script.
# ------------------------------------------------------------------
_session_status = None
_assume_yes = False


def session_status(state):
    """Optional supervisor notification; regular interactive logging is unchanged."""
    if _session_status:
        path = Path(_session_status)
        temporary = path.with_suffix('.tmp')
        temporary.write_text(yaml.safe_dump({'state': state}))
        temporary.replace(path)


def prompt(text, default=''):
    if _assume_yes:
        print(text + default)
        return default
    return input(text)


def wait_for_stop(batch):
    """Batch waits for a signal, independent of stdin; interactive mode accepts stop."""
    print('Batch recording: Ctrl-C stops and saves.' if batch else "Type 'stop' to finish recording.")
    while True:
        try:
            if batch:
                time.sleep(.2)
            elif input('>> ').strip().lower() == 'stop':
                return
            else:
                print("Invalid command. Please type 'stop'.")
        except (EOFError, KeyboardInterrupt):
            return


def main(args=None):
    global run_folder, client, bag_process, logging_started, mpc_config, mpc_config_received
    global mpc_info_service
    
    # Setup signal handlers for graceful termination
    setup_signal_handlers()
    # Parse command-line arguments for max_queue_size and flush_interval.
    parser = argparse.ArgumentParser(
        description="Interactive logger starter for BeamNG simulation."
    )
    parser.add_argument(
        "--max_queue_size",
        type=int,
        default=5000,
        help="Maximum size for the logger queue (default: 50)",
    )
    parser.add_argument(
        "--flush_interval",
        type=float,
        default=5.0,
        help="Flush interval for logger process (default: 5.0 seconds)",
    )
    parser.add_argument(
        "--record_topics",
        type=str,
        default="all",
        help="Topics to record with ROS bag: 'default' for predefined topics, 'all' for all topics, "
             "or a comma-separated list of topics (default: 'all')",
    )
    parser.add_argument(
        "--mpc_info_service", default=DEFAULT_INFO_SERVICE,
        help="GetRefTrajInfo service for the controller (supports renamed/namespaced nodes)",
    )
    parser.add_argument(
        "--no_rosbag",
        action="store_true",
        help="Disable ROS bag recording",
    )
    parser.add_argument(
        "--bag_format",
        type=str,
        default="mcap",
        choices=["mcap", "sqlite3"],
        help="Format for ROS bag recording (default: mcap)",
    )
    parser.add_argument('--sim-config', help='Resolved manager configuration captured by drive-run')
    parser.add_argument('--run-folder', help='Explicit recording directory supplied by drive-run')
    parser.add_argument('--managed-run', action='store_true', help='Preserve session/ diagnostics when recording is discarded; requires --run-folder')
    parser.add_argument('--status-file', help='Optional recording-state notification file')
    parser.add_argument('--require-reference', action='store_true', help='Do not announce recording ready without a verified controller reference')
    parser.add_argument('--yes', action='store_true', help='Batch logging: confirm/save automatically; SIGINT stops cleanly')
    parsed_args, unknown = parser.parse_known_args(args)
    global _session_status, _assume_yes
    _session_status, _assume_yes = parsed_args.status_file, parsed_args.yes
    global _managed_run
    _managed_run = parsed_args.managed_run
    if _managed_run and (not parsed_args.run_folder or not (Path(parsed_args.run_folder) / "session").is_dir()):
        raise ValueError('--managed-run requires --run-folder with session/ diagnostics')
    mpc_info_service = parsed_args.mpc_info_service

    data_root = os.path.expanduser("~/beamng_log_data")

    rclpy.init(args=args)
    client = LoggerClient()
    globals()['client'] = client

    if parsed_args.sim_config:
        scenario_infos = yaml.safe_load(Path(parsed_args.sim_config).read_text())
        if not isinstance(scenario_infos, dict):
            raise ValueError('Simulator snapshot must be a mapping')
        scenario_infos['vehicles_part'] = {
            name: str(cfg.get('model_args',{}).get('model',''))+'_'+Path(cfg.get('model_args',{}).get('part_config','unknown')).stem
            for name,cfg in scenario_infos.get('vehicles',{}).items()}
    else:
        scenario_infos = send_request("get_sim_config", node_ros=client) or {}
    vehicle_slug = vehicle_slug_from_config(scenario_infos)
    run_folder = parsed_args.run_folder or get_next_run_folder(data_root, vehicle=vehicle_slug)
    os.makedirs(run_folder, exist_ok=True)
    globals()['run_folder'] = run_folder
    file_path = os.path.join(run_folder, "data")

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    bag_folder = os.path.join(run_folder, f"rosbag_{timestamp}")

    print("\nLogger will use the following file for logging data:")
    print(f"  vehicle: {vehicle_slug}")
    print(f"  {file_path}")
    confirm = prompt("Proceed? (y/n): ", "y").strip().lower()
    if confirm != "y":
        session_status("declined")
        globals()["run_folder"] = None
        sys.exit("Aborted by user.\n")

    print_info("Starting logger service...")
    try:
        logged_run = begin_run(
            client,
            data_root,
            run_folder=run_folder,
            scenario_infos=scenario_infos,
            max_queue_size=parsed_args.max_queue_size,
            flush_interval=parsed_args.flush_interval,
        )
    except RuntimeError:
        print_error("Failed to start logger service.")
        sys.exit(1)
    print_success("Logging started.")
    globals()['logging_started'] = True
    
    # Initialize and start MPC configuration checking thread
    global mpc_config_thread, mpc_config_stop_event
    mpc_config_stop_event = threading.Event()
    mpc_config_thread = threading.Thread(target=check_mpc_config, daemon=True)
    mpc_config_thread.start()
    print_info("Started MPC configuration checking thread.")
    
    # Start ROS bag recording if enabled
    bag_process = None
    recorded_topics = []
    
    if not parsed_args.no_rosbag:
        # Determine which topics to record
        if parsed_args.record_topics.lower() == "default":
            topics = DEFAULT_TOPICS
            print_info(f"Recording default topics: {', '.join(topics)}")
        elif parsed_args.record_topics.lower() == "all":
            topics = ["-a"]  # Special flag for ros2 bag to record all topics
            print_info("Recording all topics")
        else:
            topics = parsed_args.record_topics.split(",")
            print_info(f"Recording specified topics: {', '.join(topics)}")
        
        recorded_topics = topics
        
        # Build the ros2 bag command
        bag_cmd = ["ros2", "bag", "record", "-o", bag_folder]
        
        # Add storage format (mcap or sqlite3)
        bag_cmd.extend(["-s", parsed_args.bag_format])
        
        # Add topics
        bag_cmd.extend(topics)
        
        # Start the recording process
        try:
            bag_process = subprocess.Popen(
                bag_cmd, preexec_fn=os.setsid,
                stdin=subprocess.DEVNULL,  # Prevent interference with input()
                stdout=None,               # Let stdout flow to terminal
                stderr=None                # Let stderr flow to terminal
            )
            # Update global variable for cleanup handler
            globals()['bag_process'] = bag_process
            print_success(f"Started ROS bag recording in {parsed_args.bag_format} format")
        except Exception as e:
            print_error(f"Failed to start ROS bag recording: {e}")
            bag_process = None
        # Pause briefly to ensure the process starts
        time.sleep(2)

    if bag_process is not None and bag_process.poll() is not None:
        raise RuntimeError('ROS recorder exited before session readiness')
    if parsed_args.require_reference:
        deadline = time.monotonic() + 60.0
        while not mpc_config_received:
            if time.monotonic() > deadline:
                raise RuntimeError('Controller reference snapshot was not captured')
            time.sleep(.1)
        if mpc_config.get('reference_snapshot_error') or mpc_config.get('tracking_snapshot_error'):
            raise RuntimeError('Recording setup is incomplete: '+str(mpc_config))
    session_status('recording')
    def stop_recording(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop_recording)
    signal.signal(signal.SIGTERM, stop_recording)

    wait_for_stop(parsed_args.yes)

    # Stop ROS bag recording if it was started
    if bag_process:
        try:
            print_info("Stopping ROS bag recording...")
            os.killpg(os.getpgid(bag_process.pid), signal.SIGINT)
            bag_process.wait(timeout=15)
            print_success("ROS bag recording stopped")
        except Exception as e:
            print_error(f"Error stopping ROS bag recording: {e}")
            # Force kill if needed
            try:
                os.killpg(os.getpgid(bag_process.pid), signal.SIGKILL)
            except:
                pass

    # Ask the user for metadata details.
    print("\nEnter metadata information for this run:")
    map_name = prompt("Map Name: ").strip()
    additional_info = prompt("Additional Info: ").strip()

    # Stop the MPC config thread if it's still running
    if mpc_config_thread and mpc_config_thread.is_alive():
        if mpc_config_stop_event:
            mpc_config_stop_event.set()
        mpc_config_thread.join(timeout=2.0)
        if mpc_config_thread.is_alive():
            print("Warning: MPC configuration thread did not terminate cleanly.")
    
    if mpc_config_received and mpc_config:
        print_success("Added MPC configuration to metadata.")
    else:
        print_warning("MPC configuration was not received.")

    if not parsed_args.no_rosbag:
        rosbag_meta = {
            "enabled": True,
            "format": parsed_args.bag_format,
            "topics": recorded_topics if recorded_topics != ["-a"] else ["all"],
            "path": os.path.basename(bag_folder),
        }
    else:
        rosbag_meta = {"enabled": False}

    try:
        data_pkl = end_run(
            client,
            logged_run,
            {"map_name": map_name, "additional_info": additional_info},
            mpc_config=mpc_config if mpc_config_received else None,
            rosbag=rosbag_meta,
        )
    except RuntimeError:
        print_error("Failed to stop logger service.")
        sys.exit(1)
    print_success("Logging stopped successfully.")
    print_success(f"Metadata saved to {os.path.join(run_folder, 'metadata.yaml')}")
    print_success(f"Consolidated data saved to {data_pkl}")

    # Prompt the user if they want to keep the log folder.
    keep = prompt("Do you want to keep the run folder? (y/n): ", "y").strip().lower()
    if keep == "n":
        discard_run_folder(run_folder, preserve_session=_managed_run)
        print(f"Recording discarded; diagnostics retained in {run_folder}/session" if _managed_run else f"Run folder {run_folder} has been deleted.")
    else:
        print(f"Run folder {run_folder} retained.")

    session_status("saved" if keep != "n" else "discarded")

    # Cleanup is handled by atexit handler, but we need to reset the globals
    # to prevent the atexit handler from asking about keeping data
    globals()['run_folder'] = None
    
    # Make sure the MPC config thread is stopped
    if mpc_config_stop_event:
        mpc_config_stop_event.set()
    
    global cleanup_done
    cleanup_done = True
    print("\nExiting logger starter.\n")

if __name__ == "__main__":
    main()
