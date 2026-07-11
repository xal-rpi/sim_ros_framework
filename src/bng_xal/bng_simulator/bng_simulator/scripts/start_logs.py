#!/usr/bin/env python3
"""
Interactive Logger Starter

This script:
- Ensures that a run folder (e.g., run_001) exists under ~/beamng_log_data.
- Determines the new run folder and proposes the log file (data.pkl).
- Asks the user to confirm the file name.
- Starts the ROS services (StartLogger and StopLogger) with a provided max_queue_size.
- Waits for the user to type "stop" to end logging.
- Records specified ROS topics as mcap files in the run folder.
- Prompts the user for metadata (map name, additional info) and saves it as metadata.yaml.
- Consolidates all temporary logger files into data.pkl.
- Deletes the run folder if the user aborts or opts not to keep it.
"""

import os
import sys
import time
import argparse
import shutil
import rclpy
import subprocess
import signal
import atexit
import threading
import pathlib
import json

from bng_simulator.utils.log_session import (
    LoggerClient,
    begin_run,
    end_run,
    get_next_run_folder,
)

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

# Default topics to record with ROS bag
DEFAULT_TOPICS = [
    "/vehicle/mpc_solution",
    "/vehicle/state_n_control",
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
            keep = input("\nDo you want to keep the partial data? (y/n): ").strip().lower()
            if keep != 'y':
                print(f"Removing run folder {run_folder}...")
                shutil.rmtree(run_folder)
                print("Run folder removed.")
            else:
                print(f"Partial data kept in {run_folder}")
        except:
            # If we can't get user input (e.g., in non-interactive mode), keep the data
            print(f"Keeping partial data in {run_folder}")

    print("Cleanup completed.")
    sys.exit()

# ------------------------------------------------------------------
# Copy MPC configuration file to the run folder
# ------------------------------------------------------------------
def copy_mpc_config_file(config_file_path, run_folder):
    """
    Copy the MPC configuration file to the run folder.
    
    Args:
        config_file_path: Path to the MPC configuration file
        run_folder: Path to the run folder where the file will be copied
        
    Returns:
        bool: True if the file was copied successfully, False otherwise
        str: Path to the copied file if successful, None otherwise
    """
    if not config_file_path:
        print("No MPC configuration file path provided.")
        return False, None
        
    # Check if the file exists
    if not os.path.isfile(config_file_path):
        print_error(f"MPC configuration file not found: {config_file_path}")
        return False, None
        
    try:
        # Create the destination path
        dest_file = os.path.join(run_folder, "mpc_config.yaml")
        
        # Copy the file
        shutil.copy2(config_file_path, dest_file)
        print_success(f"MPC configuration file copied to: {dest_file}")
        return True, dest_file
    except Exception as e:
        print_error(f"Error copying MPC configuration file: {e}")
        return False, None

# ------------------------------------------------------------------
# Function to periodically check for MPC configuration
# ------------------------------------------------------------------
def check_mpc_config():
    """
    Periodically check for MPC configuration until received or stopped.
    
    This function runs in a separate thread and attempts to call the
    /mpc_high_level/get_config service at regular intervals. Once it
    receives a successful response, it stores the configuration and stops.
    The thread can also be stopped externally by setting the mpc_config_stop_event.
    """
    global mpc_config, mpc_config_received, client, mpc_config_stop_event, run_folder
    
    print_info("Starting to check for MPC configuration...")
    
    # Check every second until we receive the config or are told to stop
    while not mpc_config_received and client is not None:
        # Check if we've been asked to stop
        if mpc_config_stop_event and mpc_config_stop_event.is_set():
            print_warning("MPC configuration checking stopped.")
            return
            
        try:
            # Try to get the MPC configuration
            result = client.get_mpc_config()
            if result and result.success:
                # Store the configuration in the global variable
                mpc_config = {
                    "num_x": result.num_x,
                    "mpc_fields_name": result.mpc_fields_name,
                    "num_fields": result.num_fields,
                    "mpc_horizon": result.mpc_horizon,
                    "mpc_config_file": result.mpc_config_file,
                    "state_n_control_fields_name": result.state_n_control_fields_name,
                }
                
                # Try to copy the MPC configuration file
                if result.mpc_config_file:
                    success, copied_path = copy_mpc_config_file(result.mpc_config_file, run_folder)
                    if success and copied_path:
                        # Update the config file path to the local copy
                        mpc_config["mpc_config_file_local"] = os.path.basename(copied_path)
                
                mpc_config_received = True
                print("\n")
                print_success("MPC configuration received successfully.")
                
                # Pretty print the received configuration immediately
                print(f"\n{Colors.CYAN}{Colors.BOLD}📋 MPC Configuration Details:{Colors.END}")
                print(f"{Colors.CYAN}{'─' * 50}{Colors.END}")
                for key, value in mpc_config.items():
                    if isinstance(value, list):
                        print(f"{Colors.WHITE}{key:25}{Colors.END}: {Colors.YELLOW}{json.dumps(value, indent=2)}{Colors.END}")
                    else:
                        print(f"{Colors.WHITE}{key:25}{Colors.END}: {Colors.YELLOW}{value}{Colors.END}")
                print(f"{Colors.CYAN}{'─' * 50}{Colors.END}\n")
                
                return  # Exit the thread once we have the config
        except Exception as e:
            print_error(f"Error checking MPC configuration: {e}")
        
        # Sleep for a short time before checking again
        # Use a small timeout to allow for responsive stopping
        for _ in range(10):  # 10 * 0.1 = 1.0 second total
            if mpc_config_stop_event and mpc_config_stop_event.is_set():
                print_warning("MPC configuration checking stopped during sleep.")
                return
            time.sleep(0.1)

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
def main(args=None):
    global run_folder, client, bag_process, logging_started, mpc_config, mpc_config_received
    
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
    parsed_args, unknown = parser.parse_known_args(args)

    # Determine the run folder and log file path.
    data_root = os.path.expanduser("~/beamng_log_data")
    run_folder = get_next_run_folder(data_root)
    # Update global variable for cleanup handler
    globals()['run_folder'] = run_folder
    file_path = os.path.join(run_folder, "data")
    
    # Create a timestamp-based subdirectory name for the bag
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    bag_folder = os.path.join(run_folder, f"rosbag_{timestamp}")
    # Don't create the directory - let ros2 bag do it

    # Inform the user of the file that will be used.
    print("\nLogger will use the following file for logging data:")
    print(f"  {file_path}")
    confirm = input("Proceed? (y/n): ").strip().lower()
    if confirm != "y":
        # Delete the created run folder if user aborts.
        if os.path.exists(run_folder):
            shutil.rmtree(run_folder)
        sys.exit("Aborted by user.\n")

    rclpy.init(args=args)
    client = LoggerClient()
    globals()['client'] = client

    print_info("Starting logger service...")
    try:
        logged_run = begin_run(
            client,
            data_root,
            run_folder=run_folder,
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
    time.sleep(1)  # Give the thread a moment to start
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

    # Wait for the user to enter "stop" to end logging.
    print("\nType 'stop' and press Enter when you wish to terminate logging.")
    while True:
        try:
            cmd = input(">> ").strip().lower()
            if cmd == "stop":
                break
            else:
                print("Invalid command. Please type 'stop' to end logging.")
        except EOFError:
            print("\nInput stream closed. Stopping logging.")
            break  # Treat EOF as a stop command
        except KeyboardInterrupt:
            print("\nInterrupted. Stopping logging.")
            break  # Treat interruption as a stop command

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
    map_name = input("Map Name: ").strip()
    additional_info = input("Additional Info: ").strip()

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
    keep = input("Do you want to keep the run folder? (y/n): ").strip().lower()
    if keep == "n":
        shutil.rmtree(run_folder)
        print(f"Run folder {run_folder} has been deleted.")
    else:
        print(f"Run folder {run_folder} retained.")

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
