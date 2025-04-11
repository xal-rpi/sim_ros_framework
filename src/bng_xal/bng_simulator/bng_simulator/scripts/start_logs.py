#!/usr/bin/env python3
"""
Interactive Logger Starter

This script:
- Ensures that a run folder (e.g., run_001) exists under ~/beamng_log_data.
- Determines the new run folder and proposes the log file (data.pkl).
- Asks the user to confirm the file name.
- Starts the ROS services (StartLogger and StopLogger) with a provided max_queue_size.
- Waits for the user to type "stop" to end logging.
- Prompts the user for metadata (map name, additional info) and saves it as metadata.yaml.
- Deletes the run folder if the user aborts or opts not to keep it.
"""

import os
import sys
import time
import yaml  # Requires PyYAML installed
import argparse
import shutil
import rclpy
from rclpy.node import Node
from bng_msgs.srv import StartLogger, StopLogger
import pickle
from tqdm.auto import tqdm  # added import at module level

from bng_simulator.utils.services_utils import send_request


# ------------------------------------------------------------------
# Utility to create and return the next run folder (e.g., run_001, run_002, ...)
# ------------------------------------------------------------------
def get_next_run_folder(root_dir):
    if not os.path.exists(root_dir):
        os.makedirs(root_dir)
    runs = [d for d in os.listdir(root_dir) if d.startswith("run_")]
    if runs:
        nums = [int(d.split("_")[1]) for d in runs if d.split("_")[1].isdigit()]
        next_num = max(nums) + 1 if nums else 1
    else:
        next_num = 1
    run_folder = os.path.join(root_dir, f"run_{next_num:03d}")
    os.makedirs(run_folder)
    return run_folder


# ------------------------------------------------------------------
# ROS Node client for starting and stopping the logger service
# ------------------------------------------------------------------
class LoggerClient(Node):
    def __init__(self):
        super().__init__("logger_client")
        # Create ROS service clients for StartLogger and StopLogger.
        self.start_client = self.create_client(StartLogger, "start_logger")
        self.stop_client = self.create_client(StopLogger, "stop_logger")
        # Wait until services are available
        while not self.start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for start_logger service...")
        while not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for stop_logger service...")

    # ------------------------------------------------------------------
    # Send a StartLogger request to the ROS service.
    # ------------------------------------------------------------------
    def start_logging(self, file_path, max_queue_size, flush_interval):
        req = StartLogger.Request()
        req.save_location = file_path
        req.max_queue_size = max_queue_size
        req.flush_interval = flush_interval  # mapping flush_interval to request field.
        future = self.start_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # ------------------------------------------------------------------
    # Send a StopLogger request to the ROS service.
    # ------------------------------------------------------------------
    def stop_logging(self):
        req = StopLogger.Request()
        future = self.stop_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


# ------------------------------------------------------------------
# Consolidate all temporary logger data files into a single file.
# ------------------------------------------------------------------
def consolidate_logger_files(run_folder):
    """
    Consolidates all temporary logger data files (data_*.pkl) in run_folder into a single file (data.pkl),
    merging the structured dictionaries, then deletes the temporary files.
    """
    consolidated = {}
    # List temporary files using os.listdir and extract the numeric part.
    temp_files = []
    for fname in os.listdir(run_folder):
        if fname.startswith("data_") and fname.endswith(".pkl"):
            try:
                num = int(fname[len("data_") : -len(".pkl")])
                temp_files.append((num, os.path.join(run_folder, fname)))
            except ValueError:
                continue
    # Sort files by the extracted number.
    temp_files.sort(key=lambda x: x[0])

    # Use tqdm to display progress while processing files.
    for num, file_path in tqdm(
        temp_files, desc="Consolidating data files", unit="file"
    ):
        try:
            with open(file_path, "rb") as f:
                data = pickle.load(f)
            # Merge data: keys are tuples (vehicle, sensor)
            for key, sensor_data in data.items():
                if key not in consolidated:
                    consolidated[key] = {}
                for field, values in sensor_data.items():
                    consolidated[key].setdefault(field, []).extend(values)
        except Exception as e:
            print(f"Error consolidating {file_path}: {e}")
    # Save the consolidated file.
    consolidated_file = os.path.join(run_folder, "data.pkl")
    try:
        with open(consolidated_file, "wb") as f:
            pickle.dump(consolidated, f)
    except Exception as e:
        print("Failed to save consolidated file:", e)
    # Delete temporary files.
    for _, file_path in temp_files:
        try:
            os.remove(file_path)
        except Exception as e:
            print(f"Failed to delete temporary file {file_path}: {e}")
    print(f"Consolidated data saved to {consolidated_file}")


# ------------------------------------------------------------------
# Main entry point of the script.
# ------------------------------------------------------------------
def main(args=None):
    # Parse command-line arguments for max_queue_size and flush_interval.
    parser = argparse.ArgumentParser(
        description="Interactive logger starter for BeamNG simulation."
    )
    parser.add_argument(
        "--max_queue_size",
        type=int,
        default=50,
        help="Maximum size for the logger queue (default: 50)",
    )
    parser.add_argument(
        "--flush_interval",
        type=float,
        default=5.0,
        help="Flush interval for logger process (default: 5.0 seconds)",
    )
    parsed_args, unknown = parser.parse_known_args(args)

    # Determine the run folder and log file path.
    data_root = os.path.expanduser("~/beamng_log_data")
    run_folder = get_next_run_folder(data_root)
    file_path = os.path.join(run_folder, "data")

    # Inform the user of the file that will be used.
    print("\nLogger will use the following file for logging data:")
    print(f"  {file_path}")
    confirm = input("Proceed? (y/n): ").strip().lower()
    if confirm != "y":
        # Delete the created run folder if user aborts.
        if os.path.exists(run_folder):
            shutil.rmtree(run_folder)
        sys.exit("Aborted by user.\n")

    # Initialize ROS and create our LoggerClient.
    rclpy.init(args=args)
    client = LoggerClient()

    # Send a request for getting information about the scenario
    scenario_infos = send_request("get_sim_config", node_ros=client)

    # Start the logger service with the given max_queue_size and flush_interval.
    print("\nStarting logger service...")
    start_resp = client.start_logging(
        file_path,
        max_queue_size=parsed_args.max_queue_size,
        flush_interval=parsed_args.flush_interval,  # Using flush_interval as per srv definition.
    )
    if not start_resp or not start_resp.success:
        # Remove folder if logger service fails to start.
        if os.path.exists(run_folder):
            shutil.rmtree(run_folder)
        sys.exit("Failed to start logger service.\n")
    print("Logging started.")

    # Wait for the user to enter "stop" to end logging.
    print("\nType 'stop' and press Enter when you wish to terminate logging.")
    while True:
        cmd = input(">> ").strip().lower()
        if cmd == "stop":
            break
        else:
            print("Invalid command. Please type 'stop' to end logging.")

    # Request the service to stop logging.
    stop_resp = client.stop_logging()
    if not stop_resp or not stop_resp.success:
        print("Failed to stop logger service.")
    else:
        print("Logging stopped successfully.")

    # Ask the user for metadata details.
    print("\nEnter metadata information for this run:")
    map_name = input("Map Name: ").strip()
    additional_info = input("Additional Info: ").strip()

    metadata = {
        "map_name": map_name,
        "additional_info": additional_info,
        "run_folder": os.path.basename(run_folder),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        **scenario_infos["vehicles_part"],
        "sim": scenario_infos,
    }

    # Save metadata to metadata.yaml in the run folder.
    metadata_path = os.path.join(run_folder, "metadata.yaml")
    with open(metadata_path, "w") as f:
        yaml.dump(metadata, f, sort_keys=False)
    print(f"Metadata saved to {metadata_path}")

    # Consolidate temporary logger files into one data.pkl and remove temporary data files.
    consolidate_logger_files(file_path)

    # Prompt the user if they want to keep the log folder.
    keep = input("Do you want to keep the run folder? (y/n): ").strip().lower()
    if keep == "n":
        shutil.rmtree(run_folder)
        print(f"Run folder {run_folder} has been deleted.")
    else:
        print(f"Run folder {run_folder} retained.")

    # Cleanup ROS node and shutdown.
    client.destroy_node()
    rclpy.shutdown()
    print("\nExiting logger starter.\n")


if __name__ == "__main__":
    main()
