"""
Implement utility functions to format beamng data into out standard
package format for training neural based dynamics models.
"""

import os
from typing import Tuple, Union, Dict, Any, List, Callable
import traceback

import numpy as np

from tqdm.auto import tqdm

from bng_simulator.utils.logger_utils import load_log_data


def find_consecutive_true(metrics, min_length=-1):
    """ Given a boolean array, find the consecutive sections 
        of True values that are at least min_length long.
    
    Args:
        metrics: The boolean array
            (n,) array
        min_length: The minimum length of the consecutive 
        True sections
            int
    
    Returns:
        full_auto_inds: The indices of the consecutive 
        True sections
            list of arrays
    """
    full_auto = metrics
    section_inds_ = np.split(
        np.r_[:len(full_auto)], 
        np.where(np.diff(full_auto) != 0)[0]+1
    )
    full_auto_inds = []
    for inds_ in section_inds_:
        if full_auto[inds_[0]] and len(inds_) > min_length:
            full_auto_inds.append(inds_)
    return full_auto_inds

def format_gtstate_traj(
    metadata: Dict[str, Any],
    data: Dict[str, np.ndarray],
    valid_trajectory_cond: callable = None,
    min_traj_len: int = 200,
    indx_traj: int = 0
):
    """ Format the ground truth states and controls in a format 
        compatible with data_utils.
    
    Args:
        data: dictionary containing the ground truth states and controls
            dict
        valid_trajectory_cond: condition to filter out invalid 
            trajectories or transitions
            callable
    """
    if valid_trajectory_cond is not None:
        valid = valid_trajectory_cond(data)
    else:
        valid = np.ones(data["time"].shape, dtype=bool)
    valid_indexes = find_consecutive_true(valid, min_length=min_traj_len)
    # If no valid section is found, skip the run
    success = True
    if len(valid_indexes) == 0:
        success = False
        tqdm.write(
            f'No subset of length < {min_traj_len} is invertible'
        )
        tqdm.write('-----------------\n')
        return (None, None), success
    # Extract the valid transitions
    trajectories = []
    trajectories_info = []
    for seq in valid_indexes:
        # Extract the valid transitions
        curr_data = {}
        max_min_data = {}
        for key, value in data.items():
            curr_data[key] = value[seq]
            max_min_data[key] = (np.max(curr_data[key]), np.min(curr_data[key]))
        trajectories.append(curr_data)
        curr_traj_info = {
            **metadata,
            'duration (s)' : curr_data['time'][-1] - curr_data['time'][0],
            'traj_idx' : indx_traj + len(trajectories) - 1,
            'max_min_data' : max_min_data,
            "system_physical_params": {}, # TODO: Completed later
        }
        trajectories_info.append(curr_traj_info)
    return (trajectories, trajectories_info), success

def exponential_moving_average(data, alpha=0.2):
    """
    Compute the exponential moving average of a 1D array.

    Args:
        data (np.ndarray): Input data array.
        alpha (float): Smoothing factor between 0 and 1.
            Higher alpha means less smoothing.
    
    Returns:
        np.ndarray: Filtered data array.
    """
    ema = np.zeros_like(data)
    ema[0] = data[0]
    for i in range(1, len(data)):
        ema[i] = alpha * data[i] + (1 - alpha) * ema[i - 1]
    return ema

def preprocess_data(data: Dict[str, np.ndarray], extra_processing: callable = None):
    """
    Perform the necessary processing of the data.
    Typically renaming some convenient states and controls.
    Converting to the right units.
    etc...
    """
    # Average rear wheel speed into a single variable
    w_rl = data["wheelRR_angVel"] # In radian per second
    w_rr = data["wheelRL_angVel"] # In radian per second
    data["rear_wheelspeed"] = 0.5 * (w_rl + w_rr)
    # Convert RPM into radian per second
    data["engine_speed"] = data["RPM"] * (2 * np.pi / 60)
    # Rename boost pressure
    data["boost_pressure"] = data["turboBoost"]
    data["throttle_cmd"] = data["throttle"]
    data["throttle"] = data.get("throttleValve", data["throttle"])
    if extra_processing is not None:
        extra_processing(data)

def format_dataset(
    runs: List[int],
    sensor_type: str = "gtstate",
    vehicle_model: str = "utv_wild",
    valid_trajectory_cond: callable = None,
    extra_processing: callable = None,
    min_traj_len: int = 200
):
    """
    Format the dataset into a format compatible with data_utils.
    
    Args:
        runs: list of run numbers
            list of int
        sensor_type: type of sensor
            str
        vehicle_model: type of vehicle model
            str
        valid_trajectory_cond: condition to filter out invalid 
            trajectories or transitions
            callable
        min_traj_len: minimum length of a trajectory when 
            enforcing valid_trajectory_cond
            int
        extra_processing: extra processing to be applied to the data
            callable - function (data: Dict[str, np.ndarray]) - modified in place
    
    """
    # The main variables
    m_trajectories = []
    m_trajectories_info = []
    # Loading logging
    failed_logs = []
    successful_logs = []
    for run in runs:
        run_name = f"Run {run}"
        tqdm.write(f'\n========== Loading {run_name} ==========\n')
        # Load the run first
        try:
            metadata, data = load_log_data(run)
            metadata.pop('sim', None)
        except Exception:
            traceback_str = traceback.format_exc()
            tqdm.write(traceback_str)
            failed_logs.append(run_name)
            tqdm.write('Error loading...')
            tqdm.write('-----------------\n')
            continue
        # Load was successful - Let's iterate through all vehicle sensor to pick relevant one
        data_of_interest = {}
        for (vehicle_name, veh_sensor), curr_data in data.items():
            veh_model_type = metadata[vehicle_name]
            if veh_sensor != sensor_type or veh_model_type != vehicle_model:
                tqdm.write(f"Ignoring {vehicle_name} - {veh_sensor} of type {veh_model_type} from {run_name}")
                continue
            # Otherwise, this is a valid data
            data_of_interest[(vehicle_name, veh_sensor)] = curr_data 
        if len(data_of_interest) < 1:
            tqdm.write(f"No useful data from {run_name}. Skipping.....")
            failed_logs.append(run_name)
            continue
        # Otherwise, we have useful data.
        num_success = 0
        for info, curr_data in data_of_interest.items():
            tqdm.write(f"Processing {info}....")
            curr_data =  {k : np.array(v) for k, v in curr_data.items()}
            # Let's process the data
            preprocess_data(curr_data, extra_processing)
            # Now let's try to split it.
            (_trajectories, _trajectories_info), _success = format_gtstate_traj(
                metadata, curr_data, valid_trajectory_cond = valid_trajectory_cond,
                min_traj_len=min_traj_len, indx_traj=len(m_trajectories)
            )
            if not _success:
                continue
            num_success += 1
            # Trajectories were loaded successfully, so let's append them
            m_trajectories.extend(_trajectories)
            m_trajectories_info.extend(_trajectories_info)
        if num_success >= 1:
            successful_logs.append(run_name)
        else:
            failed_logs.append(run_name)
    # Let's print out a summary of successful and failed loading
    print(f"Total number of successful logs: {len(successful_logs)}")
    print(f"Total number of failed logs: {len(failed_logs)}")
    print(f"Total number of trajectories: {len(m_trajectories)}")

    # Print failed loading
    if len(failed_logs) > 0:
        print('\nFailed loading : ')
        for log_info in failed_logs:
            print('- ', log_info)

    if len(m_trajectories) == 0:
        print("No valid trajectory found. Exiting...")
        return

    if len(successful_logs) > 0:
        print('\nSuccessful loading : ')
        for log_info in successful_logs:
            print('- ', log_info)

    # Dataset
    dataset ={
        'trajectories' : m_trajectories,
        'trajectories_info' : m_trajectories_info,
        'system_physical_params' : {}, # To be updated
        'data_fields': list(m_trajectories[0].keys()),
    }
    return dataset
