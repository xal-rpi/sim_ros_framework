"""
This module contains some functions needed to load log data
and provide some utility functions to work with the data.
"""

import os
import pickle
import yaml  # Requires PyYAML installed
from bng_simulator.utils.io_dict_utils import (
    load_yaml,
    save_yaml,
)


def load_metadata(run_number, root_dir="~/beamng_log_data"):
    """
    Load metadata from the specified run number.

    Args:
        run_number (int): The run number (e.g., 1 for run_001).
        root_dir (str): The root directory where run folders are stored (default: ~/beamng_log_data).

    Returns:
        dict: The metadata dictionary loaded from metadata.yaml.

    Raises:
        FileNotFoundError: If the metadata file does not exist.
    """
    run_folder = os.path.join(os.path.expanduser(root_dir), f"run_{run_number:03d}")
    metadata_path = os.path.join(run_folder, "metadata.yaml")
    if not os.path.exists(metadata_path):
        raise FileNotFoundError(f"Metadata file not found: {metadata_path}")
    metadata = load_yaml(metadata_path)
    return metadata


def load_consolidated_data(run_number, root_dir="~/beamng_log_data"):
    """
    Load consolidated log data from the specified run number.

    Args:
        run_number (int): The run number (e.g., 1 for run_001).
        root_dir (str): The root directory where run folders are stored (default: ~/beamng_log_data).

    Returns:
        dict: The consolidated log data loaded from data.pkl.

    Raises:
        FileNotFoundError: If the consolidated data file does not exist.
    """
    run_folder = os.path.join(os.path.expanduser(root_dir), f"run_{run_number:03d}")
    data_file = os.path.join(run_folder, "data", "data.pkl")
    if not os.path.exists(data_file):
        raise FileNotFoundError(f"Consolidated data file not found: {data_file}")
    with open(data_file, "rb") as f:
        data = pickle.load(f)
    return data


def load_log_data(run_number, root_dir="~/beamng_log_data"):
    """
    Load log data and metadata from the specified run number.

    Args:
        run_number (int): The run number (e.g., 1 for run_001).
        root_dir (str): The root directory where run folders are stored (default: ~/beamng_log_data).

    Returns:
        dict: The log data loaded from the run folder.

    Raises:
        FileNotFoundError: If the run folder or log data file does not exist.
    """
    metadata = load_metadata(run_number, root_dir)
    data = load_consolidated_data(run_number, root_dir)
    return metadata, data
