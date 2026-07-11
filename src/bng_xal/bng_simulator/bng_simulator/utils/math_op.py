"""
Utility functions for mathematical operations. Convertion between different units and coordinate systems.
ROS2 to BeamNG coordinate system conversion etc...
"""

from typing import Tuple
import numpy as np


def convert_euler_to_quaternion(
    euler_angles: Tuple[float, float, float],
) -> Tuple[float, float, float, float]:
    """
    Convert Euler angles to quaternion.

    Args:
        euler_angles (Tuple[float, float, float]): The Euler angles. roll, pitch, yaw.

    Returns:
        Tuple[float, float, float, float]: The quaternion. x, y, z, w.
    """
    roll, pitch, yaw = euler_angles
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return [x, y, z, w]


def process_euler_to_quat(args_dict: dict, deg_to_rad_factor: float = np.pi / 180) -> None:
    """
    Process Euler angles in args_dict and convert to quaternion in place.
    
    Modifies args_dict: sets 'rot_quat' and removes Euler angle keys.
    
    Args:
        args_dict: Dictionary that may contain yaw_angle, pitch_angle, roll_angle
        deg_to_rad_factor: Conversion factor from degrees to radians
    """
    if "yaw_angle" in args_dict or "pitch_angle" in args_dict or "roll_angle" in args_dict:
        yaw_rad = args_dict.get("yaw_angle", 0) * deg_to_rad_factor
        pitch_rad = args_dict.get("pitch_angle", 0) * deg_to_rad_factor
        roll_rad = args_dict.get("roll_angle", 0) * deg_to_rad_factor
        rot_quat = convert_euler_to_quaternion((roll_rad, pitch_rad, yaw_rad))
        args_dict["rot_quat"] = tuple([float(q) for q in rot_quat])
        args_dict.pop("yaw_angle", None)
        args_dict.pop("pitch_angle", None)
        args_dict.pop("roll_angle", None)
        args_dict.pop("xlab_yaw_deg", None)


def apply_xlab_yaw_to_beamng(
    args_dict: dict,
    yaw_offset_deg: float,
) -> None:
    """
    Convert xlab yaw to BeamNG spawn euler (in place).

    beamng_yaw = xlab_yaw - yaw_offset_deg
    (utv: gtState yaw at rest ≈ beamng_spawn_yaw + yaw_offset_deg)
    """
    if "rot_quat" in args_dict:
        return
    xlab_yaw = args_dict.pop("xlab_yaw_deg", None)
    if xlab_yaw is None and "yaw_angle" in args_dict:
        xlab_yaw = args_dict["yaw_angle"]
    if xlab_yaw is None:
        return
    args_dict["xlab_yaw_deg"] = xlab_yaw
    args_dict["yaw_angle"] = float(xlab_yaw) - float(yaw_offset_deg)
