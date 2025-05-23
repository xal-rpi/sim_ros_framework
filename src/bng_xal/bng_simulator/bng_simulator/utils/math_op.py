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
