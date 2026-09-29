"""
Implements the custom BeamNG Roads sensor for ROS 2.
This version dynamically handles cached message mismatches to prevent iteration errors.
"""

from typing import Optional
import numpy as np

# Import the native beamngpy sensor
from beamngpy.sensors import RoadsSensor
from beamngpy.beamng import BeamNGpy
from beamngpy.vehicle import Vehicle

# Import custom base classes and registries
from bng_simulator.vehicle.sensors import SensorBase, SensorRegistry
from bng_simulator.utils.services_utils import convert_time_to_header

# Import the ROS message type
from bng_msgs.msg import RoadsMsg


@SensorRegistry.register("BngRoadsSensor")
class BngRoadsSensor(SensorBase):
    """
    A wrapper to integrate the native BeamNGpy RoadsSensor into the ROS 2 pipeline.
    """

    def __init__(self, name: str, vehicle: Vehicle, beamng: BeamNGpy, config: dict):
        super().__init__(name, vehicle, beamng, config)

        physics_update_time = config.get("physics_update_time", 0.01)
        gfx_update_time = config.get("gfx_update_time", 0.05)

        self._sensor = RoadsSensor(
            name,
            beamng,
            vehicle,
            physics_update_time=physics_update_time,
            gfx_update_time=gfx_update_time
        )

    def poll(self):
        """
        Poll the sensor for the latest data and isolate a single segment.
        """
        readings = self._sensor.poll()

        if not readings:
            self._last_data = None
            self._all_data = []
            return

        single_segment = None

        if isinstance(readings, list) and len(readings) > 0:
            single_segment = readings[0]
        elif isinstance(readings, dict):
            if 'headingAngle' in readings or 'dist2CL' in readings:
                single_segment = readings
            else:
                for key, val in readings.items():
                    if isinstance(val, dict):
                        single_segment = val
                        break

        if not isinstance(single_segment, dict):
            self._last_data = None
            self._all_data = []
            return

        # Ensure we store it as a list to satisfy the SimulationManager
        self._all_data = [single_segment]
        self._last_data = single_segment

    def get_all_data(self):
        """
        Explicitly override to guarantee an iterable list is returned to SimulationManager.
        """
        return self._all_data if isinstance(self._all_data, list) else []

    def ros_msg_type(self):
        return RoadsMsg

    def _safe_assign(self, msg, field_name, value):
        """
        Dynamically handles scalar vs array message compilation mismatches.
        """
        try:
            # Try to assign it as a scalar (flat message)
            setattr(msg, field_name, value)
        except TypeError:
            # If the ROS 2 setter throws a TypeError, it expects an array. Wrap the value.
            setattr(msg, field_name, [value])

    def to_ros_msg(self, data: Optional[dict] = None, frame_id="map"):
        """
        Convert the basic sensor state to a ROS message safely.
        """
        if data is None:
            data = self._last_data

        if data is None:
            return None

        msg = RoadsMsg()

        sim_time = data.get('time', 0.0)
        msg.header = convert_time_to_header(sim_time, frame_id)

        # Use the safe assigner to map all fields, dodging the cache trap
        self._safe_assign(msg, 'heading_angle', float(data.get('headingAngle', 0.0)))
        self._safe_assign(msg, 'half_width', float(data.get('halfWidth', 0.0)))
        
        radius = float(data.get('roadRadius', 0.0))
        self._safe_assign(msg, 'radius', radius)
        
        # Calculate curvature safely
        curvature = 1.0 / radius if radius and radius != 0.0 and not np.isnan(radius) else 0.0
        self._safe_assign(msg, 'curvature', curvature)

        self._safe_assign(msg, 'lane_count', int(data.get('numlane', 0)))
        self._safe_assign(msg, 'dist_centerline', float(data.get('dist2CL', 0.0)))
        self._safe_assign(msg, 'dist_left_edge', float(data.get('dist2Left', 0.0)))
        self._safe_assign(msg, 'dist_right_edge', float(data.get('dist2Right', 0.0)))

        return msg