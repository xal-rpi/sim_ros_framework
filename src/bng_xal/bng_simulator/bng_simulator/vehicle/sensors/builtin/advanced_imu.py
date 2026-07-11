"""
Stock AdvancedIMU via beamngpy (techCore.lua) — thin bringup adapter only.

Creation/poll for GE logger path uses beamngpy as-is. Live sensor_send on vlua
reads extensions.tech_advancedIMU.getLatest(sensorId); this wrapper only
exposes sensorId to sensor_broadcast resolution.
"""

from typing import Any, Dict, Optional

from beamngpy import BeamNGpy
from beamngpy.sensors import AdvancedIMU as BeamngAdvancedIMU
from beamngpy.vehicle import Vehicle

from bng_simulator.vehicle.sensors import SensorBase, SensorRegistry


def _map_yaml_to_beamngpy(config: Dict[str, Any]) -> Dict[str, Any]:
    """Map scenario YAML keys to beamngpy AdvancedIMU keyword names."""
    key_map = {
        "gfx_update_time": "gfx_update_time",
        "physics_update_time": "physics_update_time",
        "smoother_strength": "smoother_strength",
        "is_send_immediately": "is_send_immediately",
        "is_using_gravity": "is_using_gravity",
        "is_allow_wheel_nodes": "is_allow_wheel_nodes",
        "is_visualised": "is_visualised",
        "is_snapping_desired": "is_snapping_desired",
        "is_force_inside_triangle": "is_force_inside_triangle",
        "is_dir_world_space": "is_dir_world_space",
    }
    out: Dict[str, Any] = {}
    for yaml_key, bng_key in key_map.items():
        if yaml_key in config:
            out[bng_key] = config[yaml_key]
    for key in ("pos", "dir", "up"):
        if key in config:
            out[key] = tuple(config[key])
    return out


@SensorRegistry.register("AdvancedIMU")
class AdvancedIMU(SensorBase):
    """ROS/logger adapter around beamngpy.sensors.AdvancedIMU."""

    def __init__(self, name: str, vehicle: Vehicle, beamng: BeamNGpy, config: dict):
        super().__init__(name, vehicle, beamng, config)
        self._sensor = BeamngAdvancedIMU(
            name, beamng, vehicle, **_map_yaml_to_beamngpy(config)
        )

    @property
    def beamng_sensor_id(self) -> int:
        return int(self._sensor.sensorId)

    def poll(self):
        readings = self._sensor.poll()
        if not readings:
            self._last_data = None
            self._all_data = []
            return
        if isinstance(readings, dict):
            self._all_data = [readings]
        else:
            self._all_data = list(readings)
        self._last_data = self._all_data[-1] if self._all_data else None

    def ros_msg_type(self):
        return None

    def to_ros_msg(self, data: Optional[dict] = None, frame_id: str = "map"):
        return None

    def remove(self):
        self._sensor.remove()
