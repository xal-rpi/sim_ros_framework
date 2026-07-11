# sensors/__init__.py
#
# Sensor bringup adapters for bng_simulator YAML:
#   CUSTOM (xlab mod):  GtState  → xlabCore.lua / extensions.xlab_gtState
#   STOCK (beamngpy):   AdvancedIMU, GPS, … → techCore.lua / extensions.tech_*
#
# Do not reimplement stock sensor Open*/Poll* in xlabCore. Use beamngpy classes
# for attach; pass resolved sensorId into LowLevelController sensor_broadcast.
from .base import SensorBase
from .base import SensorRegistry
from .builtin.basic_state import BasicState
from .builtin.advanced_imu import AdvancedIMU
from .custom.GtState import GtState
