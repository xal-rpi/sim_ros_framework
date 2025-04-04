"""
Simplified base class for the vehicle sensors.
"""

from typing import Dict, Type
from abc import ABC, abstractmethod
from copy import deepcopy

from beamngpy import Vehicle, BeamNGpy


class SensorBase(ABC):
    """
    Base class for the vehicle sensors.
    """

    def __init__(self, name: str, vehicle: Vehicle, beamng: BeamNGpy, config: dict):
        """
        Initialize the sensor with the vehicle and configuration.

        Args:
            vehicle (Vehicle): The vehicle to attach the sensor to.
            beamng (BeamNGpy): The BeamNGpy instance.
            config (dict): The sensor configuration.
        """
        self._name = name
        self._vehicle = vehicle
        self._beamng = beamng
        self._config = deepcopy(config)
        self._last_data = None
        self._all_data = []

    @abstractmethod
    def poll(self):
        """
        Retrieve raw sensor date from BeamNG.
        """
        pass

    @abstractmethod
    def to_ros_msg(self, data: dict = None, frame_id="map"):
        """
        Convert the sensor data to a ROS message.

        Returns:
            Any: The ROS message or None if no data available.
        """
        return None

    def get_last_data(self):
        """
        Get the last sensor data.

        Returns:
            Any: The last sensor data.
        """
        return self._last_data

    def get_all_data(self):
        """
        Get all the sensor data.

        Returns:
            List[Any]: All the sensor data.
        """
        return self._all_data

    @property
    def name(self):
        """
        Get the sensor name.

        Returns:
            str: The sensor name.
        """
        return self._name

    @abstractmethod
    def ros_msg_type(self):
        """
        Get the ROS message type for the sensor.

        Returns:
            str: The ROS message type.
        """
        raise NotImplementedError(
            "ros_msg_type() must be implemented in the derived class."
        )


class SensorRegistry:
    _sensor_classes: Dict[str, Type["SensorBase"]] = {}

    @classmethod
    def register(cls, sensor_type: str) -> callable:
        def decorator(subclass: Type["SensorBase"]) -> Type["SensorBase"]:
            if sensor_type in cls._sensor_classes:
                raise ValueError(f"Sensor type {sensor_type} already registered")
            cls._sensor_classes[sensor_type] = subclass
            return subclass

        return decorator

    @classmethod
    def get_class(cls, sensor_type: str) -> Type["SensorBase"]:
        try:
            return cls._sensor_classes[sensor_type]
        except KeyError:
            raise ValueError(f"Unregistered sensor type: {sensor_type}")
