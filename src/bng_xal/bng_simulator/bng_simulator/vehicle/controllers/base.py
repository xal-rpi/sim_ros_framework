"""
Base class for vehicle controllers.
"""

from typing import Dict, Type
from abc import ABC, abstractmethod
from copy import deepcopy

from beamngpy import Vehicle, BeamNGpy
from beamngpy.logging import BNGValueError


class ControllerBase(ABC):
    """
    Base class for vehicle controllers.
    """

    def __init__(self, name: str, vehicle: Vehicle, beamng: BeamNGpy, config: dict):
        """Initialize the controller with the vehicle and configuration."""
        self._name = name
        self._vehicle = vehicle
        self._beamng = beamng
        self._config = deepcopy(config)
        self._is_running = False

    @abstractmethod
    def start(self):
        """Start the controller."""
        pass

    @abstractmethod
    def stop(self):
        """Stop the controller."""
        pass

    @property
    def name(self):
        """Get the controller name."""
        return self._name

    @property
    def is_running(self):
        """Check if the controller is running."""
        return self._is_running


class ControllerRegistry:
    """Registry for controller classes."""

    _controller_classes: Dict[str, Type["ControllerBase"]] = {}

    @classmethod
    def register(cls, controller_type: str) -> callable:
        def decorator(subclass: Type["ControllerBase"]) -> Type["ControllerBase"]:
            if controller_type in cls._controller_classes:
                raise ValueError(
                    f"Controller type {controller_type} already registered"
                )
            cls._controller_classes[controller_type] = subclass
            return subclass

        return decorator

    @classmethod
    def get_class(cls, controller_type: str) -> Type["ControllerBase"]:
        try:
            return cls._controller_classes[controller_type]
        except KeyError:
            raise ValueError(f"Unregistered controller type: {controller_type}")
