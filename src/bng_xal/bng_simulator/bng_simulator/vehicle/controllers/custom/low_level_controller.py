"""
Low-level controller implementation.
"""

from rclpy import logging

from beamngpy.connection import CommBase
from beamngpy.beamng import BeamNGpy
from beamngpy.vehicle import Vehicle

from bng_simulator.vehicle.controllers.base import ControllerBase, ControllerRegistry


class LowLevelControllerWrapper(CommBase):
    """
    Wrapper for the low-level controller communication with the BeamNG
    simulator. Accepts arbitrary config parameters and converts them to the
    expected message format.
    """

    def __init__(self, name: str, vehicle: Vehicle, beamng: BeamNGpy, **config):
        super().__init__(beamng, vehicle)
        self.logger = logging.get_logger("LowLevelController")
        self.name = name
        self.vehicle = vehicle

        self._open_controller(name, vehicle, **config)

    def remove(self) -> None:
        """Remove this controller from the simulation."""
        self._close_controller()
        self.logger.info(f"LowLevelController - controller removed: {self.name}")

    def _open_controller(self, name: str, vehicle: Vehicle, **config) -> None:
        data = {
            "name": name,
            "vid": vehicle.vid,
        }

        for key, value in config.items():
            if value is None:
                continue
            data[key] = value
            if key == "gt_state_name":
                self.logger.info(f"Using gtState sensor: {value} for controller")

        self.send_ack_ge(
            type="OpenController",
            ack="OpenedController",
            **data,
        )
        self.logger.info(f"Opened LowLevelController: {name}\n{data}")

    def _close_controller(self) -> None:
        self.send_ack_ge(
            type="CloseController",
            ack="ClosedController",
            name=self.name,
            vid=self.vehicle.vid,
        )
        self.logger.info(f'Closed LowLevelController: "{self.name}"')


@ControllerRegistry.register("LowLevelController")
class LowLevelController(ControllerBase):
    """
    Low-level controller for BeamNG vehicles.
    """

    def __init__(
        self,
        name: str,
        vehicle: Vehicle,
        beamng: BeamNGpy,
        config: dict,
    ):
        super().__init__(name, vehicle, beamng, config)
        self._controller = LowLevelControllerWrapper(name, vehicle, beamng, **config)

    def start(self):
        """Start the controller."""
        self._is_running = True

    def stop(self):
        """Stop the controller."""
        if self._is_running:
            self._controller.remove()
            self._is_running = False
