"""
Low-level controller implementation.
"""

from rclpy import logging

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID
from beamngpy.beamng import BeamNGpy
from beamngpy.vehicle import Vehicle

from bng_simulator.vehicle.controllers.base import ControllerBase, ControllerRegistry


class LowLevelControllerWrapper(CommBase):
    """
    Wrapper for the low-level controller communication with the BeamNG simulator.
    """

    def __init__(
        self,
        name: str,
        vehicle: Vehicle,
        beamng: BeamNGpy,
        listen_ip: str = "127.0.1.1",
        listen_port: int = 64257,
        send_ip: str = "127.0.1.1",
        send_port: int = 64258,
        gt_state_name: str = None,
    ):
        super().__init__(beamng, vehicle)

        self.logger = logging.get_logger(f"{LOGGER_ID}.LowLevelController")
        self.name = name
        self.vehicle = vehicle

        # Open the controller in the simulation
        self._open_controller(
            name,
            vehicle,
            listen_ip,
            listen_port,
            send_ip,
            send_port,
            gt_state_name,
        )

    def remove(self) -> None:
        """Remove this controller from the simulation."""
        self._close_controller()
        self.logger.info(f"LowLevelController - controller removed: {self.name}")

    def _open_controller(
        self,
        name: str,
        vehicle: Vehicle,
        listen_ip: str,
        listen_port: int,
        send_ip: str,
        send_port: int,
        gt_state_name: str = None,
    ) -> None:
        data = dict()
        data["name"] = name
        data["vid"] = vehicle.vid
        data["listen_ip"] = listen_ip
        data["listen_port"] = listen_port
        data["send_ip"] = send_ip
        data["send_port"] = send_port

        # Add gtStateName if provided
        if gt_state_name:
            data["gtStateName"] = gt_state_name
            self.logger.info(f"Using gtState sensor: {gt_state_name} for controller")

        self.send_ack_ge(
            type="XlabCommand", xtype="OpenController", ack="OpenedController", **data
        )
        self.logger.info(f"Opened LowLevelController: {name} \n{data}")

    def _close_controller(self) -> None:
        self.send_ack_ge(
            type="XlabCommand",
            xtype="CloseController",
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

    def __init__(self, name: str, vehicle: Vehicle, beamng: BeamNGpy, config: dict):
        super().__init__(name, vehicle, beamng, config)

        # Get the gt_state_name from config
        gt_state_name = config.get("gt_state_name", None)

        # Create the controller instance
        self._controller = LowLevelControllerWrapper(
            name,
            vehicle,
            beamng,
            config.get("listen_ip", "127.0.1.1"),
            config.get("listen_port", 64257),
            config.get("send_ip", "127.0.1.1"),
            config.get("send_port", 64258),
            gt_state_name,
        )

    def start(self):
        """Start the controller."""
        # Controller is automatically started when opened
        self._is_running = True

    def stop(self):
        """Stop the controller."""
        if self._is_running:
            self._controller.remove()
            self._is_running = False
