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
        listen_ip: str = "127.0.0.1",
        listen_port: int = 64257,
        send_ip: str = "127.0.0.1",
        send_port: int = 64258,
        gt_state_name: str = "gtstate",
        ctrl_type: str = "default",
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
            ctrl_type,
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
        gt_state_name: str,
        ctrl_type: str,
    ) -> None:
        data = dict()
        data["name"] = name
        data["vid"] = vehicle.vid
        data["listenIp"] = listen_ip
        data["listenPort"] = listen_port
        data["sendIp"] = send_ip
        data["sendPort"] = send_port
        data["controllerType"] = ctrl_type

        # Add gtStateName if provided
        if gt_state_name:
            data["gtStateName"] = gt_state_name
            self.logger.info(f"Using gtState sensor: {gt_state_name} for controller")

        self.send_ack_ge(type="OpenController", ack="OpenedController", **data)
        self.logger.info(f"Opened LowLevelController: {name} \n{data}")

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

    def __init__(self, name: str, vehicle: Vehicle, beamng: BeamNGpy, config: dict):
        super().__init__(name, vehicle, beamng, config)

        # Create the controller instance
        self._controller = LowLevelControllerWrapper(
            name,
            vehicle,
            beamng,
            config.get("listen_ip", "127.0.0.1"),
            config.get("listen_port", 64257),
            config.get("send_ip", "127.0.0.1"),
            config.get("send_port", 64258),
            config.get("gt_state_name", None),
            config.get("type", "default"),
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
