"""
Implements the custom ground truth state sensor.
"""

from logging import getLogger
from sys import stderr
from typing import Optional

import numpy as np

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID
from beamngpy.types import StrDict

from beamngpy.beamng import BeamNGpy
from beamngpy.vehicle import Vehicle

from bng_simulator.vehicle.sensors import SensorBase, SensorRegistry

from bng_msgs.msg import GtStateMsg
from bng_simulator.utils.services_utils import convert_time_to_header


class GtStateWrapper(CommBase):
    """xlab GtState: COM + RPY path. Lua owns the report point."""

    def __init__(
        self,
        name: str,
        vehicle: Vehicle,
        beamng: BeamNGpy,
        gfx_update_time: float = 0.05,
        physics_update_time: float = 0.01,
        num_physics_steps_for_gfx_save: int = 1,
        is_visualised: bool = True,
        accel_tau_s: Optional[float] = None,
        gyro_tau_s: Optional[float] = None,
        vel_tau_s: Optional[float] = None,
        wheel_angvel_tau_s: Optional[float] = None,
        debug_raw: Optional[bool] = None,
        torque_map: Optional[dict] = None,
        **_ignored,
    ):
        super().__init__(beamng, vehicle)
        self.logger = getLogger(f"{LOGGER_ID}.GtState")
        self.name = name
        self.vehicle = vehicle
        self._open_gt_state(
            name,
            vehicle,
            gfx_update_time,
            physics_update_time,
            num_physics_steps_for_gfx_save,
            is_visualised,
            accel_tau_s,
            gyro_tau_s,
            vel_tau_s,
            wheel_angvel_tau_s,
            debug_raw,
            torque_map,
        )
        self.sensorId = self._get_gt_state_id()

    def remove(self) -> None:
        self._close_gt_state()
        self.logger.info("GtState - sensor removed: " f"{self.name}")

    def poll(self) -> StrDict:
        return self._poll_gt_state_ge()

    def _get_gt_state_id(self) -> int:
        return int(self.send_recv_ge(type="GetGtStateId", name=self.name)["data"])

    def _open_gt_state(
        self,
        name: str,
        vehicle: Vehicle,
        gfx_update_time: float,
        physics_update_time: float,
        num_physics_steps_for_gfx_save: int,
        is_visualised: bool,
        accel_tau_s: Optional[float] = None,
        gyro_tau_s: Optional[float] = None,
        vel_tau_s: Optional[float] = None,
        wheel_angvel_tau_s: Optional[float] = None,
        debug_raw: Optional[bool] = None,
        torque_map: Optional[dict] = None,
    ) -> None:
        data: StrDict = dict()
        data["name"] = name
        data["vid"] = vehicle.vid
        data["GFXUpdateTime"] = gfx_update_time
        data["physicsUpdateTime"] = physics_update_time
        data["numPhysicsStepsForGFXSave"] = num_physics_steps_for_gfx_save
        data["isVisualised"] = is_visualised
        if accel_tau_s is not None:
            data["accel_tau_s"] = accel_tau_s
        if gyro_tau_s is not None:
            data["gyro_tau_s"] = gyro_tau_s
        if vel_tau_s is not None:
            data["vel_tau_s"] = vel_tau_s
        if wheel_angvel_tau_s is not None:
            data["wheel_angvel_tau_s"] = wheel_angvel_tau_s
        if debug_raw is not None:
            data["debug_raw"] = debug_raw
        if torque_map is not None:
            data["torque_map"] = torque_map
        args = {
            "type": "OpenGtState",
            "ack": "OpenedGtState",
            **data,
        }
        print(args, flush=True, file=stderr)
        self.send_ack_ge(**args)
        self.logger.info(f"Opened GtState sensor: {name} \n{data}")

    def _close_gt_state(self) -> None:
        self.send_ack_ge(
            type="CloseGtState",
            ack="ClosedGtState",
            name=self.name,
            vid=self.vehicle.vid,
        )
        self.logger.info(f'Closed GtState sensor: "{self.name}"')

    def _poll_gt_state_ge(self) -> StrDict:
        return self.send_recv_ge(type="PollGtStateGE", name=self.name)["data"]


@SensorRegistry.register("GtState")
class GtState(SensorBase):
    """The custom ground truth state sensor."""

    def __init__(self, name: str, vehicle: Vehicle, beamng: BeamNGpy, config: dict):
        super().__init__(name, vehicle, beamng, config)
        cfg = dict(config)
        for dead in (
            "is_using_gravity",
            "kf_predict_gain",
            "attach_z_offset",
            "attitude_mode",
            "attitude_tau_s",
            "is_force_inside_triangle",
            "is_snapping_desired",
            "is_allow_wheel_nodes",
            "pos",
            "dir",
            "left",
        ):
            cfg.pop(dead, None)
        self._sensor = GtStateWrapper(name, vehicle, beamng, **cfg)
        self.__DEG_TO_RAD = np.pi / 180.0

    def poll(self):
        all_readings = self._sensor.poll()
        if len(all_readings) == 0:
            self._last_data = None
            self._all_data = []
            return
        if type(all_readings) == dict:
            assert 0.0 not in all_readings, "0.0 in all_readings"
            self._all_data = [all_readings]
        else:
            assert type(all_readings) == list, "all_readings is not a list"
            self._all_data = all_readings
        self.process_data()
        self._last_data = self._all_data[-1]

    def process_data(self):
        for data in self._all_data:
            data["steering"] = data["steering"] * self.__DEG_TO_RAD

    def ros_msg_type(self):
        return GtStateMsg

    def to_ros_msg(self, data: Optional[dict] = None, frame_id="map"):
        if data is None:
            data = self._last_data
        if self._last_data is None:
            return None

        header = convert_time_to_header(data["time"], frame_id)
        msg = GtStateMsg()
        msg.header = header
        msg.time = data["time"]
        msg.dir_x.x, msg.dir_x.y, msg.dir_x.z = data["dirX"]
        msg.dir_y.x, msg.dir_y.y, msg.dir_y.z = data["dirY"]
        msg.vel.x, msg.vel.y, msg.vel.z = data["vel"]
        msg.accel.x, msg.accel.y, msg.accel.z = data["accel"]
        msg.ang_vel.x, msg.ang_vel.y, msg.ang_vel.z = data["angVel"]
        msg.pos.x, msg.pos.y, msg.pos.z = data["pos"]
        msg.quat.x, msg.quat.y, msg.quat.z, msg.quat.w = data["quat"]

        wheelFR = data["wheelFR"]
        wheelFL = data["wheelFL"]
        wheelRR = data["wheelRR"]
        wheelRL = data["wheelRL"]

        msg.wheel_fr_speed = wheelFR["speed"]
        msg.wheel_fr_ang_vel = wheelFR["angVel"]
        msg.wheel_fr_brake_torque = wheelFR["brakeTorque"]
        msg.wheel_fr_prop_torque = wheelFR["propTorque"]
        msg.wheel_fr_angle = wheelFR["angle"]
        msg.wheel_fr_downforce = wheelFR["downForce"]

        msg.wheel_fl_speed = wheelFL["speed"]
        msg.wheel_fl_ang_vel = wheelFL["angVel"]
        msg.wheel_fl_brake_torque = wheelFL["brakeTorque"]
        msg.wheel_fl_prop_torque = wheelFL["propTorque"]
        msg.wheel_fl_angle = wheelFL["angle"]
        msg.wheel_fl_downforce = wheelFL["downForce"]

        msg.wheel_rr_speed = wheelRR["speed"]
        msg.wheel_rr_ang_vel = wheelRR["angVel"]
        msg.wheel_rr_brake_torque = wheelRR["brakeTorque"]
        msg.wheel_rr_prop_torque = wheelRR["propTorque"]
        msg.wheel_rr_angle = wheelRR["angle"]
        msg.wheel_rr_downforce = wheelRR["downForce"]

        msg.wheel_rl_speed = wheelRL["speed"]
        msg.wheel_rl_ang_vel = wheelRL["angVel"]
        msg.wheel_rl_brake_torque = wheelRL["brakeTorque"]
        msg.wheel_rl_prop_torque = wheelRL["propTorque"]
        msg.wheel_rl_angle = wheelRL["angle"]
        msg.wheel_rl_downforce = wheelRL["downForce"]

        msg.steering = data["steering"]
        msg.throttle = data["throttle"]
        msg.brake = data["brake"]
        msg.clutch = data["clutch"]
        msg.pbrake = data["pbrake"]

        msg.steering_input = data["steeringInput"]
        msg.throttle_input = data["throttleInput"]
        msg.brake_input = data["brakeInput"]
        msg.clutch_input = data["clutchInput"]

        driveStatus = data["driveStatus"]
        msg.esc = bool(driveStatus.get("esc", False))
        msg.abs = bool(driveStatus.get("abs", False))
        msg.tcs = bool(driveStatus.get("tcs", False))
        msg.engine_running = bool(driveStatus.get("engineRunning", False))
        msg.is_realistic_drive = bool(driveStatus.get("isRealisticDrive", False))
        msg.mode_4wd = bool(driveStatus.get("mode4WD", False))
        msg.mode_range_box = bool(driveStatus.get("modeRangeBox", False))
        msg.is_front_diff_locked = bool(driveStatus.get("isFrontDiffLocked", False))
        msg.is_rear_diff_locked = bool(driveStatus.get("isRearDiffLocked", False))

        msg.engine_load = data["engineLoad"]
        msg.engine_torque = data["engineTorque"]
        msg.rpm = data["RPM"]
        msg.flywheel_torque = data["flywheelTorque"]
        msg.turbo_boost = data["turboBoost"]
        msg.supercharger_boost = data["superchargerBoost"]

        msg.gearbox_torque = data["gearboxTorque"]
        msg.gear_ratio = data["gearRatio"]
        msg.gear_index = int(data["gearIndex"])

        return msg
