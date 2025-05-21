"""
Implements the custom ground truth state sensor.
"""

from logging import getLogger
from sys import stderr
from typing import Optional

import numpy as np

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID
from beamngpy.types import Float3, StrDict

from beamngpy.beamng import BeamNGpy
from beamngpy.vehicle import Vehicle

from bng_simulator.core.vehicle_properties import get_vehicle_principal_axis
from bng_simulator.vehicle.sensors import SensorBase, SensorRegistry

# Import ROS message type
from bng_msgs.msg import GtStateMsg
from bng_simulator.utils.services_utils import convert_time_to_header


class GtStateWrapper(CommBase):
    """
    An interactive, automated sensor that extract all useful state from the
    Beamng simulator.  This sensor is a custom sensor that is not part of the
    Beamng.tech sensor suite.  It is designed to be used in conjunction with ...
    """

    def __init__(
        self,
        name: str,
        vehicle: Vehicle,
        beamng: BeamNGpy,
        gfx_update_time: float = 0.05,
        physics_update_time: float = 0.01,
        num_physics_steps_for_gfx_save: int = 1,
        pos: Float3 = (0, 0, 0),
        dir: Float3 = (
            1,
            0,
            0,
        ),  # vector relative to the vehicle's forward direction, local frame
        left: Float3 = (
            0,
            1,
            0,
        ),  # vector relative to the vehicle's left direction, local frame
        is_using_gravity: bool = True,
        is_allow_wheel_nodes: bool = False,
        is_visualised: bool = True,
        is_snapping_desired: bool = False,
        is_force_inside_triangle: bool = False,
    ):
        super().__init__(beamng, vehicle)

        self.logger = getLogger(f"{LOGGER_ID}.GtState")

        # Cache some properties we will need later.
        self.name = name
        self.vehicle = vehicle

        # Cache additional vehicle properties.
        self.extract_vehicle_properties()

        # Create and initialise this sensor in the simulation.
        self._open_gt_state(
            name,
            vehicle,
            gfx_update_time,
            physics_update_time,
            num_physics_steps_for_gfx_save,
            pos,
            dir,
            left,
            is_using_gravity,
            is_allow_wheel_nodes,
            is_visualised,
            is_snapping_desired,
            is_force_inside_triangle,
        )

        # Fetch the unique Id number (in the simulator)
        # for this sensor.  We will need this later.
        self.sensorId = self._get_gt_state_id()

    def remove(self) -> None:
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_gt_state()
        self.logger.info("GtState - sensor removed: " f"{self.name}")

    def poll(self) -> StrDict:
        """
        Gets the most-recent readings for this sensor.
        Returns:
        """
        # Send and receive a request for readings data from this sensor.
        readings_data = self._poll_gt_state_ge()
        return readings_data

    def _get_gt_state_id(self) -> int:
        return int(self.send_recv_ge(type="GetGtStateId", name=self.name)["data"])

    def _open_gt_state(
        self,
        name: str,
        vehicle: Vehicle,
        gfx_update_time: float,
        physics_update_time: float,
        num_physics_steps_for_gfx_save: int,
        pos: Float3,
        dir: Float3,
        left: Float3,
        is_using_gravity: bool,
        is_allow_wheel_nodes: bool,
        is_visualised: bool,
        is_snapping_desired: bool,
        is_force_inside_triangle: bool,
    ) -> None:
        data: StrDict = dict()
        data["name"] = name
        data["vid"] = vehicle.vid
        data["GFXUpdateTime"] = gfx_update_time
        data["physicsUpdateTime"] = physics_update_time
        data["numPhysicsStepsForGfxSave"] = num_physics_steps_for_gfx_save
        data["pos"] = self.calculate_cog_pos(pos)
        data["dir"] = self.calculate_dir(dir)
        data["left"] = self.calculate_dir(left)
        data["isUsingGravity"] = is_using_gravity
        data["isAllowWheelNodes"] = is_allow_wheel_nodes
        data["isVisualised"] = is_visualised
        data["isSnappingDesired"] = is_snapping_desired
        data["isForceInsideTriangle"] = is_force_inside_triangle
        data["isDirWorldSpace"] = True
        args = {
            "type": "OpenGtState",
            "ack": "OpenedGtState",
            **data,
        }
        print(args, flush=True, file=stderr)
        self.send_ack_ge(**args)
        self.logger.info(f"Opened GtState sensor: {name} \n{data}")

    def _close_gt_state(
        self,
    ) -> None:
        self.send_ack_ge(
            type="CloseGtState",
            ack="ClosedGtState",
            name=self.name,
            vid=self.vehicle.vid,
        )
        self.logger.info(f'Closed GtState sensor: "{self.name}"')

    def _poll_gt_state_ge(self) -> StrDict:
        return self.send_recv_ge(type="PollGtStateGE", name=self.name)["data"]

    def extract_vehicle_properties(self):
        """
        Extracts the coordinates of the principal axis of the vehicles as
        well as the vehicle's center of mass.
        """
        veh_prop = get_vehicle_principal_axis(self.vehicle)
        self.cogPos = veh_prop["cogPosStatic"]
        self.vectorForward = veh_prop["vectorForward"]
        self.vectorLeft = veh_prop["vectorLeft"]
        self.vectorUp = veh_prop["vectorUp"]
        self.logger.info(f"Vehicle properties extracted: \n{veh_prop}")

    def calculate_cog_pos(self, pos: Float3) -> Float3:
        """
        Calculates the center of gravity position in world coordinates.
        """
        return (
            self.cogPos[0]
            + pos[0] * self.vectorForward[0]
            + pos[1] * self.vectorLeft[0]
            + pos[2] * self.vectorUp[0],
            self.cogPos[1]
            + pos[0] * self.vectorForward[1]
            + pos[1] * self.vectorLeft[1]
            + pos[2] * self.vectorUp[1],
            self.cogPos[2]
            + pos[0] * self.vectorForward[2]
            + pos[1] * self.vectorLeft[2]
            + pos[2] * self.vectorUp[2],
        )

    def calculate_dir(self, dir: Float3) -> Float3:
        """
        Calculates the direction vector in world coordinates.
        """
        return (
            dir[0] * self.vectorForward[0]
            + dir[1] * self.vectorLeft[0]
            + dir[2] * self.vectorUp[0],
            dir[0] * self.vectorForward[1]
            + dir[1] * self.vectorLeft[1]
            + dir[2] * self.vectorUp[1],
            dir[0] * self.vectorForward[2]
            + dir[1] * self.vectorLeft[2]
            + dir[2] * self.vectorUp[2],
        )


@SensorRegistry.register("GtState")
class GtState(SensorBase):
    """
    The custom ground truth state sensor.
    """

    def __init__(self, name: str, vehicle: Vehicle, beamng: BeamNGpy, config: dict):
        super().__init__(name, vehicle, beamng, config)
        # Create the sensor instance.
        self._sensor = GtStateWrapper(name, vehicle, beamng, **config)
        self.__DEG_TO_RAD = np.pi / 180.0

    def poll(self):
        """
        Poll the sensor for the latest data.
        """
        # Poll the sensor for the latest data.
        all_readings = self._sensor.poll()

        # If there is no data, set the last data to None.
        if len(all_readings) == 0:
            self._last_data = None
            self._all_data = []
            return

        # If single data or list of data, convert to list of data.
        if type(all_readings) == dict:
            assert 0.0 not in all_readings, "0.0 in all_readings"
            self._all_data = [
                all_readings,
            ]
        else:
            assert type(all_readings) == list, "all_readings is not a list"
            self._all_data = all_readings

        self.process_data()
        self._last_data = self._all_data[-1]

    def process_data(self):
        """
        Process the sensor data.
        """
        # Process the sensor data.
        for data in self._all_data:
            # Let's convert the steering to radians
            data["steering"] = data["steering"] * self.__DEG_TO_RAD

    def ros_msg_type(self):
        """
        Get the ROS message type.

        Returns:
            Any: The ROS message type.
        """
        return GtStateMsg

    def to_ros_msg(self, data: Optional[dict] = None, frame_id="map"):
        """
        Convert the basic sensor state to a ROS message.

        Returns:
            Any: The ROS message.
        """
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
        msg.ang_accel.x, msg.ang_accel.y, msg.ang_accel.z = data["angAccel"]
        msg.pos.x, msg.pos.y, msg.pos.z = data["pos"]
        msg.quat.x, msg.quat.y, msg.quat.z, msg.quat.w = data["quat"]

        # Cache wheel data
        wheelFR = data["wheelFR"]
        wheelFL = data["wheelFL"]
        wheelRR = data["wheelRR"]
        wheelRL = data["wheelRL"]

        msg.wheel_fr_speed = wheelFR["speed"]
        msg.wheel_fr_ang_vel_b = wheelFR["angVelB"]
        msg.wheel_fr_ang_vel = wheelFR["angVel"]
        msg.wheel_fr_brake_torque = wheelFR["brakeTorque"]
        msg.wheel_fr_prop_torque = wheelFR["propTorque"]
        msg.wheel_fr_angle = wheelFR["angle"]

        msg.wheel_fl_speed = wheelFL["speed"]
        msg.wheel_fl_ang_vel_b = wheelFL["angVelB"]
        msg.wheel_fl_ang_vel = wheelFL["angVel"]
        msg.wheel_fl_brake_torque = wheelFL["brakeTorque"]
        msg.wheel_fl_prop_torque = wheelFL["propTorque"]
        msg.wheel_fl_angle = wheelFL["angle"]

        msg.wheel_rr_speed = wheelRR["speed"]
        msg.wheel_rr_ang_vel_b = wheelRR["angVelB"]
        msg.wheel_rr_ang_vel = wheelRR["angVel"]
        msg.wheel_rr_brake_torque = wheelRR["brakeTorque"]
        msg.wheel_rr_prop_torque = wheelRR["propTorque"]
        msg.wheel_rr_angle = wheelRR["angle"]

        msg.wheel_rl_speed = wheelRL["speed"]
        msg.wheel_rl_ang_vel_b = wheelRL["angVelB"]
        msg.wheel_rl_ang_vel = wheelRL["angVel"]
        msg.wheel_rl_brake_torque = wheelRL["brakeTorque"]
        msg.wheel_rl_prop_torque = wheelRL["propTorque"]
        msg.wheel_rl_angle = wheelRL["angle"]

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
