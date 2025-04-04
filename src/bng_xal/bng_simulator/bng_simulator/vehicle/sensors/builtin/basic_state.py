"""
Basic Sensor State. Should already be present for any vehicle instance.
"""

from bng_simulator.vehicle.sensors import SensorBase, SensorRegistry
from bng_simulator.utils.services_utils import convert_time_to_header

from bng_msgs.msg import BasicStateMsg


@SensorRegistry.register("BasicState")
class BasicState(SensorBase):
    """
    Basic sensor state for the vehicle.
    """

    def __init__(self, name, vehicle, beamng, config):
        super().__init__(name, vehicle, beamng, config)
        # Create the sensor
        self.create_sensor()

    def create_sensor(self):
        """
        Create the sensor.
        """
        self._sensor = self._vehicle.sensors["state"]

    def poll(self):
        """
        Retrieve the basic sensor state.
        """
        # This function will poll data from all basic sensors
        # attached via attach_sensor
        self._vehicle.poll_sensors("state")
        self._last_data = self._sensor
        self._all_data = [
            self._sensor,
        ]

    def get_last_data(self):
        """
        Get the last basic sensor data.

        Returns:
            Any: The last basic sensor data.
        """
        return self._sensor

    def get_all_data(self):
        """
        Get all the basic sensor data.

        Returns:
            Any: The basic sensor data.
        """
        return [self._sensor]

    def ros_msg_type(self):
        """
        Get the ROS message type.

        Returns:
            Any: The ROS message type.
        """
        return BasicStateMsg

    def to_ros_msg(self, frame_id="map"):
        """
        Convert the basic sensor state to a ROS message.

        Returns:
            BasicState: The ROS message.
        """
        latest_data = self.get_last_data()
        if latest_data is None:
            return None

        data_time = latest_data["time"]
        # Let's convert the time for the message header

        msg = BasicStateMsg()

        # Header
        msg.header = convert_time_to_header(data_time, frame_id)

        # Time
        msg.time = data_time

        # Position
        msg.position.x = latest_data["pos"][0]
        msg.position.y = latest_data["pos"][1]
        msg.position.z = latest_data["pos"][2]

        # Direction
        msg.direction.x = latest_data["dir"][0]
        msg.direction.y = latest_data["dir"][1]
        msg.direction.z = latest_data["dir"][2]

        # Up vector
        msg.up_vector.x = latest_data["up"][0]
        msg.up_vector.y = latest_data["up"][1]
        msg.up_vector.z = latest_data["up"][2]

        # Velocity
        msg.velocity.x = latest_data["vel"][0]
        msg.velocity.y = latest_data["vel"][1]
        msg.velocity.z = latest_data["vel"][2]

        # Rotation (quaternion)
        msg.rotation.x = latest_data["rotation"][0]
        msg.rotation.y = latest_data["rotation"][1]
        msg.rotation.z = latest_data["rotation"][2]
        msg.rotation.w = latest_data["rotation"][3]

        return msg
