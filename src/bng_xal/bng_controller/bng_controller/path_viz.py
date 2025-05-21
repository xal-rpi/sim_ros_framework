#!/usr/bin/env python3
from sys import stderr
from time import sleep
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import Path
from bng_msgs.msg import GtStateMsg
import tf2_ros
from bng_simulator.utils.resource_manager import ResourceManager


class PathVisAdapter(Node):
    def __init__(self):
        super().__init__("path_vis_adapter")

        self.declare_parameter("path_file", "circle.csv")

        # Broadcast a static "world" → "map" identity transform so RViz sees 'map'
        self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = "world"
        static_tf.child_frame_id = "map"
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(static_tf)

        # Load centerline CSV
        pf = ResourceManager.get_path(
            "bng_controller", "paths/" + self.get_parameter("path_file").value
        )
        wps = np.loadtxt(pf, delimiter=",")

        # PlotJuggler topics
        self.pub_px = self.create_publisher(Float32MultiArray, "/path_x", 1)
        self.pub_py = self.create_publisher(Float32MultiArray, "/path_y", 1)
        self.pub_vx = self.create_publisher(Float32, "/veh_x", 1)
        self.pub_vy = self.create_publisher(Float32, "/veh_y", 1)

        # RViz topics
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        for x, y in wps:
            ps = PoseStamped()
            ps.header = self.path_msg.header
            ps.pose = Pose(
                position=Point(x=x, y=y, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            self.path_msg.poses.append(ps)
        self.pub_path = self.create_publisher(Path, "/reference_path", 1)
        self.pub_pose = self.create_publisher(PoseStamped, "/vehicle_pose", 1)

        # Subscribe to GT-state
        self.create_subscription(GtStateMsg, "/ego/gtstate", self.cb_gt, 10)

        # Publish static data once
        msg_x = Float32MultiArray(data=list(wps[:, 0]))
        msg_y = Float32MultiArray(data=list(wps[:, 1]))
        self.pub_px.publish(msg_x)
        self.pub_py.publish(msg_y)
        self.pub_path.publish(self.path_msg)

    def cb_gt(self, msg: GtStateMsg):
        # Publish current vehicle pose for RViz
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose = Pose(
            position=Point(x=msg.pos.x, y=msg.pos.y, z=msg.pos.z),
            orientation=Quaternion(
                x=msg.quat.x, y=msg.quat.y, z=msg.quat.z, w=msg.quat.w
            ),
        )
        self.pub_pose.publish(ps)

        # Publish live XY for PlotJuggler
        vx = Float32(data=msg.pos.x)
        vy = Float32(data=msg.pos.y)
        self.pub_vx.publish(vx)
        self.pub_vy.publish(vy)


def main():
    rclpy.init()
    node = PathVisAdapter()
    exit_code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught, cleaning up interface...")
    except Exception as e:
        print("Uncaught exception:", e, file=stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    exit(exit_code)


if __name__ == "__main__":
    main()
