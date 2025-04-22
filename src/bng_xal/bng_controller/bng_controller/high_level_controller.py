#!/usr/bin/env python3
import socket
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

# fallback if C extension missing
try:
    from bng_controller.core import controller_core
except ImportError:
    rclpy.logging.get_logger("high_level_controller").error(
        "C extension not found, using Python fallback"
    )

    class controller_core:
        @staticmethod
        def compute_control_targets(_):
            return (0.0, 0.0, 0.0)


class HighLevelController(Node):
    def __init__(self):
        super().__init__("high_level_controller")

        # declare all params, including our sim_start_delay
        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 64258)
        self.declare_parameter("send_ip", "172.26.32.1")
        self.declare_parameter("send_port", 64257)
        self.declare_parameter("control_rate", 0.01)
        self.declare_parameter("sim_start_delay", 1.0)
        self.declare_parameter("log_level", "INFO")
        self.log_level_str = self.get_parameter("log_level").value.upper()
        level_map = {
            "DEBUG": rclpy.logging.LoggingSeverity.DEBUG,
            "INFO": rclpy.logging.LoggingSeverity.INFO,
            "WARN": rclpy.logging.LoggingSeverity.WARN,
            "ERROR": rclpy.logging.LoggingSeverity.ERROR,
            "FATAL": rclpy.logging.LoggingSeverity.FATAL,
        }

        severity = level_map.get(self.log_level_str, rclpy.logging.LoggingSeverity.INFO)
        rclpy.logging.set_logger_level(self.get_logger().name, severity)

        # pull them out
        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = self.get_parameter("listen_port").value
        self.send_ip = self.get_parameter("send_ip").value
        self.send_port = self.get_parameter("send_port").value
        self.control_rate = self.get_parameter("control_rate").value
        self.sim_start_delay = self.get_parameter("sim_start_delay").value

        # State
        self.latest_sensor_data = {}
        self.running = False
        self.exit_event = threading.Event()
        self.message_counter = 0
        self.latency_values = []
        self.last_command_time = 0.0
        self.command_counter = 0

        # Publishers & subscribers
        self.status_pub = self.create_publisher(Bool, "controller_status", 1)
        self.latency_pub = self.create_publisher(Float32, "controller_latency", 1)
        self.create_subscription(Bool, "simulation_ready", self._on_sim_ready, 1)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 1)

        # dummy callbacks / timer (we will only start them after sim_ready)
        self._init_udp()
        self.timer = self.create_timer(self.control_rate, self._control_callback)

        self.get_logger().info("HLC initialized; waiting for simulation_ready")

    def _init_udp(self):
        """set up UDP sockets but do not start receive loop yet."""
        self.listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_socket.bind((self.listen_ip, self.listen_port))
        # small timeout so recvfrom can check exit_event
        self.listen_socket.settimeout(0.2)

        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _on_sim_ready(self, msg: Bool):
        """callback when /simulation_ready arrives."""
        if not msg.data or self.running:
            return

        self.get_logger().info(
            f"Received simulation_ready; delaying start by " f"{self.sim_start_delay}s"
        )
        # schedule `start()` on a background timer/thread
        threading.Timer(self.sim_start_delay, self._delayed_start).start()

    def _delayed_start(self):
        """Actually start receive-thread + timer."""
        if self.running:
            return

        # publish status = True
        st = Bool()
        st.data = True
        self.status_pub.publish(st)

        # receive thread
        self.receive_thread = threading.Thread(
            target=self._receive_sensor_data, daemon=True
        )
        self.running = True
        self.receive_thread.start()

        # now start the control timer
        self.timer.reset()
        self.get_logger().info("HLC started")

    def _receive_sensor_data(self):
        self.get_logger().info("Receive thread running")
        consecutive_timeouts = 0
        while self.running and not self.exit_event.is_set():
            try:
                data, _ = self.listen_socket.recvfrom(8192)
                receive_time = time.time()
                consecutive_timeouts = 0

                sensor = json.loads(data.decode())
                self.latest_sensor_data = sensor
                self.message_counter += 1

                # latency calc...
                if self.last_command_time:
                    latency = receive_time - self.last_command_time
                    self.latency_values.append(latency)
                    if len(self.latency_values) > 100:
                        self.latency_values.pop(0)
                    if self.message_counter % 10 == 0:
                        avg = sum(self.latency_values) / len(self.latency_values)
                        lm = Float32()
                        lm.data = float(avg)
                        self.latency_pub.publish(lm)

            except socket.timeout:
                consecutive_timeouts += 1
                # *** no longer killing the node on timeouts! ***
                if consecutive_timeouts % 50 == 0:
                    self.get_logger().warn(
                        f"Still no data after " f"{consecutive_timeouts} timeouts"
                    )
            except Exception as e:
                self.get_logger().error(f"Receive error: {e}")
                break

        self.get_logger().info("Receive thread exiting")
        try:
            self.listen_socket.close()
        except:
            pass

    def _cmd_vel_callback(self, msg: Twist):
        self.get_logger().debug(
            f"Got cmd_vel: lin={msg.linear.x}, " f"ang={msg.angular.z}"
        )

    def _control_callback(self):
        """runs at self.control_rate once we've started."""
        if not self.running:
            return

        now = time.time()
        if not self.latest_sensor_data:
            # default startup behavior
            tgt = {
                "engine_torque": 0.0,
                "road_wheel_angle": 0.0,
                "brake_torque": 0.0,
                "timestamp": int(now * 1000),
            }
        else:
            et, wa, bt = controller_core.compute_control_targets(
                self.latest_sensor_data
            )
            tgt = {
                "engine_torque": et,
                "road_wheel_angle": wa,
                "brake_torque": bt,
                "timestamp": int(now * 1000),
            }

        # send it
        try:
            pkt = json.dumps(tgt).encode("utf-8")
            self.send_socket.sendto(pkt, (self.send_ip, self.send_port))
            self.get_logger().debug(
                f"Send target {tgt}",
                throttle_duration_sec=2,
            )
            self.last_command_time = now
            self.command_counter += 1
        except Exception as e:
            self.get_logger().error(f"Send error: {e}")

    def stop(self):
        """stop receive thread + timer + send zeros."""
        if not self.running:
            return
        self.get_logger().info("Stopping HLC...")
        self.running = False
        self.exit_event.set()
        if self.receive_thread:
            self.receive_thread.join(0.2)

        # send a few zero-commands for safety
        zero = {
            "engine_torque": 0.0,
            "road_wheel_angle": 0.0,
            "brake_torque": 1000.0,
            "timestamp": int(time.time() * 1000),
        }
        for _ in range(3):
            try:
                self.send_socket.sendto(
                    json.dumps(zero).encode(), (self.send_ip, self.send_port)
                )
                time.sleep(0.05)
            except:
                pass
        self.get_logger().info("Zero controls sent")

    def destroy(self):
        self.stop()
        try:
            super().destroy_node()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = HighLevelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down")
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
