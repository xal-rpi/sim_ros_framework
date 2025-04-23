#!/usr/bin/env python3
import socket
import json
import threading
import time
from sys import exit, stderr

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
        # --- parameters ---
        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 64258)
        self.declare_parameter("send_ip", "172.26.32.1")
        self.declare_parameter("send_port", 64257)
        self.declare_parameter("control_rate", 0.01)
        self.declare_parameter("sim_start_delay", 1.0)
        self.declare_parameter("log_level", "INFO")

        # set log level
        lvl = self.get_parameter("log_level").value.upper()
        lvl_map = {
            "DEBUG": rclpy.logging.LoggingSeverity.DEBUG,
            "INFO": rclpy.logging.LoggingSeverity.INFO,
            "WARN": rclpy.logging.LoggingSeverity.WARN,
            "ERROR": rclpy.logging.LoggingSeverity.ERROR,
            "FATAL": rclpy.logging.LoggingSeverity.FATAL,
        }
        severity = lvl_map.get(lvl, rclpy.logging.LoggingSeverity.INFO)
        rclpy.logging.set_logger_level(self.get_logger().name, severity)

        # pull parameters
        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = self.get_parameter("listen_port").value
        self.send_ip = self.get_parameter("send_ip").value
        self.send_port = self.get_parameter("send_port").value
        self.control_rate = self.get_parameter("control_rate").value
        self.sim_start_delay = self.get_parameter("sim_start_delay").value

        # internal state
        self.latest_sensor_data = {}
        self.running = False
        self.exit_event = threading.Event()
        self.receive_thread = None
        self.last_command_time = 0.0
        self.latency_values = []
        self.message_counter = 0

        # pubs & subs
        self.status_pub = self.create_publisher(Bool, "controller_status", 1)
        self.latency_pub = self.create_publisher(Float32, "controller_latency", 1)
        self.create_subscription(Bool, "simulation_ready", self._on_sim_ready, 1)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 1)

        # UDP sockets
        self._init_udp()
        self.max_consec_timeouts = 30

        # control timer
        self.timer = self.create_timer(self.control_rate, self._control_callback)

        self.get_logger().info("HLC initialized; waiting for simulation_ready")

    def _init_udp(self):
        self.listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_socket.bind((self.listen_ip, self.listen_port))
        self.listen_socket.settimeout(0.2)
        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _on_sim_ready(self, msg: Bool):
        if not msg.data or self.running:
            return
        self.get_logger().info(
            f"Received simulation_ready; delaying start by " f"{self.sim_start_delay}s"
        )
        threading.Timer(self.sim_start_delay, self._delayed_start).start()

    def _delayed_start(self):
        if self.running:
            return
        # publish status = True
        st = Bool()
        st.data = True
        self.status_pub.publish(st)

        # start receive thread
        self.receive_thread = threading.Thread(
            target=self._receive_sensor_data, daemon=True
        )
        self.running = True
        self.receive_thread.start()

        # reset & start timer
        self.timer.reset()
        self.get_logger().info("HLC started")

    def _receive_sensor_data(self):
        self.get_logger().info("Receive thread running")
        consecutive_timeouts = 0
        while self.running and not self.exit_event.is_set():
            try:
                data, _ = self.listen_socket.recvfrom(8192)
                recv_time = time.time()
                consecutive_timeouts = 0
                sensor = json.loads(data.decode())
                self.latest_sensor_data = sensor
                self.message_counter += 1

                # latency
                if self.last_command_time:
                    lat = recv_time - self.last_command_time
                    self.latency_values.append(lat)
                    if len(self.latency_values) > 100:
                        self.latency_values.pop(0)
                    if self.message_counter % 10 == 0:
                        avg = sum(self.latency_values) / len(self.latency_values)
                        lm = Float32()
                        lm.data = float(avg)
                        self.latency_pub.publish(lm)

            except socket.timeout:
                consecutive_timeouts += 1
                if consecutive_timeouts % self.max_consec_timeouts == 0:
                    self.get_logger().error(
                        f"No data after {consecutive_timeouts} consecutive timeouts, shutting down controller..."
                    )
                    self.exit_event.set()
                    break
            except Exception as e:
                self.get_logger().error(f"Receive error: {e}")
                break

        self.get_logger().info("Receive thread exiting")
        try:
            self.listen_socket.close()
        except Exception:
            pass

    def _cmd_vel_callback(self, msg: Twist):
        self.get_logger().debug(f"Got cmd_vel: lin={msg.linear.x}, ang={msg.angular.z}")

    def _control_callback(self):
        if self.exit_event.is_set():
            self.stop()
            return
        if not self.running:
            return
        now = time.time()
        if not self.latest_sensor_data:
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

        try:
            pkt = json.dumps(tgt).encode("utf-8")
            self.send_socket.sendto(pkt, (self.send_ip, self.send_port))
            self.get_logger().debug(f"Send target {tgt}", throttle_duration_sec=2)
            self.last_command_time = now
        except Exception as e:
            self.get_logger().error(f"Send error: {e}")

    def stop(self):
        """Stop receiving thread and timers, send zero commands."""
        if not self.running:
            return
        self.get_logger().info("Stopping HLC...")
        self.running = False
        self.exit_event.set()
        if self.receive_thread:
            self.receive_thread.join(0.2)

        zero = {
            "engine_torque": 0.0,
            "road_wheel_angle": 0.0,
            "brake_torque": 1000.0,
            "timestamp": int(time.time() * 1000),
        }

        # TODO : Does not work
        for _ in range(1):
            try:
                self.send_socket.sendto(
                    json.dumps(zero).encode("utf-8"), (self.send_ip, self.send_port)
                )
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().debug(f"Could not send zero target: {e}")
                pass
        self.get_logger().info("Zero controls sent")

    def destroy_node(self):
        """Called by main() when shutting down."""
        self.get_logger().info("Cleaning up HighLevelController...")
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0

    try:
        node = HighLevelController()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("KeyboardInterrupt caught, cleaning up HLC...")

    except Exception as e:
        print("Uncaught exception:", e, file=stderr)
        exit_code = 1

    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

    exit(exit_code)


if __name__ == "__main__":
    main()
