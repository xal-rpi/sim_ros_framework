#!/usr/bin/env python3
import socket
import json
import threading
import time
import logging
from sys import exit, stderr
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

from bng_controller.core import controller_core
from bng_simulator.utils.config_manager import ConfigManager


class PerformanceMetrics:
    def __init__(self, window_size: int = 100):
        self._latencies = deque(maxlen=window_size)
        self.max_latency = 0.0

    def add(self, sample: float):
        """Add a new latency sample (in seconds)."""
        self._latencies.append(sample)
        if sample > self.max_latency:
            self.max_latency = sample

    @property
    def average(self) -> float:
        """Return the average over the current window (or 0.0 if empty)."""
        if not self._latencies:
            return 0.0
        return sum(self._latencies) / len(self._latencies)


class HighLevelController(Node):
    def __init__(self):
        super().__init__("high_level_controller")
        # --- parameters ---
        self.declare_parameter("config_path", "")
        self.declare_parameter("log_level", "INFO")

        # set log level
        self.log_level_str = self.get_parameter("log_level").value.upper()
        lvl_map = {
            "FULL": rclpy.logging.LoggingSeverity.DEBUG,
            "DEBUG": rclpy.logging.LoggingSeverity.DEBUG,
            "INFO": rclpy.logging.LoggingSeverity.INFO,
            "WARN": rclpy.logging.LoggingSeverity.WARN,
            "ERROR": rclpy.logging.LoggingSeverity.ERROR,
            "FATAL": rclpy.logging.LoggingSeverity.FATAL,
        }
        severity = lvl_map.get(self.log_level_str, rclpy.logging.LoggingSeverity.INFO)
        rclpy.logging.set_logger_level(self.get_logger().name, severity)

        # Catch all debug from external modules
        if self.log_level_str == "FULL":
            for h in logging.root.handlers[:]:
                logging.root.removeHandler(h)

            fmt = "[%(levelname)s] [%(name)s]: %(message)s"
            logging.basicConfig(
                level=logging.DEBUG,
                stream=stderr,
                format=fmt,
            )

        # pull parameters from config
        cfg = self.get_parameter("config_path").value
        self.config = ConfigManager.get_config(cfg)

        llc_cfg = self.config["vehicles"]["ego"]["controllers"]["LowLevelController"]
        self.listen_ip = llc_cfg.get("listen_ip", "127.0.0.1")
        self.listen_port = llc_cfg.get("listen_port", 25252)
        self.send_ip = llc_cfg.get("send_ip", "127.0.0.1")
        self.send_port = llc_cfg.get("send_port", 25252)

        hlc_cfg = self.config["high_level_controller"]
        self.control_fn_name = hlc_cfg["control_fn"]
        self.control_rate = hlc_cfg["control_rate"]

        try:
            self.compute_control = getattr(controller_core, self.control_fn_name)
        except AttributeError:
            raise RuntimeError(
                f"No function '{self.control_fn_name}' in controller_core"
            )

        self.sim_start_delay = 1  # second

        # internal state
        self.latest_sensor_data = {}
        self.running = False
        self.exit_event = threading.Event()
        # real time metrics
        self.metrics = PerformanceMetrics(window_size=100)
        self.last_command_time = 0.0
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
        self.listen_socket.bind((self.send_ip, self.send_port))
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
                self.message_counter += 1
                # stash both simtime and the low‐level realtime
                self.latest_sensor_data = sensor

                # wall clock latency
                if self.last_command_time:
                    lat = recv_time - self.last_command_time
                    self.metrics.add(lat)
                    # publish every 10 messages
                    if self.message_counter % 10 == 0:
                        m = Float32()
                        m.data = float(self.metrics.average)
                        self.latency_pub.publish(m)

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

        self.get_logger().debug(
            f"Calling {self.control_fn_name} with args : {self.latest_sensor_data, self.control_rate, self.metrics.max_latency}",
            throttle_duration_sec=2,
        )
        targets = self.compute_control(
            self.latest_sensor_data, self.control_rate, self.metrics.max_latency
        )

        try:
            pkt = json.dumps(targets).encode("utf-8")
            self.send_socket.sendto(pkt, (self.listen_ip, self.listen_port))
            self.get_logger().debug(
                f"Sent to {self.listen_ip}:{self.listen_port} target {targets}",
                throttle_duration_sec=2,
            )
            self.last_command_time = time.time()
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
            "brake_torque": 0.0,
            "time": 0,  # apply immediately
        }

        for _ in range(1):
            try:
                self.send_socket.sendto(
                    json.dumps(zero).encode("utf-8"), (self.listen_ip, self.listen_port)
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
