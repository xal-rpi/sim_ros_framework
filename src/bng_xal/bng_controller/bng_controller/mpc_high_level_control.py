#!/usr/bin/env python3
import socket
import select
import json
import threading
import time
import logging
import os
import importlib
import importlib.util
from sys import exit, stderr
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from bng_simulator.utils.config_manager import ConfigManager
from bng_simulator.utils.services_utils import convert_time_to_header
from bng_msgs.msg import HLCMsg
from rclpy.duration import Duration


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
        self.declare_parameter("config_path", "nn_mpc_scenario.yaml")
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
        if self.config is None:
            raise RuntimeError("Could not open config.")

        # TODO: Name is hardcoded, that could be a problem?
        llc_cfg = self.config["vehicles"]["ego"]["controllers"]["LowLevelController"]
        self.listen_ip = llc_cfg.get("listenIp", "127.0.0.1")
        self.listen_port = llc_cfg.get("listenPort", 0)
        self.send_ip = llc_cfg.get("sendIp", "127.0.0.1")
        self.send_port = llc_cfg.get("sendPort", 0)

        self.sim_start_delay = 1  # second
        self.max_consec_timeouts = 10  # max consecutive timeouts before error

        # internal state
        self.latest_sensor_data = {}
        self.running = False
        self.exit_event = threading.Event()
        
        # real time metrics
        self.metrics = PerformanceMetrics(window_size=100)
        self.last_command_time = 0.0
        self.message_counter = 0

        self.override_targets = None
        self.override_expiry_time = None  # Will store an rclpy.time.Time object

        # pubs & subs
        self.create_subscription(Bool, "simulation_ready", self._on_sim_ready, 1)

        # UDP sockets
        self._init_udp()
        self.get_logger().info("HLC initialized; waiting for simulation_ready")

    def _init_udp(self):
        self.listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_socket.bind((self.send_ip, self.send_port))
        self.get_logger().info(
            f"Bound listen socket to {self.send_ip}:{self.send_port}"
        )
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

        # start receive thread
        self.receive_thread = threading.Thread(
            target=self._receive_sensor_data, daemon=True
        )
        self.running = True
        self.receive_thread.start()

        self.get_logger().info("HLC started")

    def _recv_last(self, sock, timeout=None, bufsize=8192):
        """
        Block up to `timeout` seconds for *one* packet, then
        drain everything else in the queue, returning only
        the last datagram seen (or None on timeout).
        """
        # wait for at least one packet
        ready, _, _ = select.select([sock], [], [], timeout)
        if not ready:
            return None

        data, _ = sock.recvfrom(bufsize)

        sock.setblocking(False)
        try:
            while True:
                data, _ = sock.recvfrom(bufsize)
        except BlockingIOError:
            # no more packets available right now
            pass
        finally:
            sock.setblocking(True)

        return data

    def _receive_sensor_data(self):
        self.get_logger().info("Receive thread running")
        consecutive_timeouts = 0
        while self.running and not self.exit_event.is_set():
            try:
                data = self._recv_last(self.listen_socket)
                recv_time = time.time()
                consecutive_timeouts = 0
                sensor = json.loads(data.decode())
                self.message_counter += 1
                self.latest_sensor_data = sensor

                # wall clock latency
                if self.last_command_time:
                    lat = recv_time - self.last_command_time
                    self.metrics.add(lat)

            except socket.timeout:
                consecutive_timeouts += 1
                if consecutive_timeouts % self.max_consec_timeouts == 0:
                    self.get_logger().error(
                        f"No data after {consecutive_timeouts} consecutive timeouts"
                    )
            except Exception as e:
                self.get_logger().error(f"Receive error: {e}")
                break

        self.get_logger().info("Receive thread exiting")
        try:
            self.listen_socket.close()
        except Exception:
            pass

    # def _control_callback(self):
    #     if self.exit_event.is_set():
    #         self.stop()
    #         return
    #     if not self.running or self.latest_sensor_data == {}:
    #         return

    #     active_override = False
    #     if self.override_targets is not None:  # Override targets
    #         if self.override_expiry_time is None:  # Indefinite
    #             active_override = True
    #         elif self.get_clock().now() < self.override_expiry_time:
    #             active_override = True
    #         else:
    #             self.get_logger().info("Override targets expired.")
    #             self.override_targets = None  # Clear expired override
    #             self.override_expiry_time = None

    #     if active_override:
    #         targets = self.override_targets
    #         # Ensure 'time' key is present if not already in override_targets,
    #         # as the original logic adds it.
    #         if "time" not in targets:
    #             targets["time"] = 0
    #         self.get_logger().debug(
    #             f"Using overridden targets: {targets}", throttle_duration_sec=2
    #         )
    #     else:
    #         self.get_logger().debug(
    #             f"Calling {self.control_function_name} with args : {self.latest_sensor_data, self.control_rate, self.metrics.max_latency}",
    #             throttle_duration_sec=10,
    #         )
    #         targets = self.compute_control(
    #             self.latest_sensor_data, self.control_rate, self.metrics.max_latency
    #         )

    #     try:
    #         if targets is None:
    #             raise RuntimeError("targets is None")
    #         pkt = json.dumps(targets).encode("utf-8")
    #         self.send_socket.sendto(pkt, (self.listen_ip, self.listen_port))
    #         self.get_logger().debug(
    #             f"Sent to {self.listen_ip}:{self.listen_port} target {targets}",
    #             throttle_duration_sec=2,
    #         )
    #         self.last_command_time = time.time()

    #         # --- publish dynamic targets to /current_target ---
    #         ros_msg = HLCMsg()
    #         ros_msg.header = convert_time_to_header(
    #             targets.get("time", self.last_command_time)
    #         )
    #         ros_msg.controller_latency = float(self.metrics.average)

    #         # assume targets["targets"] is a list of dicts, all with the same keys:
    #         first = targets["targets"][0]
    #         keys  = sorted(first.keys())
    #         ros_msg.target_labels = keys
    #         ros_msg.target_values = [
    #             float(tgt[k]) for tgt in targets["targets"] for k in keys
    #         ]

    #         self.target_pub.publish(ros_msg)

    #     except Exception as e:
    #         self.get_logger().error(f"Send error: {e}, target : {targets}")

    def stop(self):
        """Stop receiving thread and timers, send zero commands."""
        if not self.running:
            return
        self.get_logger().info("Stopping HLC...")
        self.running = False
        self.exit_event.set()
        if self.receive_thread:
            self.receive_thread.join(0.2)
        
        # Proper termination on the UDP socket
        try:
            self.listen_socket.close()
        except Exception as e:
            self.get_logger().error(f"Error closing listen socket: {e}")
        try:
            self.send_socket.close()
        except Exception as e:
            self.get_logger().error(f"Error closing send socket: {e}")
        self.get_logger().info("HLC stopped, sending zero command.")

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
