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
from bng_msgs.srv import OverrideTargets
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
        if self.config is None:
            raise RuntimeError("Could not open config.")

        llc_cfg = self.config["vehicles"]["ego"]["controllers"]["LowLevelController"]
        self.listen_ip = llc_cfg.get("listenIp", "127.0.0.1")
        self.listen_port = llc_cfg.get("listenPort", 0)
        self.send_ip = llc_cfg.get("sendIp", "127.0.0.1")
        self.send_port = llc_cfg.get("sendPort", 0)

        hlc_cfg = self.config["high_level_controller"]
        self.control_full_path = hlc_cfg["control_fn"]
        self.control_rate = hlc_cfg["control_rate"]

        control_prefix, control_path = self.control_full_path.split("://", 1)
        if not ":" in control_path:
            raise ValueError(
                "Missing ':' in control function path, format is (file|core)://module:function"
            )
        control_module, self.control_function_name = control_path.rsplit(":", 1)

        if control_prefix == "file":
            if not os.path.isabs(control_module):
                self.get_logger().warning(
                    f"Path for file:// scheme is not absolute: {control_module}. Resolution might be unpredictable if not in sys.path."
                )

            try:
                module_name = (
                    "custom_control_module_"
                    + os.path.splitext(os.path.basename(control_module))[0]
                )

                spec = importlib.util.spec_from_file_location(
                    module_name, control_module
                )
                if spec is None:
                    raise ImportError(
                        f"Could not create module spec for file: {control_module}"
                    )

                custom_module = importlib.util.module_from_spec(spec)
                if custom_module is None:
                    raise ImportError(
                        f"Could not create module from spec for file: {control_module}"
                    )

                spec.loader.exec_module(custom_module)

                self.compute_control = getattr(
                    custom_module, self.control_function_name
                )
                self.get_logger().info(
                    f"Successfully loaded control function '{self.control_function_name}' from file: {control_module}"
                )

            except FileNotFoundError:
                self.get_logger().error(f"Control file not found: {control_module}")
                raise
            except AttributeError:
                self.get_logger().error(
                    f"Function '{self.control_function_name}' not found in file: {control_module}"
                )
                raise
            except Exception as e:
                self.get_logger().error(
                    f"Error loading control function from file '{control_module}': {e}"
                )
                raise

        elif control_prefix == "core":
            try:
                # control_module already holds the Python module path, e.g., "my_package.my_module"
                self.get_logger().debug(
                    f"module : {control_module}, function : {self.control_function_name}"
                )
                imported_module = importlib.import_module(
                    "bng_controller.core." + control_module
                )

                self.compute_control = getattr(
                    imported_module, self.control_function_name
                )
                self.get_logger().info(
                    f"Successfully loaded control function '{self.control_function_name}' from module: {control_module}"
                )

            except ModuleNotFoundError:
                self.get_logger().error(
                    f"Control module not found: {control_module}"
                )  # Scheme context is implicitly pymod
                raise
            except AttributeError:
                self.get_logger().error(
                    f"Function '{self.control_function_name}' not found in module: {control_module}"
                )
                raise
            except Exception as e:
                self.get_logger().error(
                    f"Error loading control function from module '{control_module}': {e}"
                )
                raise
        else:
            raise ValueError(
                f"Prefix '{control_prefix}' is unknown for high level controller"
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

        self.override_targets = None
        self.override_expiry_time = None  # Will store an rclpy.time.Time object

        # pubs & subs
        self.create_subscription(Bool, "simulation_ready", self._on_sim_ready, 1)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 1)
        self.target_pub = self.create_publisher(HLCMsg, "hlc_msg", 1)

        self.override_service = self.create_service(
            OverrideTargets,
            "~/override_targets",  # Node-private service
            self._override_targets_callback,
        )
        self.get_logger().info("Created override_targets service.")

        # UDP sockets
        self._init_udp()
        self.max_consec_timeouts = 30

        # control timer
        self.timer = self.create_timer(self.control_rate, self._control_callback)

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

        # reset & start timer
        self.timer.reset()
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

    def _override_targets_callback(self, request, response):
        self.get_logger().info(
            f"Received override request: {request.target_labels}, "
            f"values: {request.target_values}, lifetime: {request.lifetime_sec}s"
        )

        if len(request.target_labels) != len(request.target_values):
            response.success = False
            response.message = (
                "Mismatch between target_labels and target_values length."
            )
            self.get_logger().error(response.message)
            return response

        self.override_targets = dict(zip(request.target_labels, request.target_values))

        if request.lifetime_sec > 0:
            self.override_expiry_time = self.get_clock().now() + Duration(
                seconds=request.lifetime_sec
            )
        else:  # A lifetime of 0 or less means apply indefinitely or until cleared
            self.override_expiry_time = (
                None  # Or a very far future time if Time object is always expected
            )

        response.success = True
        response.message = "Targets overridden successfully."
        self.get_logger().info(
            f"Targets overridden. Expiry set to: {self.override_expiry_time}"
        )
        return response

    def _control_callback(self):
        if self.exit_event.is_set():
            self.stop()
            return
        if not self.running or self.latest_sensor_data == {}:
            return

        active_override = False
        if self.override_targets is not None:  # Override targets
            if self.override_expiry_time is None:  # Indefinite
                active_override = True
            elif self.get_clock().now() < self.override_expiry_time:
                active_override = True
            else:
                self.get_logger().info("Override targets expired.")
                self.override_targets = None  # Clear expired override
                self.override_expiry_time = None

        if active_override:
            targets = self.override_targets
            # Ensure 'time' key is present if not already in override_targets,
            # as the original logic adds it.
            if "time" not in targets:
                targets["time"] = 0
            self.get_logger().debug(
                f"Using overridden targets: {targets}", throttle_duration_sec=2
            )
        else:
            self.get_logger().debug(
                f"Calling {self.control_function_name} with args : {self.latest_sensor_data, self.control_rate, self.metrics.max_latency}",
                throttle_duration_sec=10,
            )
            targets = self.compute_control(
                self.latest_sensor_data, self.control_rate, self.metrics.max_latency
            )

        try:
            if targets is None:
                raise RuntimeError("targets is None")
            pkt = json.dumps(targets).encode("utf-8")
            self.send_socket.sendto(pkt, (self.listen_ip, self.listen_port))
            self.get_logger().debug(
                f"Sent to {self.listen_ip}:{self.listen_port} target {targets}",
                throttle_duration_sec=2,
            )
            self.last_command_time = time.time()

            # --- publish dynamic targets to /current_target ---
            ros_msg = HLCMsg()
            ros_msg.header = convert_time_to_header(
                targets.get("time", self.last_command_time)
            )
            ros_msg.controller_latency = float(self.metrics.average)

            # assume targets["targets"] is a list of dicts, all with the same keys:
            first = targets["targets"][0]
            keys  = sorted(first.keys())
            ros_msg.target_labels = keys
            ros_msg.target_values = [
                float(tgt[k]) for tgt in targets["targets"] for k in keys
            ]

            self.target_pub.publish(ros_msg)

        except Exception as e:
            self.get_logger().error(f"Send error: {e}, target : {targets}")

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
