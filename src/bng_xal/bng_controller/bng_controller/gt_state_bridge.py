#!/usr/bin/env python3
"""
GT State Bridge Node

Receives reduced ground truth state from BeamNG Lua controller_manager via UDP
and publishes to ROS topics. Automatically discovers vehicle configurations
from sim_manager_node and creates publishers for each vehicle.
"""

import socket
import json
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from bng_simulator.utils.services_utils import convert_time_to_header, send_request
from bng_msgs.msg import ReducedGtStateMsg


class VehicleGtReceiver:
    """Manages UDP reception and ROS publishing for a single vehicle."""
    
    def __init__(self, vehicle_name: str, listen_ip: str, listen_port: int, 
                 publisher, logger, frame_id: str = "map"):
        self.vehicle_name = vehicle_name
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.publisher = publisher
        self.logger = logger
        self.frame_id = frame_id
        
        self.running = False
        self.socket = None
        self.thread = None
        self.message_count = 0
        self.last_receive_time = 0.0
        
    def start(self):
        """Initialize UDP socket and start receiver thread."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((self.listen_ip, self.listen_port))
            self.socket.settimeout(0.2)
            self.logger.info(
                f"[{self.vehicle_name}] Bound UDP receiver on {self.listen_ip}:{self.listen_port}"
            )
        except Exception as e:
            self.logger.error(
                f"[{self.vehicle_name}] Failed to bind UDP socket: {e}"
            )
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        return True
    
    def stop(self):
        """Stop receiver thread and close socket."""
        if not self.running:
            return
        
        self.logger.info(f"[{self.vehicle_name}] Stopping receiver...")
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=0.5)
        
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
        
        self.logger.info(
            f"[{self.vehicle_name}] Stopped. Received {self.message_count} messages."
        )
    
    def _receive_loop(self):
        """Main UDP receive loop running in dedicated thread."""
        self.logger.info(f"[{self.vehicle_name}] Receiver thread started")
        consecutive_timeouts = 0
        max_consecutive_timeouts = 50  # ~10s at 0.2s timeout
        
        while self.running:
            try:
                data, _ = self.socket.recvfrom(8192)
                self.last_receive_time = time.time()
                consecutive_timeouts = 0
                
                # Decode JSON payload
                state_dict = json.loads(data.decode())
                
                # Publish ROS message
                self._publish_state(state_dict)
                self.message_count += 1
                
            except socket.timeout:
                consecutive_timeouts += 1
                if consecutive_timeouts >= max_consecutive_timeouts:
                    self.logger.warn(
                        f"[{self.vehicle_name}] No data for {max_consecutive_timeouts * 0.2}s"
                    )
                    consecutive_timeouts = 0  # Reset to avoid spam
            except json.JSONDecodeError as e:
                self.logger.error(f"[{self.vehicle_name}] JSON decode error: {e}")
            except Exception as e:
                self.logger.error(f"[{self.vehicle_name}] Receive error: {e}")
                break
        
        self.logger.info(f"[{self.vehicle_name}] Receiver thread exiting")
    
    def _publish_state(self, state: Dict):
        """Convert state dict to ROS message and publish."""
        msg = ReducedGtStateMsg()
        
        # Header with BeamNG simulation time
        msg.header = convert_time_to_header(state.get('t', 0.0), self.frame_id)
        msg.time = state.get('t', 0.0)
        
        # Extract fields with defaults
        msg.x = float(state.get('x', 0.0))
        msg.y = float(state.get('y', 0.0))
        msg.yaw = float(state.get('yaw', 0.0))
        
        msg.vel_mag = float(state.get('V', 0.0))
        msg.vx = float(state.get('vx', 0.0))
        msg.vy = float(state.get('vy', 0.0))
        
        msg.beta = float(state.get('beta', 0.0))
        msg.r = float(state.get('r', 0.0))
        
        msg.delta = float(state.get('delta', 0.0))
        msg.wr = float(state.get('wr', 0.0))
        msg.wf = float(state.get('wf', 0.0))
        msg.we = float(state.get('we', 0.0))
        
        msg.pb = float(state.get('pb', 0.0))
        msg.throttle = float(state.get('throttle', 0.0))
        msg.brake = float(state.get('brake', 0.0))
        
        msg.accel_x = float(state.get('accel_x', 0.0))
        msg.accel_y = float(state.get('accel_y', 0.0))
        
        msg.rear_wheel_torque_est = float(state.get('rear_wheel_torque_est', 0.0))
        
        self.publisher.publish(msg)


class GtStateBridge(Node):
    """
    GT State Bridge Node
    
    Discovers vehicle configurations from sim_manager and creates UDP receivers
    and ROS publishers for each vehicle's reduced ground truth state.
    """
    
    def __init__(self):
        super().__init__("gt_state_bridge")
        
        # Parameters
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("frame_id", "map")
        
        # Set log level
        log_level_str = self.get_parameter("log_level").value.upper()
        lvl_map = {
            "DEBUG": rclpy.logging.LoggingSeverity.DEBUG,
            "INFO": rclpy.logging.LoggingSeverity.INFO,
            "WARN": rclpy.logging.LoggingSeverity.WARN,
            "ERROR": rclpy.logging.LoggingSeverity.ERROR,
            "FATAL": rclpy.logging.LoggingSeverity.FATAL,
        }
        severity = lvl_map.get(log_level_str, rclpy.logging.LoggingSeverity.INFO)
        rclpy.logging.set_logger_level(self.get_logger().name, severity)
        
        self.frame_id = self.get_parameter("frame_id").value
        self.receivers: Dict[str, VehicleGtReceiver] = {}
        
        # Wait for simulation and fetch config
        self.get_logger().info("Waiting for simulation to be ready...")
        self._wait_for_sim_ready()
        
        # Fetch manager config and setup receivers
        self._setup_vehicle_receivers()
        
        self.get_logger().info(
            f"GT State Bridge initialized with {len(self.receivers)} vehicles"
        )
    
    def _wait_for_sim_ready(self, max_attempts: int = 60, retry_delay: float = 0.5):
        """Poll is_sim_ready service until simulation is ready."""
        for attempt in range(max_attempts):
            try:
                result = send_request(
                    function_name="is_sim_ready",
                    function_args={},
                    timeout_sec=2.0,
                    node_ros=self,
                )
                if result and result.get("ready", False):
                    self.get_logger().info("Simulation is ready!")
                    return
            except Exception as e:
                self.get_logger().debug(f"Error checking readiness: {e}")
            
            self.get_logger().debug(
                f"Sim not ready, retrying ({attempt + 1}/{max_attempts})..."
            )
            time.sleep(retry_delay)
        
        raise RuntimeError("Simulation did not become ready in time")
    
    def _setup_vehicle_receivers(self):
        """Fetch vehicle configs and create receivers for each vehicle."""
        # Fetch full manager config
        self.get_logger().info("Fetching manager config...")
        manager_config = send_request(
            function_name="get_manager_config",
            function_args={},
            timeout_sec=5.0,
            node_ros=self,
        )
        
        if manager_config is None:
            raise RuntimeError("Failed to fetch manager config")
        
        # Extract vehicles config
        vehicles_config = manager_config.get("vehicles", {})
        if not vehicles_config:
            self.get_logger().warn("No vehicles found in config")
            return
        
        self.get_logger().info(f"Found {len(vehicles_config)} vehicles in config")
        
        # Create receiver for each vehicle
        for vehicle_name, vehicle_cfg in vehicles_config.items():
            self._create_vehicle_receiver(vehicle_name, vehicle_cfg)
    
    def _create_vehicle_receiver(self, vehicle_name: str, vehicle_cfg: Dict):
        """Create UDP receiver and ROS publisher for a single vehicle."""
        # Extract LowLevelController config for UDP endpoints
        llc_cfg = vehicle_cfg.get("controllers", {}).get("LowLevelController", {})
        
        if not llc_cfg:
            self.get_logger().warn(
                f"[{vehicle_name}] No LowLevelController config found, skipping"
            )
            return
        
        # Get UDP endpoints
        # Lua sends to sendIp:sendPort, so we receive on that endpoint
        listen_ip = llc_cfg.get("sendIp")
        listen_port = llc_cfg.get("sendPort")
        
        if not listen_ip or not listen_port:
            self.get_logger().warn(
                f"[{vehicle_name}] Missing sendIp/sendPort, skipping"
            )
            return
        
        # Create ROS publisher for this vehicle
        topic_name = f"/{vehicle_name}/reduced_state"
        publisher = self.create_publisher(ReducedGtStateMsg, topic_name, 10)
        self.get_logger().info(f"[{vehicle_name}] Created publisher on {topic_name}")
        
        # Create and start receiver
        receiver = VehicleGtReceiver(
            vehicle_name=vehicle_name,
            listen_ip=listen_ip,
            listen_port=listen_port,
            publisher=publisher,
            logger=self.get_logger(),
            frame_id=self.frame_id,
        )
        
        if receiver.start():
            self.receivers[vehicle_name] = receiver
            self.get_logger().info(f"[{vehicle_name}] GT state receiver started")
        else:
            self.get_logger().error(f"[{vehicle_name}] Failed to start receiver")
    
    def destroy_node(self):
        """Clean up all receivers before shutdown."""
        self.get_logger().info("Shutting down GT State Bridge...")
        
        for vehicle_name, receiver in self.receivers.items():
            receiver.stop()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    
    try:
        node = GtStateBridge()
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught, cleaning up GT State Bridge...")
    
    except Exception as e:
        print(f"Uncaught exception: {e}")
        import traceback
        traceback.print_exc()
        exit_code = 1
    
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
    
    rclpy.shutdown()
    exit(exit_code)


if __name__ == "__main__":
    main()
