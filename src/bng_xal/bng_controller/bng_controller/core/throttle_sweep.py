import collections
from rclpy.node import get_logger


class ThrottleSweepLogic:
    def __init__(self, control_rate):
        self.logger = get_logger(self.__class__.__name__)
        self.logger.info(
            f"ThrottleSweepLogic initializing with control_rate: {control_rate}"
        )

        self.control_rate = control_rate  # Expected control rate in seconds
        self.throttle_levels = [i / 10.0 for i in range(1, 11)]  # 0.1 to 1.0

        self.max_speed_plateau_duration = 3.0  # seconds
        self.max_speed_epsilon = 0.1  # m/s
        self.stop_speed_threshold = 0.05  # m/s

        # RPM limiter
        self.rpm_hit_required = 2
        self.rpm_limiter_min_speed = 30  # m/s
        self.rpm_limiter_threshold = 7950.0
        self.rpm_limiter_hysteresis = 400.0
        self.rpm_limiter_min_interval = 2
        self.rpm_limiter_last_hit_time = -1e9
        self.rpm_limiter_hit_count = 0
        self.rpm_limiter_exceeded = False

        # Calculate min_readings_for_plateau based on control_rate.
        # If control_rate is very small (high frequency), this could be large.
        # If control_rate is 0 or None, use a default.
        if self.control_rate and self.control_rate > 0:
            self.min_readings_for_plateau = int(
                self.max_speed_plateau_duration / self.control_rate
            )
        else:
            self.logger.warning(
                f"Control rate is {self.control_rate}, using default plateau readings (60 for ~20Hz over 3s)."
            )
            self.min_readings_for_plateau = int(
                self.max_speed_plateau_duration / 0.05
            )  # Default assuming 20Hz

        if self.min_readings_for_plateau < 5:  # Ensure a minimum number of readings
            self.logger.warning(
                f"Calculated min_readings_for_plateau is {self.min_readings_for_plateau}, setting to 5."
            )
            self.min_readings_for_plateau = 5

        self.stop_duration_consecutive_readings = 1 / self.control_rate  # 1 second

        self.recent_speeds = collections.deque(maxlen=self.min_readings_for_plateau)

        # State variables
        self.current_throttle_index = 0
        self.phase = "idle"
        self.stop_readings_count = 0
        self.sim_time = 0.0  # Current simulation time from LLC
        self.plateau_check_sim_time_start = 0.0 # Sim time when we started having enough data for plateau check
        self.initial_run = True  # To trigger initial reset

        self.logger.info(
            f"ThrottleSweepLogic initialized. Plateau duration: {self.max_speed_plateau_duration}s, "
            f"Plateau readings needed: {self.min_readings_for_plateau} (based on {self.control_rate}s rate)."
        )
        self.reset()  # Initialize state correctly

    def reset(self):
        self.logger.info("Resetting ThrottleSweepLogic state.")
        self.current_throttle_index = 0
        # self.phase = 'idle' # Start in idle, compute_control will transition
        self.rpm_limiter_last_hit_time = -1e9
        self.rpm_limiter_hit_count = 0
        self.rpm_limiter_exceeded = False
        self.phase = "starting_level"  # Or start directly here if auto-start is desired
        self.recent_speeds.clear()
        self.stop_readings_count = 0
        self.plateau_check_sim_time_start = 0.0
        self.initial_run = False  # Reset complete

    def compute_control(self, latest_sensor_data, control_rate_val, max_latency_val):
        # Update control_rate if it changed (though typically fixed)
        if self.control_rate != control_rate_val:
            self.logger.info(
                f"Control rate updated from {self.control_rate} to {control_rate_val}"
            )
            self.control_rate = control_rate_val
            # Recalculate dependent parameters if necessary, e.g., min_readings_for_plateau
            if self.control_rate and self.control_rate > 0:
                new_min_readings = int(
                    self.max_speed_plateau_duration / self.control_rate
                )
                if new_min_readings < 5:
                    new_min_readings = 5
                if self.min_readings_for_plateau != new_min_readings:
                    self.min_readings_for_plateau = new_min_readings
                    self.recent_speeds = collections.deque(
                        maxlen=self.min_readings_for_plateau
                    )
                    self.logger.info(
                        f"Re-initialized recent_speeds deque with maxlen {self.min_readings_for_plateau}"
                    )
            else:  # handle invalid control_rate during update
                self.logger.warning(
                    f"Invalid control_rate {self.control_rate} during update, keeping old deque settings."
                )

        self.sim_time = latest_sensor_data.get(
            "simtime", self.sim_time
        )  # Use last known if not present

        # Warmup
        if self.sim_time < 10:
            return {
                "throttle_target": 0,
                "brake_target": 0,
                "time": 0,
            }

        command_reach_time = 0  # Apply immediately

        if self.initial_run:  # Should have been called by __init__ or external reset
            self.reset()
            # Phase is 'starting_level' after reset by default

        # --- State Machine ---
        if self.phase == "idle":
            # self.logger.debug("Phase: idle")
            return {
                "throttle_target": 0,
                "brake_target": 0,
                "time": command_reach_time,
            }

        elif self.phase == "starting_level":
            self.logger.debug("Phase: starting_level")
            if self.current_throttle_index >= len(self.throttle_levels):
                self.logger.info("Throttle sweep fully completed for all levels.")
                self.phase = "idle"
                return {
                    "throttle_target": 0,
                    "brake_target": 0,
                    "time": command_reach_time,
                }

            current_throttle = self.throttle_levels[self.current_throttle_index]
            self.logger.info(f"Starting level for throttle: {current_throttle:.1f}")
            self.recent_speeds.clear()
            self.plateau_check_sim_time_start = 0.0  # Reset for new acceleration phase
            self.phase = "accelerating"
            return {
                "throttle_target": current_throttle,
                "brake_target": 0,
                "time": command_reach_time,
            }

        if self.phase == "accelerating":
            t = self.sim_time
            speed = latest_sensor_data.get("speed", 0.0)
            self.recent_speeds.append(speed)
            rpm = latest_sensor_data.get("engine", {}).get("rpm", 0.0)
            current_throttle = self.throttle_levels[self.current_throttle_index]

            # 1) Mark that we’ve exceeded the limiter
            if (
                not self.rpm_limiter_exceeded
                and rpm >= self.rpm_limiter_threshold
                and speed > self.rpm_limiter_min_speed
            ):
                self.rpm_limiter_exceeded = True

            # 2) On falling-edge (below threshold-hysteresis) *and* we had exceeded:
            elif (
                self.rpm_limiter_exceeded
                and rpm < (self.rpm_limiter_threshold - self.rpm_limiter_hysteresis)
                and (t - self.rpm_limiter_last_hit_time)
                >= self.rpm_limiter_min_interval
            ):
                self.rpm_limiter_hit_count += 1
                self.rpm_limiter_last_hit_time = t
                self.rpm_limiter_exceeded = False
                self.logger.info(
                    f"Detected RPM‐limiter event #{self.rpm_limiter_hit_count} "
                    f"at throttle {current_throttle:.2f} (rpm={rpm:.0f}, t={t:.2f})"
                )

            # 3) If we've now counted two hits, move to coasting
            if self.rpm_limiter_hit_count >= self.rpm_hit_required:
                self.logger.info(
                    f"Reached {self.rpm_hit_required} RPM‐limiter events at "
                    f"throttle {current_throttle:.2f}, coasting."
                )
                self.phase = "coasting"
                return {
                    "throttle_target": 0,
                    "brake_target": 0,
                    "time": command_reach_time,
                }

            # 4) Plateau detection
            if len(self.recent_speeds) == self.min_readings_for_plateau:
                if self.plateau_check_sim_time_start == 0.0:  # First time deque is full
                    self.plateau_check_sim_time_start = self.sim_time

                # Check if max_speed_plateau_duration has passed in simtime since we started looking
                if (
                    self.sim_time - self.plateau_check_sim_time_start
                    >= self.max_speed_plateau_duration
                ):
                    speed_diff = max(self.recent_speeds) - min(self.recent_speeds)
                    self.logger.debug(
                        f"Plateau check: speed_diff={speed_diff:.3f} (sim_time elapsed for check: {self.sim_time - self.plateau_check_sim_time_start:.2f}s)"
                    )
                    if speed_diff < self.max_speed_epsilon:
                        rpm = latest_sensor_data.get("engine", {}).get("rpm", 0.0)
                        self.logger.info(
                            f"Max speed plateau reached for throttle {current_throttle:.1f} "
                            f"at speed {speed:.2f} m/s, RPM {rpm:.0f}, sim_time {self.sim_time:.2f}. "
                            f"Speed diff in plateau: {speed_diff:.3f} m/s."
                        )
                        self.phase = "coasting"
                        return {
                            "throttle_target": 0,
                            "steer_target": 0,
                            "brake_target": 0,
                            "time": command_reach_time,
                        }

            # If not enough data or plateau duration not met, or speed diff too high, continue accelerating
            return {
                "throttle_target": current_throttle,
                "brake_target": 0,
                "time": command_reach_time,
            }

        elif self.phase == "coasting":
            # self.logger.debug("Phase: coasting")
            speed = latest_sensor_data.get("speed", 0.0)
            if speed < self.stop_speed_threshold:
                self.stop_readings_count += 1
            else:
                self.stop_readings_count = 0

            if self.stop_readings_count >= self.stop_duration_consecutive_readings:
                self.logger.info(
                    f"Car stopped at sim_time {self.sim_time:.2f} (speed: {speed:.2f} m/s)."
                )
                self.current_throttle_index += 1
                self.rpm_limiter_hit_count = 0
                self.phase = "starting_level"
                self.stop_readings_count = 0  # Reset for next stop detection

            return {
                "throttle_target": 0,
                "brake_target": 0,
                "time": command_reach_time,
            }

        else:
            self.logger.error(f"Unknown phase: {self.phase}. Resetting to idle.")
            self.phase = "idle"
            return {
                "throttle_target": 0,
                "brake_target": 0,
                "time": command_reach_time,
            }


# Global instance of the logic class
# The control_rate is a default. high_level_controller.py will pass its actual control_rate
# to the compute_control function, which can update the instance's control_rate.
sweeper_logic = ThrottleSweepLogic(control_rate=0.05)  # Default to 20Hz


def compute_control(latest_sensor_data, control_rate, max_latency):
    """
    Main entry point called by high_level_controller.py.
    latest_sensor_data: dict from LLC (e.g., {'speed': x, 'simtime': y, 'engine': {'rpm': z}})
    control_rate: float, the rate at which this function is called.
    max_latency: float, estimated max round-trip latency to LLC.
    """
    return sweeper_logic.compute_control(latest_sensor_data, control_rate, max_latency)


def reset_sweep_state():
    """Allows external reset of the sweeper logic (e.g., from high_level_controller.py)."""
    sweeper_logic.reset()
