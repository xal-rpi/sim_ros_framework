import os
import csv
import math
import threading
import tkinter as tk
from tkinter import ttk

from rclpy.node import get_logger
from rclpy.logging import LoggingSeverity


class PIDTuneControler:
    """
    Basic controller with multi-test pattern similar to compute_control_multi_test.
    """

    def __init__(self, control_rate):
        self.logger = get_logger(self.__class__.__name__)
        self.control_rate = control_rate

        # Thread safety
        self.lock = threading.Lock()

        # Control targets
        self.sim_time = 0.0
        self.current_speed = 0.0  # velocity.x from sensor data

        # Multi-test pattern settings (matching C function)
        self.base_max_torque = 2000.0  # base_maxT
        self.max_speed = 30.0  # max_speed for torque fade
        self.cycle_duration = 60.0  # 60s repeating cycle

        # Test phases (matching C function logic)
        self.test_phases = {
            "step": {"start": 0.0, "end": 15.0, "name": "Step (70%)"},
            "ramp": {"start": 15.0, "end": 30.0, "name": "Linear Ramp"},
            "sine": {"start": 30.0, "end": 45.0, "name": "Sine (0.2Hz)"},
            "chirp": {"start": 45.0, "end": 60.0, "name": "Chirp (0.1-1.0Hz)"},
        }

        # PID parameters (thread-safe)
        self.pid_params = {
            "throttleP": 1.0,
            "throttleI": 0.2,
            "throttleD": 0.1,
            "brakeP": 1.5,
            "brakeI": 0.1,
            "brakeD": 0.05,
        }

        # Test control
        self.test_active = True
        self.test_start_time = 0.0
        self.step_counter = 0

        # Current test state
        self.current_phase = "step"
        self.available_torque = self.base_max_torque
        self.raw_torque = 0.0
        self.wheel_torque = 0.0
        self.brake_torque = 0.0

        # Logging
        self.log_file_path = os.path.join(os.getcwd(), "multi_test_log.csv")
        self._init_log_file()
        # Logging levels
        self.log_levels = {
            "DEBUG": LoggingSeverity.DEBUG,
            "INFO": LoggingSeverity.INFO,
            "WARNING": LoggingSeverity.WARN,
            "ERROR": LoggingSeverity.ERROR,
            "CRITICAL": LoggingSeverity.FATAL,
        }
        self.current_log_level = "INFO"

        # GUI
        self.gui_thread = None
        self.gui_root = None
        self.start_gui()

        self.logger.info("Multi-Test Controller initialized (60s cycle)")

    def _init_log_file(self):
        """Initialize CSV log file"""
        with open(self.log_file_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "sim_time",
                    "cycle_time",
                    "phase",
                    "current_speed",
                    "available_torque",
                    "raw_torque",
                    "wheel_torque",
                    "brake_torque",
                    "throttleP",
                    "throttleI",
                    "throttleD",
                    "brakeP",
                    "brakeI",
                    "brakeD",
                ]
            )

    def start_gui(self):
        """Start the GUI in a separate thread"""
        self.gui_thread = threading.Thread(target=self._run_gui, daemon=True)
        self.gui_thread.start()

    def _run_gui(self):
        """Run the GUI - executed in separate thread"""
        self.gui_root = tk.Tk()
        self.gui_root.title("Multi-Test Controller (60s Cycle)")
        self.gui_root.geometry("900x700")

        # Create GUI elements
        self._create_gui_elements()

        # Start GUI update loop
        self._update_gui()

        try:
            self.gui_root.mainloop()
        except:
            pass

    def _create_gui_elements(self):
        """Create all GUI elements"""
        # Main frame
        main_frame = ttk.Frame(self.gui_root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Title
        title_label = ttk.Label(
            main_frame,
            text="Multi-Test Controller (60s Cycle)",
            font=("Arial", 16, "bold"),
        )
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))

        # Test Control Frame
        control_frame = ttk.LabelFrame(main_frame, text="Test Control", padding="10")
        control_frame.grid(
            row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10)
        )

        # First row: Test active and logging level
        control_row1 = ttk.Frame(control_frame)
        control_row1.grid(
            row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10)
        )

        # Test active checkbox
        self.active_var = tk.BooleanVar(value=self.test_active)
        active_check = ttk.Checkbutton(
            control_row1,
            text="Test Active",
            variable=self.active_var,
            command=self._on_active_change,
        )
        active_check.grid(row=0, column=0, sticky=tk.W)

        # Logging level control
        ttk.Label(control_row1, text="Log Level:").grid(
            row=0, column=1, sticky=tk.W, padx=(40, 10)
        )
        self.log_level_var = tk.StringVar(value=self.current_log_level)
        log_level_combo = ttk.Combobox(
            control_row1,
            textvariable=self.log_level_var,
            values=list(self.log_levels.keys()),
            state="readonly",
            width=10,
        )
        log_level_combo.grid(row=0, column=2, sticky=tk.W)
        log_level_combo.bind("<<ComboboxSelected>>", self._on_log_level_change)

        # Base torque setting
        ttk.Label(control_frame, text="Base Max Torque:").grid(
            row=1, column=0, sticky=tk.W, pady=2
        )
        self.base_torque_var = tk.DoubleVar(value=self.base_max_torque)
        base_torque_scale = ttk.Scale(
            control_frame,
            from_=1000,
            to=3000,
            variable=self.base_torque_var,
            orient=tk.HORIZONTAL,
            length=200,
            command=self._on_base_torque_change,
        )
        base_torque_scale.grid(row=1, column=1, padx=(10, 0), sticky=(tk.W, tk.E))
        self.base_torque_label = ttk.Label(
            control_frame, text=f"{self.base_max_torque:.0f}"
        )
        self.base_torque_label.grid(row=1, column=2, padx=(10, 0))

        # Max speed setting
        ttk.Label(control_frame, text="Max Speed (fade):").grid(
            row=2, column=0, sticky=tk.W, pady=2
        )
        self.max_speed_var = tk.DoubleVar(value=self.max_speed)
        max_speed_scale = ttk.Scale(
            control_frame,
            from_=20,
            to=50,
            variable=self.max_speed_var,
            orient=tk.HORIZONTAL,
            length=200,
            command=self._on_max_speed_change,
        )
        max_speed_scale.grid(row=2, column=1, padx=(10, 0), sticky=(tk.W, tk.E))
        self.max_speed_label = ttk.Label(control_frame, text=f"{self.max_speed:.1f}")
        self.max_speed_label.grid(row=2, column=2, padx=(10, 0))

        # Test Pattern Info Frame
        pattern_frame = ttk.LabelFrame(
            main_frame, text="Test Pattern (60s Cycle)", padding="10"
        )
        pattern_frame.grid(
            row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10)
        )

        # Phase descriptions
        phase_info = [
            "0-15s: Step to 70% of available torque",
            "15-30s: Linear ramp from 0% to 100% over 15s",
            "30-45s: Low-frequency sine wave (0.2 Hz)",
            "45-60s: Chirp sweep from 0.1Hz to 1.0Hz over 15s",
        ]

        for i, info in enumerate(phase_info):
            label = ttk.Label(pattern_frame, text=info, font=("Courier", 10))
            label.grid(row=i, column=0, sticky=tk.W, pady=2)

        # PID Parameters Frame
        pid_frame = ttk.LabelFrame(main_frame, text="PID Parameters", padding="10")
        pid_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))

        # Throttle PID
        throttle_frame = ttk.LabelFrame(pid_frame, text="Throttle PID", padding="5")
        throttle_frame.grid(
            row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10)
        )

        self._create_pid_sliders(throttle_frame, "throttle")

        # Brake PID
        brake_frame = ttk.LabelFrame(pid_frame, text="Brake PID", padding="5")
        brake_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))

        self._create_pid_sliders(brake_frame, "brake")

        # Status Frame
        status_frame = ttk.LabelFrame(main_frame, text="Status", padding="10")
        status_frame.grid(
            row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10)
        )

        self.status_labels = {}
        status_items = [
            ("Sim Time", "0.0s"),
            ("Cycle Time", "0.0s"),
            ("Current Phase", "step"),
            ("Current Speed", "0.0 m/s"),
            ("Available Torque", "0.0 Nm"),
            ("Raw Torque", "0.0 Nm"),
            ("Wheel Torque", "0.0 Nm"),
            ("Brake Torque", "0.0 Nm"),
        ]

        for i, (label, default) in enumerate(status_items):
            row, col = i // 2, (i % 2) * 2
            ttk.Label(status_frame, text=f"{label}:").grid(
                row=row, column=col, sticky=tk.W, padx=(0, 10), pady=2
            )
            self.status_labels[label] = ttk.Label(
                status_frame, text=default, font=("Courier", 10)
            )
            self.status_labels[label].grid(
                row=row, column=col + 1, sticky=tk.W, padx=(0, 20), pady=2
            )

        # Configure grid weights
        main_frame.columnconfigure(1, weight=1)
        control_frame.columnconfigure(1, weight=1)
        control_row1.columnconfigure(3, weight=1)
        pid_frame.columnconfigure(0, weight=1)
        pid_frame.columnconfigure(1, weight=1)

    def _create_pid_sliders(self, parent, prefix):
        """Create PID parameter sliders"""
        params = ["P", "I", "D"]
        ranges = {"P": (0.1, 5.0), "I": (0.0, 2.0), "D": (0.0, 1.0)}

        for i, param in enumerate(params):
            param_key = f"{prefix}{param}"

            ttk.Label(parent, text=f"{param}:").grid(
                row=i, column=0, sticky=tk.W, pady=2
            )

            var = tk.DoubleVar(value=self.pid_params[param_key])
            scale = ttk.Scale(
                parent,
                from_=ranges[param][0],
                to=ranges[param][1],
                variable=var,
                orient=tk.HORIZONTAL,
                length=200,
                command=lambda v, key=param_key: self._on_pid_change(key, v),
            )
            scale.grid(row=i, column=1, padx=(10, 10), sticky=(tk.W, tk.E))

            label = ttk.Label(
                parent, text=f"{self.pid_params[param_key]:.3f}", font=("Courier", 10)
            )
            label.grid(row=i, column=2, padx=(0, 10))

            # Store references for updates
            setattr(self, f"{param_key}_var", var)
            setattr(self, f"{param_key}_label", label)

    def _on_active_change(self):
        """Handle test active change"""
        with self.lock:
            self.test_active = self.active_var.get()

    def _on_base_torque_change(self, value):
        """Handle base torque change"""
        with self.lock:
            self.base_max_torque = float(value)

    def _on_max_speed_change(self, value):
        """Handle max speed change"""
        with self.lock:
            self.max_speed = float(value)

    def _on_pid_change(self, param_key, value):
        """Handle PID parameter change"""
        with self.lock:
            self.pid_params[param_key] = float(value)
            self.logger.debug(f"Updated {param_key} to {value}")

    def _on_log_level_change(self, event=None):
        """Handle logging level change"""
        new_level = self.log_level_var.get()
        with self.lock:
            self.current_log_level = new_level

        # Set the logging level for both the controller logger and root logger
        level = self.log_levels[new_level]
        self.logger.set_level(level)

        # Log the change at the new level
        self.logger.info(f"Logging level changed to: {new_level}")

        # Demonstrate the new level with test messages
        if new_level == "DEBUG":
            self.logger.debug(
                "Debug logging is now enabled - you'll see detailed information"
            )
        elif new_level == "WARNING":
            self.logger.warning(
                "Warning level - only warnings and errors will be shown"
            )
        elif new_level == "ERROR":
            self.logger.error(
                "Error level - only errors and critical messages will be shown"
            )

    def _update_gui(self):
        """Update GUI elements periodically"""
        if not self.gui_root:
            return

        try:
            # Update parameter labels
            for param_key in self.pid_params:
                label = getattr(self, f"{param_key}_label", None)
                if label:
                    label.config(text=f"{self.pid_params[param_key]:.3f}")

            # Update control parameter labels
            self.base_torque_label.config(text=f"{self.base_max_torque:.0f}")
            self.max_speed_label.config(text=f"{self.max_speed:.1f}")

            # Update status
            if hasattr(self, "status_labels"):
                cycle_time = self.sim_time % self.cycle_duration

                self.status_labels["Sim Time"].config(text=f"{self.sim_time:.1f}s")
                self.status_labels["Cycle Time"].config(text=f"{cycle_time:.1f}s")
                self.status_labels["Current Phase"].config(text=self.current_phase)
                self.status_labels["Current Speed"].config(
                    text=f"{self.current_speed:.1f} m/s"
                )
                self.status_labels["Available Torque"].config(
                    text=f"{self.available_torque:.0f} Nm"
                )
                self.status_labels["Raw Torque"].config(
                    text=f"{self.raw_torque:.0f} Nm"
                )
                self.status_labels["Wheel Torque"].config(
                    text=f"{self.wheel_torque:.0f} Nm"
                )
                self.status_labels["Brake Torque"].config(
                    text=f"{self.brake_torque:.0f} Nm"
                )

            # Schedule next update
            self.gui_root.after(100, self._update_gui)

        except:
            pass

    def _calculate_multi_test_pattern(self):
        """Calculate multi-test pattern exactly like C function"""
        if not self.test_active:
            return 0.0, 0.0, 0.0, "inactive"

        # Fade max torque with speed (exactly like C function)
        frac = 1.0 - (self.current_speed / self.max_speed)
        if frac < 0.0:
            frac = 0.0
        self.available_torque = self.base_max_torque * frac

        # Build 60s repeating test waveform
        cycle = self.sim_time % self.cycle_duration
        PI = math.pi

        if cycle < 15.0:
            # Step to 70% of available torque
            raw_torque = 0.7 * self.available_torque
            current_phase = "step"

        elif cycle < 30.0:
            # Linear ramp from 0 → +100% over 15s
            t = (cycle - 15.0) / 15.0
            raw_torque = t * self.available_torque
            current_phase = "ramp"

        elif cycle < 45.0:
            # Low-frequency sine (0.2 Hz)
            raw_torque = self.available_torque * math.sin(
                2.0 * PI * 0.2 * (cycle - 30.0)
            )
            current_phase = "sine"

        else:
            # Chirp: 0.1→1.0 Hz sweep over 15s
            tc = cycle - 45.0
            Tdur = 15.0
            f0, f1 = 0.1, 1.0
            k = (f1 - f0) / Tdur
            phase = 2.0 * PI * (f0 * tc + 0.5 * k * tc * tc)
            raw_torque = self.available_torque * math.sin(phase)
            current_phase = "chirp"

        # Clamp into ±available_torque
        if raw_torque > self.available_torque:
            raw_torque = self.available_torque
        if raw_torque < -self.available_torque:
            raw_torque = -self.available_torque

        # Split into drive vs. brake (exactly like C function)
        wheel_torque = raw_torque if raw_torque > 0.0 else 0.0
        brake_torque = -raw_torque if raw_torque < 0.0 else 0.0

        # Store for status display
        self.raw_torque = raw_torque
        self.wheel_torque = wheel_torque
        self.brake_torque = brake_torque
        self.current_phase = current_phase

        return wheel_torque, brake_torque, raw_torque, current_phase

    def compute_control(self, latest_sensor_data, control_rate_val, max_latency_val):
        """Main control computation - returns flat dictionary"""
        # Extract simtime exactly as C function
        self.sim_time = latest_sensor_data.get("simtime", self.sim_time)

        # Extract velocity.x exactly as C function
        vel_dict = latest_sensor_data.get("velocity", {})
        if isinstance(vel_dict, dict):
            self.current_speed = vel_dict.get("x", 0.0)
        else:
            self.current_speed = 0.0

        # Thread-safe access to parameters
        with self.lock:
            current_pid = self.pid_params.copy()
            test_active = self.test_active
            base_max_torque = self.base_max_torque
            max_speed = self.max_speed

        # Calculate multi-test pattern
        wheel_torque, brake_torque, raw_torque, current_phase = (
            self._calculate_multi_test_pattern()
        )

        # Road wheel angle (straight ahead like C function)
        road_wheel_angle = 0.0

        # Compute & clamp latency exactly like C function
        latency = max_latency_val + 0.005
        if latency > 0.1:
            latency = 0.1
        time_val = self.sim_time + control_rate_val + latency

        # Log data
        if self.step_counter % 40 == 0:  # Log every 0.2 seconds (200Hz / 40 = 5Hz)
            cycle_time = self.sim_time % self.cycle_duration
            with open(self.log_file_path, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        self.sim_time,
                        cycle_time,
                        current_phase,
                        self.current_speed,
                        self.available_torque,
                        raw_torque,
                        wheel_torque,
                        brake_torque,
                        current_pid["throttleP"],
                        current_pid["throttleI"],
                        current_pid["throttleD"],
                        current_pid["brakeP"],
                        current_pid["brakeI"],
                        current_pid["brakeD"],
                    ]
                )

        self.step_counter += 1

        # Progress logging
        if self.step_counter % 200 == 0:  # Every second
            cycle_time = self.sim_time % self.cycle_duration
            self.logger.info(
                f"Phase: {current_phase} ({cycle_time:.1f}s), Speed: {self.current_speed:.1f} m/s, "
                f"Available: {self.available_torque:.0f} Nm, Wheel: {wheel_torque:.0f} Nm, Brake: {brake_torque:.0f} Nm"
            )

        # Return flat dictionary directly - no nesting, no flattening needed
        return {
            "wheel_torque": wheel_torque,
            "brake_torque": brake_torque,
            "road_wheel_angle": road_wheel_angle,
            "time": time_val,
            "throttleP": current_pid["throttleP"],
            "throttleI": current_pid["throttleI"],
            "throttleD": current_pid["throttleD"],
            "brakeP": current_pid["brakeP"],
            "brakeI": current_pid["brakeI"],
            "brakeD": current_pid["brakeD"],
        }

    def reset(self):
        """Reset controller state"""
        with self.lock:
            self.sim_time = 0.0
            self.test_start_time = 0.0
            self.step_counter = 0
            self.current_speed = 0.0
            self.available_torque = self.base_max_torque
            self.raw_torque = 0.0
            self.wheel_torque = 0.0
            self.brake_torque = 0.0
            self.current_phase = "step"

        # Reinitialize log file
        self._init_log_file()
        self.logger.info("Multi-Test Controller reset")

    def shutdown(self):
        """Shutdown controller and GUI"""
        if self.gui_root:
            try:
                self.gui_root.quit()
                self.gui_root.destroy()
            except:
                pass


# Global instance and wrappers
test_controller = PIDTuneControler(control_rate=0.005)  # 200Hz = 0.005s


def compute_control(latest_sensor_data, control_rate, max_latency):
    return test_controller.compute_control(
        latest_sensor_data, control_rate, max_latency
    )


def reset_analyzer_state():
    test_controller.reset()


def shutdown_controller():
    test_controller.shutdown()
