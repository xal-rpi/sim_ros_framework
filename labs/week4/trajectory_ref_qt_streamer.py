"""trajectory_ref_qt_streamer.py

Qt live streamer for the path-following lab.

What this is
------------
A local GUI (Qt + pyqtgraph) that:
- subscribes to /<vehicle>/reduced_state (ReducedGtStateMsg)
- loads a CSV reference (ReferenceTrajectory)
- keeps track of progress s_closest and lookahead target (x_ref/y_ref/yaw_ref)
- live-plots key signals vs time or vs s (user selectable)
- optionally instantiates TorqueSpeedController to send UDP commands to BeamNG

What this is NOT
----------------
- It does *not* publish reference/target topics on ROS 2.
  (ROS 2 is only used for subscribing to reduced_state and (optionally) for
   service discovery used by TorqueSpeedController.)

Dependencies
------------
- PySide6
- pyqtgraph
- rclpy
- numpy

Run
---
    python3 labs/week4/trajectory_ref_qt_streamer.py --vehicle EGO --csv /tmp/ref.csv --Ld 8.0

"""

from __future__ import annotations

import argparse
import sys
import time
import threading
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, List, Sequence

import numpy as np

try:
    from PySide6 import QtCore, QtWidgets
except Exception as e:  # pragma: no cover
    raise RuntimeError(
        "PySide6 is required for the Qt streamer. Install with: pip install PySide6"
    ) from e

try:
    import pyqtgraph as pg
except Exception as e:  # pragma: no cover
    raise RuntimeError(
        "pyqtgraph is required for plotting. Install with: pip install pyqtgraph"
    ) from e

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

try:
    from bng_msgs.msg import ReducedGtStateMsg
except Exception as e:  # pragma: no cover
    raise RuntimeError(
        "ReducedGtStateMsg is not importable; did you source the workspace / build bng_msgs?"
    ) from e

from reference_trajectory import ReferenceTrajectory
from bng_controller.torque_speed_controller import TorqueSpeedController


def _finite_or_nan(x: float) -> float:
    """Best-effort float conversion that returns NaN if invalid."""
    try:
        v = float(x)
        return v if np.isfinite(v) else float("nan")
    except Exception:
        return float("nan")


@dataclass
class StreamSample:
    """One time-step of streamed state + reference info for plotting."""
    t: float
    s: float
    x: float
    y: float
    yaw: float
    V: float
    vx: float
    vy: float
    r: float
    delta: float
    s_target: float
    x_proj: float
    y_proj: float
    proj_dist: float
    x_target: float
    y_target: float
    yaw_target: float
    alpha: float
    delta_target: float
    V_target: float
    brake_cmd: float


class TrajectoryStreamNode(Node):
    """ROS 2 node that buffers reduced_state samples and (optionally) owns the GUI."""
    def __init__(
        self,
        *,
        vehicle_name: str,
        csv_path: str,
        Ld: float,
        steering_scale: float,
        enable_display: bool,
        plot_groups: Sequence[Sequence[str]],
        closed: bool = False,
        max_len: int = 2000,
    ):
        """Initialize the trajectory streamer node.

        Parameters
        ----------
        vehicle_name : str
            Vehicle namespace for the reduced_state topic ("/<vehicle>/reduced_state").
        csv_path : str
            Path to reference CSV used to build ReferenceTrajectory.
        Ld : float
            Lookahead distance used to compute the target reference point.
        closed : bool
            Whether the reference path is closed (wrap s).
        max_len : int
            Max number of samples to keep in the internal buffer.
        steering_scale : float
            Scale factor for converting steering angle to sim steering input.
        enable_display : bool
            If True, construct a QApplication/MainWindow owned by this node.
        plot_groups : Sequence[Sequence[str]]
            Plot groups to visualize; each sub-sequence is a plot of StreamSample fields.

        Example
        -------
        >>> plot_groups = [
        ...     ("V", "vx", "vy"),
        ...     ("delta", "delta_target"),
        ...     ("V_target", "brake_cmd"),
        ...     ("r", "alpha", "proj_dist"),
        ...     ("x", "y"),
        ... ]
        >>> node = TrajectoryStreamNode(
        ...     vehicle_name="EGO",
        ...     csv_path="/tmp/ref.csv",
        ...     Ld=8.0,
        ...     closed=False,
        ...     max_len=2000,
        ...     steering_scale=1.0,
        ...     enable_display=True,
        ...     plot_groups=plot_groups,
        ... )
        ... rc = node.run_display()
        """
        super().__init__(f"traj_ref_qt_stream_{vehicle_name}")

        self.vehicle_name = str(vehicle_name)
        self.ref = ReferenceTrajectory(csv_path, closed=bool(closed))

        self.Ld = float(Ld)
        if self.Ld <= 0.0:
            raise ValueError("Ld must be > 0")

        self._state_topic = f"/{self.vehicle_name}/reduced_state"
        self._sub = self.create_subscription(ReducedGtStateMsg, self._state_topic, self._on_state, 10)

        # Progress tracking
        self._last_s: Optional[float] = None
        self._s_wrap_offset = 0.0

        self._closed = bool(closed)
        self._ref_length = float(self.ref.length)

        self._steering_scale = float(steering_scale)
        if self._steering_scale == 0.0:
            raise ValueError("steering_scale must be non-zero")

        try:
            self._torque_ctl = TorqueSpeedController(
                vehicle_name=self.vehicle_name,
                subscribe_reduced_state=False,
                publish_commands=False,
            )
        except Exception as e:
            print("Warning: could not instantiate TorqueSpeedController; "
                  "UDP speed commands will not be sent.")
            self._torque_ctl = None  # type: ignore

        # Command values (set by GUI; stored on each sample)
        self._steer_cmd = 0.0
        self._speed_cmd = 0.0
        self._brake_cmd = 0.0

        # Buffer sizing
        maxlen = int(max_len)
        if maxlen <= 0:
            maxlen = 2000

        self.samples: Deque[StreamSample] = deque(maxlen=maxlen)
        self._latest_sample: Optional[StreamSample] = None

        self._display_enabled = bool(enable_display)
        self._plot_groups = [tuple(group) for group in plot_groups]
        self._app: Optional[QtWidgets.QApplication] = None
        self._window: Optional[MainWindow] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._app_thread: Optional[threading.Thread] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_running = False

        self.get_logger().info(
            f"Qt streamer subscribed to {self._state_topic}; ref_len={self.ref.length:.1f}m, maxlen={maxlen}"
        )

    def set_cmd(self, *, delta_target: float, V_target: float, brake_cmd: float) -> None:
        """Store the current control targets for logging/plotting.

        Parameters
        ----------
        delta_target : float
            Steering target (rad).
        V_target : float
            Speed target (m/s).
        brake_cmd : float
            Brake command [0..1].
        """
        self._steer_cmd = float(delta_target)
        self._speed_cmd = float(V_target)
        self._brake_cmd = float(brake_cmd)

    def get_state_on_ref_path(self) -> Optional[StreamSample]:
        """Return the latest state projected onto the reference path.

        Returns
        -------
        StreamSample | None
            The most recent sample, or None if no reduced_state messages have
            been received yet.

        Field meanings
        --------------
        t : float
            Simulation time from ReducedGtStateMsg (seconds).
        s : float
            Arc-length progress of the closest point on the reference (meters).
        x, y, yaw : float
            Measured vehicle pose in the map frame.
        V : float
            Measured speed magnitude (m/s).
        vx, vy : float
            Measured velocity components in the map frame (m/s).
        r : float
            Measured yaw rate (rad/s).
        delta : float
            Measured steering angle (rad) from ReducedGtStateMsg.
        s_target : float
            Lookahead arc-length = s + Ld (meters).
        x_proj, y_proj : float
            Closest point on the reference polyline.
        proj_dist : float
            Euclidean distance from (x, y) to (x_proj, y_proj).
        x_target, y_target, yaw_target : float
            Reference pose at s_target.
        alpha : float
            Heading error from current yaw to the lookahead target.
        delta_target : float
            Most recent steering target commanded by the GUI/controller.
        V_target : float
            Most recent speed target commanded by the GUI/controller.
        brake_cmd : float
            Most recent brake command [0..1].
        """
        return self._latest_sample

    def run_display(self, *, blocking: bool = True) -> int:
        """Start the Qt event loop.

        Parameters
        ----------
        blocking : bool, optional
            If True, block on QApplication.exec(). If False, show the window
            and return immediately; internal timers (~100 Hz) keep ROS/Qt responsive.

        Returns
        -------
        int
            Qt application exit code (0 if display is disabled or non-blocking).
        """
        if not self._display_enabled:
            return 0
        if blocking:
            self._init_display()
            self._start_spin_thread()
            return int(self._app.exec())
        if self._app_thread is None or not self._app_thread.is_alive():
            self._app_thread = threading.Thread(target=self._run_app_exec, daemon=True)
            self._app_thread.start()
        return 0

    def close_display(self) -> None:
        """Close the Qt window and shut down the Qt application."""
        self._stop_spin_thread()
        if self._executor is not None:
            try:
                self._executor.remove_node(self)
            except Exception:
                pass
            self._executor = None
        if self._window is not None:
            try:
                self._window.close()
            except Exception:
                pass
            self._window = None
        if self._app is not None:
            try:
                self._app.quit()
            except Exception:
                pass
            self._app = None
        if self._app_thread is not None and self._app_thread.is_alive():
            self._app_thread.join(timeout=1.0)
        self._app_thread = None

    def destroy_node(self) -> bool:
        """Ensure GUI closes before ROS node teardown.

        Returns
        -------
        bool
            True if destruction succeeded.
        """
        self.close_display()
        return super().destroy_node()

    def send_vehicle_speed(self, *, V_target: float, delta_target: float, brake_cmd: float = 0.0) -> None:
        """Convenience wrapper for sending speed + steering and logging targets.

        Parameters
        ----------
        V_target : float
            Speed target (m/s).
        delta_target : float
            Steering target (rad).
        brake_cmd : float, optional
            Brake command [0..1], by default 0.0.
        """
        if self._torque_ctl is None:
            return
        steer_input = float(delta_target) / self._steering_scale
        self.set_cmd(delta_target=float(delta_target), V_target=float(V_target), brake_cmd=float(brake_cmd))
        self._torque_ctl.send_command(
            vehicle_speed = float(V_target), 
            steering=steer_input, brake=float(brake_cmd)
        )

    def _run_app_exec(self) -> None:
        """Run the Qt event loop in a background thread (non-blocking mode)."""
        self._init_display()
        self._start_spin_thread()
        if self._app is None:
            return
        try:
            self._app.exec()
        except Exception:
            pass
        self._stop_spin_thread()

    def _init_display(self) -> None:
        """Create QApplication, window, and timers if needed."""
        if self._app is not None or not self._display_enabled:
            return
        self._app = QtWidgets.QApplication([])
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._window = MainWindow(
            stream_node=self,
            vehicle_name=self.vehicle_name,
            plot_groups=self._plot_groups,
        )
        self._window.resize(1100, 850)
        self._window.show()

    def _spin_loop(self) -> None:
        """Background loop to spin ROS while Qt runs on the main thread."""
        rate_hz = 100.0
        period = 1.0 / rate_hz
        while self._spin_running:
            if not rclpy.ok():
                break
            if self._executor is not None:
                try:
                    self._executor.spin_once(timeout_sec=0.0)
                except Exception:
                    break
            time.sleep(period)

    def _start_spin_thread(self) -> None:
        if self._spin_running:
            return
        self._spin_running = True
        if self._spin_thread is None or not self._spin_thread.is_alive():
            self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self._spin_thread.start()

    def _stop_spin_thread(self) -> None:
        if not self._spin_running:
            return
        self._spin_running = False
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self._spin_thread = None

    def _unwrap_s_progress(self, s_closest: float) -> float:
        """Unwrap s for closed tracks to keep progress monotonic for plotting.

        Parameters
        ----------
        s_closest : float
            Closest-point arc-length on the reference (meters).

        Returns
        -------
        float
            Unwrapped arc-length for plotting.
        """
        # For open tracks, do nothing.
        if not self._closed:
            return float(s_closest)

        if self._last_s is None or not np.isfinite(self._last_s):
            return float(s_closest)

        prev = float(self._last_s)
        cur = float(s_closest)
        L = self._ref_length
        if L <= 0.0:
            return cur

        # If s jumps backward by more than half a lap, we likely wrapped.
        if cur < prev - 0.5 * L:
            self._s_wrap_offset += L
        # If s jumps forward by more than half a lap, we likely wrapped backward (rare).
        elif cur > prev + 0.5 * L:
            self._s_wrap_offset -= L

        return cur + self._s_wrap_offset

    def _on_state(self, msg: ReducedGtStateMsg) -> None:
        """Handle reduced_state updates and append a StreamSample.

        Uses sim time only (msg.time). If time is not finite, the sample is skipped.
        """
        # Time base (sim time only)
        t_sim = _finite_or_nan(getattr(msg, "time", float("nan")))
        if not np.isfinite(t_sim):
            return
        t = float(t_sim)

        x = _finite_or_nan(getattr(msg, "x", float("nan")))
        y = _finite_or_nan(getattr(msg, "y", float("nan")))
        yaw = _finite_or_nan(getattr(msg, "yaw", float("nan")))
        vel_mag = _finite_or_nan(getattr(msg, "vel_mag", float("nan")))
        vx = _finite_or_nan(getattr(msg, "vx", float("nan")))
        vy = _finite_or_nan(getattr(msg, "vy", float("nan")))
        r = _finite_or_nan(getattr(msg, "r", float("nan")))
        delta = _finite_or_nan(getattr(msg, "delta", float("nan")))

        if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(yaw)):
            return

        refd = self.ref.lookahead_from_pose(
            x=float(x),
            y=float(y),
            yaw=float(yaw),
            Ld=float(self.Ld),
            last_s=self._last_s,
            s_back=10.0,
            s_fwd=40.0,
        )

        s_closest = float(refd["s_closest"])
        s_plot = self._unwrap_s_progress(s_closest)

        # Update last_s for windowing (keep the wrapped/original s for the query)
        self._last_s = s_closest

        sample = StreamSample(
            t=t,
            s=s_plot,
            x=float(x),
            y=float(y),
            yaw=float(yaw),
            V=float(vel_mag),
            vx=float(vx),
            vy=float(vy),
            r=float(r),
            delta=float(delta),
            s_target=float(refd["s_target"]) + (self._s_wrap_offset if self._closed else 0.0),
            x_proj=float(refd["x_proj"]),
            y_proj=float(refd["y_proj"]),
            proj_dist=float(refd["proj_dist"]),
            x_target=float(refd["x_target"]),
            y_target=float(refd["y_target"]),
            yaw_target=float(refd["yaw_target"]),
            alpha=float(refd["alpha"]),
            delta_target=float(self._steer_cmd),
            V_target=float(self._speed_cmd),
            brake_cmd=float(self._brake_cmd),
        )

        self.samples.append(sample)
        self._latest_sample = sample




##########################################################################
####################### Mainly Visualization Below #######################
###########################################################################
class MainWindow(QtWidgets.QMainWindow):
    """Qt window that visualizes StreamSample data from a TrajectoryStreamNode."""
    def __init__(
        self,
        *,
        stream_node: TrajectoryStreamNode,
        vehicle_name: str,
        plot_groups: Sequence[Sequence[str]],
    ):
        super().__init__()

        self.stream_node = stream_node
        self.vehicle_name = str(vehicle_name)

        self._paused = False
        self._plot_groups = [tuple(group) for group in plot_groups]

        self.setWindowTitle(f"Trajectory Ref Live Stream ({self.vehicle_name})")

        # ------------
        # Controls
        # ------------
        controls = QtWidgets.QWidget()
        controls_layout = QtWidgets.QGridLayout(controls)

        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["time", "s"])
        controls_layout.addWidget(QtWidgets.QLabel("X axis"), 0, 0)
        controls_layout.addWidget(self.mode_combo, 0, 1)

        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.clear_btn = QtWidgets.QPushButton("Clear")
        controls_layout.addWidget(self.pause_btn, 0, 2)
        controls_layout.addWidget(self.clear_btn, 0, 3)

        self.history_spin = QtWidgets.QDoubleSpinBox()
        self.history_spin.setRange(1.0, 300.0)
        self.history_spin.setDecimals(1)
        self.history_spin.setSingleStep(1.0)
        self.history_spin.setValue(20.0)
        controls_layout.addWidget(QtWidgets.QLabel("History (s)"), 1, 0)
        controls_layout.addWidget(self.history_spin, 1, 1)

        self.max_points_spin = QtWidgets.QSpinBox()
        self.max_points_spin.setRange(50, 50000)
        self.max_points_spin.setSingleStep(100)
        self.max_points_spin.setValue(2000)
        controls_layout.addWidget(QtWidgets.QLabel("Max points"), 1, 2)
        controls_layout.addWidget(self.max_points_spin, 1, 3)

        # ------------
        # Plots
        # ------------
        pg.setConfigOptions(antialias=True)

        self.plot_widgets: List[pg.PlotWidget] = []
        self.plot_curves: List[List[pg.PlotDataItem]] = []
        self.plot_refs: List[List[Optional[pg.PlotDataItem]]] = []
        color_cycle = ["y", "c", "m", "g", "w", "r"]

        for group in self._plot_groups:
            title = ", ".join(group)
            plot = pg.PlotWidget(title=title)
            plot.addLegend()
            curves: List[pg.PlotDataItem] = []
            refs: List[pg.PlotDataItem] = []
            for idx, field in enumerate(group):
                pen = pg.mkPen(color_cycle[idx % len(color_cycle)], width=2)
                curve = plot.plot(pen=pen, name=field)
                curve.setClipToView(True)
                curve.setDownsampling(auto=True, method="peak")
                curves.append(curve)

                refs.append(None)

            self.plot_widgets.append(plot)
            self.plot_curves.append(curves)
            self.plot_refs.append(refs)

        self.plot_xy = pg.PlotWidget(title="XY")
        self.plot_xy.setAspectLocked(True, 1)
        self.curve_path = self.plot_xy.plot(
            self.stream_node.ref.x_ref,
            self.stream_node.ref.y_ref,
            pen=pg.mkPen((120, 120, 120), width=2),
            name="ref_path",
        )
        self.curve_xy_meas = self.plot_xy.plot(
            [],
            [],
            pen=pg.mkPen((0, 180, 255), width=2),
            name="measured_path",
        )
        self.scatter_vehicle = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 180, 255), pen=pg.mkPen(None))
        self.scatter_target = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(255, 120, 0), pen=pg.mkPen(None))
        self.scatter_proj = pg.ScatterPlotItem(size=8, brush=pg.mkBrush(0, 255, 0), pen=pg.mkPen(None))
        self.plot_xy.addItem(self.scatter_vehicle)
        self.plot_xy.addItem(self.scatter_target)
        self.plot_xy.addItem(self.scatter_proj)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        layout.addWidget(controls)

        split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        split.addWidget(self.plot_xy)
        for plot in self.plot_widgets:
            split.addWidget(plot)
        split.setSizes([400] + [220] * max(1, len(self.plot_widgets)))
        layout.addWidget(split)

        self.setCentralWidget(central)

        # ------------
        # Timers
        # ------------
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_timer.start(50)  # 20 Hz plotting

        # ------------
        # Signals
        # ------------
        self.pause_btn.clicked.connect(self._toggle_pause)
        self.clear_btn.clicked.connect(self._clear)


    def closeEvent(self, event):  # noqa: N802
        try:
            self.plot_timer.stop()
        except Exception:
            pass
        event.accept()

    def _toggle_pause(self) -> None:
        self._paused = not self._paused
        self.pause_btn.setText("Resume" if self._paused else "Pause")

    def _clear(self) -> None:
        self.stream_node.samples.clear()

    def _x_axis(self) -> str:
        return str(self.mode_combo.currentText()).strip().lower()

    def _update_plots(self) -> None:
        """Refresh plots from the node's sample buffer."""
        if not self.stream_node.samples:
            return

        samples = list(self.stream_node.samples)
        if not samples:
            return

        self.curve_xy_meas.setData([s.x for s in samples], [s.y for s in samples])

        axis = self._x_axis()
        if axis == "time":
            t_end = samples[-1].t
            t_start = t_end - float(self.history_spin.value())
            samples = [s for s in samples if s.t >= t_start]
        else:
            s_end = samples[-1].s
            s_start = s_end - float(self.history_spin.value())
            samples = [s for s in samples if s.s >= s_start]

        if not samples:
            return

        max_points = int(self.max_points_spin.value())
        if len(samples) > max_points:
            stride = max(1, len(samples) // max_points)
            samples = samples[::stride]

        xs = np.fromiter((s.t if axis == "time" else s.s for s in samples), dtype=float, count=len(samples))
        for group, curves, refs in zip(self._plot_groups, self.plot_curves, self.plot_refs):
            for field, curve, ref_curve in zip(group, curves, refs):
                ys = np.fromiter((getattr(s, field) for s in samples), dtype=float)
                curve.setData(xs, ys)

        last = self.stream_node.samples[-1]
        self.scatter_vehicle.setData([last.x], [last.y])
        self.scatter_target.setData([last.x_target], [last.y_target])
        self.scatter_proj.setData([last.x_proj], [last.y_proj])

        xlabel = "t (s)" if axis == "time" else "s (m)"
        for plot in self.plot_widgets:
            plot.setLabel("bottom", xlabel)

        # Status bar: quick live readout
        self.statusBar().showMessage(
            f"t={last.t:6.2f}s | s={last.s:8.2f}m (target {last.s_target:8.2f}) | "
            f"pos=({last.x:7.2f},{last.y:7.2f}) yaw={last.yaw:6.2f} | "
            f"alpha={last.alpha:6.3f} proj={last.proj_dist:5.2f} | "
            f"V={last.V:5.2f} delta={last.delta:6.3f}"
        )


# def _parse_args(argv) -> argparse.Namespace:
#     p = argparse.ArgumentParser(description="Qt live streamer for reference trajectory + reduced_state")
#     p.add_argument("--vehicle", default="EGO", help="Vehicle name (topic prefix).")
#     p.add_argument("--csv", required=True, help="Path to reference CSV with x,y (and optional s,yaw).")
#     p.add_argument("--Ld", type=float, default=8.0, help="Lookahead distance in meters.")
#     p.add_argument("--closed", action="store_true", help="Treat reference as a closed loop (wrap s).")
#     p.add_argument(
#         "--max-len",
#         type=int,
#         default=2000,
#         help="Max number of samples stored in the stream buffer.",
#     )
#     p.add_argument(
#         "--steering-scale",
#         type=float,
#         default=1.0,
#         help="Conversion from delta (rad) to steering input [-1..1].",
#     )
#     return p.parse_args(argv)


# def main(argv=None) -> int:
#     args = _parse_args(argv or sys.argv[1:])

#     rclpy.init(args=None)

#     node = TrajectoryStreamNode(
#         vehicle_name=args.vehicle,
#         csv_path=args.csv,
#         Ld=args.Ld,
#         closed=args.closed,
#         max_len=args.max_len,
#         steering_scale=args.steering_scale,
#         enable_display=True,
#         plot_groups=[
#             ("V", "vx", "vy"),
#             ("delta", "delta_target"),
#             ("V_target", "brake_cmd"),
#             ("r", "alpha", "proj_dist"),
#             ("x", "y"),
#         ],
#     )
#     rc = node.run_display()
#     # rc = 0
    
#     try:
#         node.destroy_node()
#     except Exception:
#         pass

#     rclpy.shutdown()
#     return int(rc)


# if __name__ == "__main__":
#     raise SystemExit(main())
