#!/usr/bin/env python3
"""Queue-driven Qt live visualizer for the week11 MPC controller."""

from __future__ import annotations

import atexit
import multiprocessing as mp
import queue
import signal
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np


_THIS_DIR = Path(__file__).resolve().parent
_WEEK10_DIR = _THIS_DIR.parent / "week10"
for _path in (_THIS_DIR, _WEEK10_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from simulator_cartesian import fiala_params
from tracking_helper import TrackingReference

try:
    from PySide6 import QtCore, QtWidgets  # pyright: ignore[reportMissingImports]
except Exception as exc:  # pragma: no cover
    raise RuntimeError(
        "PySide6 is required for the week11 live visualizer. Install with: pip install PySide6"
    ) from exc

try:
    import pyqtgraph as pg  # pyright: ignore[reportMissingImports]
except Exception as exc:  # pragma: no cover
    raise RuntimeError(
        "pyqtgraph is required for the week11 live visualizer. Install with: pip install pyqtgraph"
    ) from exc


pg.setConfigOptions(antialias=False)


@dataclass(frozen=True)
class MPCVisualizerConfig:
    update_interval_ms: int = 50
    queue_size: int = 1
    max_history: int = 1500
    vehicle_scale: float = 1.5
    auto_si_prefix: bool = False
    window_width: int = 1500
    window_height: int = 920
    title: str = "Week11 MPC Visualizer"
    background: str = "#101418"
    foreground: str = "#e6eef8"


@dataclass(frozen=True)
class PlotSeriesSpec:
    key: str
    title: str
    ylabel: str
    history_source: str
    history_key: Optional[str] = None
    predicted_source: Optional[str] = None
    predicted_key: Optional[str] = None
    reference_key: Optional[str] = None


STATE_SPECS = (
    PlotSeriesSpec("e", "Lateral Error", "e [m]", "frenet", "e", "predicted_state", "e", "e"),
    PlotSeriesSpec("dphi", "Heading Error", "dphi [rad]", "frenet", "dphi", "predicted_state", "dphi", "dphi"),
    PlotSeriesSpec("V", "Speed", "V [m/s]", "measured_state", "V", "predicted_state", "V", "V"),
    PlotSeriesSpec("r", "Yaw Rate", "r [rad/s]", "measured_state", "r", "predicted_state", "r", "r"),
    PlotSeriesSpec("beta", "Slip Angle", "beta [rad]", "measured_state", "beta", "predicted_state", "beta", "beta"),
    PlotSeriesSpec("wr", "Rear Wheel Speed", "wr [m/s]", "measured_state", "wr", "predicted_state", "rear_wheelspeed_ms", "wr"),
)

CONTROL_SPECS = (
    PlotSeriesSpec(
        "roadwheel_angle",
        "Steering Command",
        "delta [rad]",
        "control",
        "roadwheel_angle",
        "predicted_control",
        "roadwheel_angle",
        "delta",
    ),
    PlotSeriesSpec(
        "rear_wheel_torque",
        "Rear Torque Command",
        "torque [Nm]",
        "control",
        "rear_wheel_torque",
        "predicted_control",
        "rear_wheel_torque",
        "rear_wheel_torque",
    ),
)

EXTRA_SPECS = (
    PlotSeriesSpec("solve_time", "Solve Time", "solve [s]", "root", "solve_time"),
)

ALL_SPECS = STATE_SPECS + CONTROL_SPECS + EXTRA_SPECS


def _dash_line_style():
    if hasattr(QtCore.Qt, "PenStyle"):
        return QtCore.Qt.PenStyle.DashLine
    return QtCore.Qt.DashLine


def _to_float(value: Any, *, default: float = float("nan")) -> float:
    try:
        out = float(value)
    except Exception:
        return default
    return out if np.isfinite(out) else default


def _as_float_array(value: Any) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.ndim == 0:
        arr = arr.reshape(1)
    return np.array(arr, copy=True)


def build_reference_payload(reference: TrackingReference) -> dict[str, object]:
    zeros = np.zeros_like(reference.s)
    x_left, y_left, _ = reference.frenet_to_cartesian(reference.s, reference.e_max, zeros)
    x_right, y_right, _ = reference.frenet_to_cartesian(reference.s, reference.e_min, zeros)
    control_map = {
        "roadwheel_angle": "delta",
        "rear_wheel_torque": "rear_wheel_torque",
    }
    return {
        "s": np.array(reference.s, copy=True),
        "x": np.array(reference.x, copy=True),
        "y": np.array(reference.y, copy=True),
        "x_left": np.array(x_left, copy=True),
        "y_left": np.array(y_left, copy=True),
        "x_right": np.array(x_right, copy=True),
        "y_right": np.array(y_right, copy=True),
        "state_reference": {
            spec.key: np.array(reference.state_profiles[spec.reference_key], copy=True)
            for spec in STATE_SPECS
            if spec.reference_key is not None
        },
        "control_reference": {
            spec.key: np.array(reference.state_profiles[control_map[spec.key]], copy=True)
            for spec in CONTROL_SPECS
        },
        "state_specs": [asdict(spec) for spec in STATE_SPECS],
        "control_specs": [asdict(spec) for spec in CONTROL_SPECS],
        "extra_specs": [asdict(spec) for spec in EXTRA_SPECS],
    }


def build_snapshot_payload(record: dict[str, Any]) -> dict[str, object]:
    return {
        "time": _to_float(record.get("time", float("nan"))),
        "solve_time": _to_float(record.get("solve_time", float("nan"))),
        "frenet": {
            key: _to_float(value)
            for key, value in record.get("frenet", {}).items()
        },
        "measured_state": {
            key: _to_float(value)
            for key, value in record.get("measured_state", {}).items()
        },
        "control": {
            key: _to_float(value)
            for key, value in record.get("control", {}).items()
        },
        "vehicle_pose": {
            key: _to_float(value)
            for key, value in record.get("vehicle_pose", {}).items()
        },
        "predicted_state": {
            key: _as_float_array(value)
            for key, value in record.get("predicted_state", {}).items()
        },
        "predicted_control": {
            key: _as_float_array(value)
            for key, value in record.get("predicted_control", {}).items()
        },
        "predicted_cartesian": {
            key: _as_float_array(value)
            for key, value in record.get("predicted_cartesian", {}).items()
        },
        "ref_traj": {
            key: _as_float_array(value)
            for key, value in record.get("ref_traj", {}).items()
        },
    }


def publish_latest_snapshot(snapshot_queue, payload: dict[str, object] | None) -> None:
    try:
        while True:
            snapshot_queue.get_nowait()
    except queue.Empty:
        pass
    except Exception:
        pass

    try:
        snapshot_queue.put_nowait(payload)
    except queue.Full:
        try:
            snapshot_queue.get_nowait()
        except queue.Empty:
            pass
        except Exception:
            pass
        try:
            snapshot_queue.put_nowait(payload)
        except queue.Full:
            pass
        except Exception:
            pass
    except Exception:
        pass


def start_visualization_process(snapshot_queue, ref_payload: dict[str, object], cfg: Optional[MPCVisualizerConfig] = None) -> int:
    def _signal_handler(signum, _frame) -> None:
        print(f"Visualizer received signal {signum}; shutting down")
        app_instance = QtWidgets.QApplication.instance()
        if app_instance is not None:
            app_instance.quit()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        app.setQuitOnLastWindowClosed(True)
        vis = MPCVisualizer(snapshot_queue, ref_payload, cfg=cfg)
        atexit.register(lambda: cleanup_visualizer(vis, app))
        if hasattr(app, "exec"):
            return int(app.exec())
        return int(app.exec_())
    except Exception as exc:
        print(f"Error in visualization process: {exc}")
        return 1


def cleanup_visualizer(vis, app) -> None:
    try:
        if vis is not None:
            vis.stop_visualization()
        if app is not None:
            app.quit()
    except Exception as exc:
        print(f"Visualizer cleanup failed: {exc}")


def launch_visualizer_process(
    reference: TrackingReference,
    *,
    cfg: Optional[MPCVisualizerConfig] = None,
):
    cfg_use = cfg or MPCVisualizerConfig()
    if sys.platform.startswith("linux"):
        ctx = mp.get_context("fork")
    else:  # pragma: no cover
        ctx = mp.get_context("spawn")
    snapshot_queue = ctx.Queue(maxsize=cfg_use.queue_size)
    process = ctx.Process(
        target=start_visualization_process,
        args=(snapshot_queue, build_reference_payload(reference), cfg_use),
        daemon=False,
    )
    process.start()
    time.sleep(0.3)
    if not process.is_alive():
        raise RuntimeError(
            f"Visualizer process exited during startup with exit code {process.exitcode}. "
            "This usually means Qt failed to initialize."
        )
    return process, snapshot_queue


def stop_visualizer_process(process, snapshot_queue, *, timeout: float = 2.0) -> None:
    if snapshot_queue is not None:
        try:
            publish_latest_snapshot(snapshot_queue, None)
        except Exception:
            pass
    if process is None:
        return
    process.join(timeout=timeout)
    if process.is_alive():
        process.terminate()
        process.join(timeout=1.0)


class VehicleGlyph:
    def __init__(self, plot_item, *, veh: dict[str, float], scale: float, color: str = "#7dd3a5"):
        self.a = float(veh["cogToFrontAxle"]) * scale
        self.b = float(veh["cogToRearAxle"]) * scale
        self.hw = float(veh.get("track_width", 1.787)) * 0.5 * scale
        self.wl = 0.6 * scale
        self.ww = 0.2 * scale
        pen = pg.mkPen(color, width=2)
        self._items = {
            name: pg.PlotCurveItem(pen=pen)
            for name in (
                "body",
                "front_axle",
                "rear_axle",
                "wheel_fl",
                "wheel_fr",
                "wheel_rl",
                "wheel_rr",
            )
        }
        for item in self._items.values():
            plot_item.addItem(item)

    @staticmethod
    def _segment(start: tuple[float, float], end: tuple[float, float]) -> tuple[np.ndarray, np.ndarray]:
        return np.array([start[0], end[0]], dtype=float), np.array([start[1], end[1]], dtype=float)

    @staticmethod
    def _rect(center_x: float, center_y: float, yaw: float, length: float, width: float) -> tuple[np.ndarray, np.ndarray]:
        corners = np.array(
            [
                [length * 0.5, width * 0.5],
                [length * 0.5, -width * 0.5],
                [-length * 0.5, -width * 0.5],
                [-length * 0.5, width * 0.5],
                [length * 0.5, width * 0.5],
            ],
            dtype=float,
        )
        c = float(np.cos(yaw))
        s = float(np.sin(yaw))
        rot = np.array([[c, -s], [s, c]], dtype=float)
        pts = corners @ rot.T
        pts[:, 0] += center_x
        pts[:, 1] += center_y
        return pts[:, 0], pts[:, 1]

    def update(self, *, x: float, y: float, yaw: float, steer: float) -> None:
        c = float(np.cos(yaw))
        s = float(np.sin(yaw))
        wheel_yaw = float(yaw + steer)
        front_center = (x + self.a * c, y + self.a * s)
        rear_center = (x - self.b * c, y - self.b * s)
        fl = (front_center[0] - self.hw * s, front_center[1] + self.hw * c)
        fr = (front_center[0] + self.hw * s, front_center[1] - self.hw * c)
        rl = (rear_center[0] - self.hw * s, rear_center[1] + self.hw * c)
        rr = (rear_center[0] + self.hw * s, rear_center[1] - self.hw * c)

        self._items["body"].setData(*self._segment(rear_center, front_center))
        self._items["front_axle"].setData(*self._segment(fl, fr))
        self._items["rear_axle"].setData(*self._segment(rl, rr))
        self._items["wheel_fl"].setData(*self._rect(fl[0], fl[1], wheel_yaw, self.wl, self.ww))
        self._items["wheel_fr"].setData(*self._rect(fr[0], fr[1], wheel_yaw, self.wl, self.ww))
        self._items["wheel_rl"].setData(*self._rect(rl[0], rl[1], yaw, self.wl, self.ww))
        self._items["wheel_rr"].setData(*self._rect(rr[0], rr[1], yaw, self.wl, self.ww))


class MPCVisualizer(QtWidgets.QMainWindow):
    def __init__(self, snapshot_queue, ref_payload: dict[str, object], *, cfg: Optional[MPCVisualizerConfig] = None):
        super().__init__()
        self.snapshot_queue = snapshot_queue
        self.ref_payload = ref_payload
        self.cfg = cfg or MPCVisualizerConfig()
        self.running = True

        self.ref_s = _as_float_array(ref_payload["s"])
        self.ref_x = _as_float_array(ref_payload["x"])
        self.ref_y = _as_float_array(ref_payload["y"])
        self.ref_x_left = _as_float_array(ref_payload["x_left"])
        self.ref_y_left = _as_float_array(ref_payload["y_left"])
        self.ref_x_right = _as_float_array(ref_payload["x_right"])
        self.ref_y_right = _as_float_array(ref_payload["y_right"])
        self.state_reference = {
            key: _as_float_array(value)
            for key, value in ref_payload["state_reference"].items()
        }
        self.control_reference = {
            key: _as_float_array(value)
            for key, value in ref_payload["control_reference"].items()
        }
        self.series_specs = tuple(PlotSeriesSpec(**spec) for spec in ref_payload["state_specs"])
        self.series_specs += tuple(PlotSeriesSpec(**spec) for spec in ref_payload["control_specs"])
        self.series_specs += tuple(PlotSeriesSpec(**spec) for spec in ref_payload["extra_specs"])

        self.history_s = deque(maxlen=self.cfg.max_history)
        self.history_xy = deque(maxlen=self.cfg.max_history)
        self.history_values = {
            spec.key: deque(maxlen=self.cfg.max_history)
            for spec in self.series_specs
        }

        self.plot_widgets: dict[str, pg.PlotWidget] = {}
        self.curves: dict[str, pg.PlotDataItem] = {}

        self._setup_ui()
        self._draw_reference()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(int(self.cfg.update_interval_ms))

    def _setup_ui(self) -> None:
        self.setWindowTitle(self.cfg.title)
        self.resize(self.cfg.window_width, self.cfg.window_height)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(6)

        main_layout.addWidget(self._build_legend())

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter, stretch=1)

        self.xy_plot = pg.PlotWidget()
        self._configure_plot_axes(self.xy_plot)
        self.xy_plot.setBackground(self.cfg.background)
        self.xy_plot.showGrid(x=True, y=True, alpha=0.2)
        self.xy_plot.setAspectLocked(True)
        self.xy_plot.setTitle("2D Tracking View", color=self.cfg.foreground)
        self.xy_plot.setLabel("bottom", "x [m]")
        self.xy_plot.setLabel("left", "y [m]")
        splitter.addWidget(self.xy_plot)

        right_panel = QtWidgets.QWidget()
        right_layout = QtWidgets.QGridLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(6)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 4)

        first_plot = None
        cols = 2
        for idx, spec in enumerate(self.series_specs):
            row = idx // cols
            col = idx % cols
            plot = pg.PlotWidget()
            self._configure_plot_axes(plot)
            plot.setBackground(self.cfg.background)
            plot.showGrid(x=True, y=True, alpha=0.2)
            plot.setTitle(spec.title, color=self.cfg.foreground)
            plot.setLabel("left", spec.ylabel)
            plot.setLabel("bottom", "s [m]")
            plot.setXRange(float(self.ref_s[0]), float(self.ref_s[-1]), padding=0.02)
            if first_plot is None:
                first_plot = plot
            else:
                plot.setXLink(first_plot)
            self.plot_widgets[spec.key] = plot
            right_layout.addWidget(plot, row, col)

            ref_curve = plot.plot(pen=pg.mkPen("#facc15", width=1.8))
            pred_curve = plot.plot(pen=pg.mkPen("#f97316", width=2.0))
            hist_curve = plot.plot(pen=pg.mkPen("#22c55e", width=1.8))
            self.curves[f"{spec.key}_ref"] = ref_curve
            self.curves[f"{spec.key}_pred"] = pred_curve
            self.curves[f"{spec.key}_hist"] = hist_curve

        self.xy_reference = self.xy_plot.plot(pen=pg.mkPen("#facc15", width=2.0))
        self.xy_left_bound = self.xy_plot.plot(pen=pg.mkPen("#64748b", width=1.0, style=_dash_line_style()))
        self.xy_right_bound = self.xy_plot.plot(pen=pg.mkPen("#64748b", width=1.0, style=_dash_line_style()))
        self.xy_history = self.xy_plot.plot(pen=pg.mkPen("#22c55e", width=2.0))
        self.xy_prediction = self.xy_plot.plot(pen=pg.mkPen("#f97316", width=2.0))
        self.xy_start = pg.ScatterPlotItem(size=9, brush=pg.mkBrush("#86efac"), pen=None)
        self.xy_current = pg.ScatterPlotItem(size=10, brush=pg.mkBrush("#ef4444"), pen=None)
        self.xy_plot.addItem(self.xy_start)
        self.xy_plot.addItem(self.xy_current)
        self.vehicle_glyph = VehicleGlyph(
            self.xy_plot,
            veh=fiala_params["vehicle"],
            scale=self.cfg.vehicle_scale,
            color="#86efac",
        )

        x_pad = 0.05 * max(1.0, float(np.max(self.ref_x) - np.min(self.ref_x)))
        y_pad = 0.05 * max(1.0, float(np.max(self.ref_y) - np.min(self.ref_y)))
        self.xy_plot.setXRange(float(np.min(self.ref_x) - x_pad), float(np.max(self.ref_x) + x_pad), padding=0.0)
        self.xy_plot.setYRange(float(np.min(self.ref_y) - y_pad), float(np.max(self.ref_y) + y_pad), padding=0.0)

        self.show()

    def _configure_plot_axes(self, plot: pg.PlotWidget) -> None:
        if self.cfg.auto_si_prefix:
            return

        for axis_name in ("left", "bottom"):
            try:
                plot.getAxis(axis_name).enableAutoSIPrefix(False)
            except Exception:
                pass

    def _build_legend(self) -> QtWidgets.QWidget:
        frame = QtWidgets.QFrame()
        if hasattr(QtWidgets.QFrame, "Shape"):
            frame.setFrameShape(QtWidgets.QFrame.Shape.Box)
            frame.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        else:  # pragma: no cover
            frame.setFrameStyle(QtWidgets.QFrame.Box | QtWidgets.QFrame.Sunken)
        layout = QtWidgets.QHBoxLayout(frame)
        layout.setContentsMargins(10, 6, 10, 6)
        items = (
            ("Reference", "#facc15"),
            ("Active Horizon", "#f97316"),
            ("History", "#22c55e"),
            ("Current Pose", "#ef4444"),
        )
        for label_text, color in items:
            label = QtWidgets.QLabel(f"\u25a0 {label_text}")
            label.setStyleSheet(f"color: {color}; font-weight: bold;")
            layout.addWidget(label)
        layout.addStretch(1)
        return frame

    def _draw_reference(self) -> None:
        self.xy_reference.setData(self.ref_x, self.ref_y)
        self.xy_left_bound.setData(self.ref_x_left, self.ref_y_left)
        self.xy_right_bound.setData(self.ref_x_right, self.ref_y_right)
        for spec in self.series_specs:
            if spec.key in self.state_reference:
                self.curves[f"{spec.key}_ref"].setData(self.ref_s, self.state_reference[spec.key])
            elif spec.key in self.control_reference:
                self.curves[f"{spec.key}_ref"].setData(self.ref_s, self.control_reference[spec.key])
            else:
                self.curves[f"{spec.key}_ref"].setData([], [])

    def stop_visualization(self) -> None:
        self.running = False
        if hasattr(self, "timer") and self.timer is not None:
            self.timer.stop()
        self.close()

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.running = False
        if hasattr(self, "timer") and self.timer is not None:
            self.timer.stop()
        event.accept()

    def _drain_queue(self) -> Optional[dict[str, object]]:
        latest = None
        while True:
            try:
                data = self.snapshot_queue.get_nowait()
            except queue.Empty:
                break
            except Exception as exc:
                print(f"Visualizer queue read failed: {exc}")
                break
            if data is None:
                self.stop_visualization()
                return None
            latest = data
        return latest

    def _extract_scalar(self, snapshot: dict[str, object], spec: PlotSeriesSpec) -> float:
        if spec.history_source == "root":
            return _to_float(snapshot.get(spec.history_key or spec.key, float("nan")))
        source = snapshot.get(spec.history_source, {})
        if isinstance(source, dict):
            return _to_float(source.get(spec.history_key or spec.key, float("nan")))
        return float("nan")

    def update_plots(self) -> None:
        if not self.running:
            return

        snapshot = self._drain_queue()
        if snapshot is None:
            return

        frenet = snapshot.get("frenet", {})
        vehicle_pose = snapshot.get("vehicle_pose", {})
        control = snapshot.get("control", {})
        current_s = _to_float(frenet.get("s", float("nan")))
        current_x = _to_float(vehicle_pose.get("x", float("nan")))
        current_y = _to_float(vehicle_pose.get("y", float("nan")))
        current_psi = _to_float(vehicle_pose.get("psi", float("nan")))
        current_delta = _to_float(vehicle_pose.get("delta", float("nan")))
        if not np.isfinite(current_delta):
            current_delta = _to_float(control.get("roadwheel_angle", 0.0), default=0.0)

        if np.isfinite(current_s):
            self.history_s.append(current_s)
        if np.isfinite(current_x) and np.isfinite(current_y):
            self.history_xy.append((current_x, current_y))

        for spec in self.series_specs:
            self.history_values[spec.key].append(self._extract_scalar(snapshot, spec))

        history_s_arr = np.asarray(self.history_s, dtype=float)
        if history_s_arr.size:
            for spec in self.series_specs:
                history_values = np.asarray(self.history_values[spec.key], dtype=float)
                self.curves[f"{spec.key}_hist"].setData(history_s_arr, history_values)

        predicted_state = snapshot.get("predicted_state", {})
        predicted_control = snapshot.get("predicted_control", {})
        for spec in self.series_specs:
            x_data = None
            y_data = None
            if spec.predicted_source == "predicted_state" and isinstance(predicted_state, dict):
                if "s" in predicted_state and spec.predicted_key in predicted_state:
                    x_data = _as_float_array(predicted_state["s"])
                    y_data = _as_float_array(predicted_state[spec.predicted_key])
            elif spec.predicted_source == "predicted_control" and isinstance(predicted_control, dict):
                if "s" in predicted_control and spec.predicted_key in predicted_control:
                    x_data = _as_float_array(predicted_control["s"])
                    y_data = _as_float_array(predicted_control[spec.predicted_key])
            if x_data is None or y_data is None:
                self.curves[f"{spec.key}_pred"].setData([], [])
            else:
                self.curves[f"{spec.key}_pred"].setData(x_data, y_data)

        if self.history_xy:
            xy_hist = np.asarray(self.history_xy, dtype=float)
            self.xy_history.setData(xy_hist[:, 0], xy_hist[:, 1])
            self.xy_start.setData([xy_hist[0, 0]], [xy_hist[0, 1]])
            self.xy_current.setData([xy_hist[-1, 0]], [xy_hist[-1, 1]])

        predicted_cartesian = snapshot.get("predicted_cartesian", {})
        if isinstance(predicted_cartesian, dict) and "x" in predicted_cartesian and "y" in predicted_cartesian:
            self.xy_prediction.setData(
                _as_float_array(predicted_cartesian["x"]),
                _as_float_array(predicted_cartesian["y"]),
            )

        if np.isfinite(current_x) and np.isfinite(current_y) and np.isfinite(current_psi):
            self.vehicle_glyph.update(x=current_x, y=current_y, yaw=current_psi, steer=current_delta)


__all__ = [
    "MPCVisualizer",
    "MPCVisualizerConfig",
    "build_reference_payload",
    "build_snapshot_payload",
    "launch_visualizer_process",
    "publish_latest_snapshot",
    "start_visualization_process",
    "stop_visualizer_process",
]
