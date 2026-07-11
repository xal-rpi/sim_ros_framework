"""High-level xlab companion API — control commands and LLC calibration.

``VehicleSession`` wraps ``VehicleIoClient`` for UDP I/O to one vehicle's xlab
ports (``control_listen``, optional ``control_state_send`` / ``sensor_send`` binds).

Session sources
---------------
1. **Running sim + dispatcher** — reuse the cached session (owns ``sensor_send`` bind)::

    session = VehicleSession.from_vehicle_name("EGO")  # use_cached=True default

2. **External script (default: live sim)** — no config file; matches running sim::

    session = VehicleSession.from_vehicle_name("EGO", recreate=True)

3. **Offline / pre-launch** — compose run YAML (source='config')::

    session = VehicleSession.from_run_config("EGO", "gridworld.yaml")

4. **Known LLC dict** (offline / tests)::

    session = VehicleSession.from_llc_config(llc_cfg)

5. **High-rate control over ROS** — publish ``BngControlCmd`` to
   ``/<vehicle>/control/cmd``; ``sensor_dispatcher`` forwards on the cached session.

Cache
-----
``sensor_dispatcher`` registers one ``VehicleSession`` per vehicle after bind.
``from_vehicle_name(use_cached=True)`` returns that instance. ``recreate=True``
always builds a new session (command sendto only by default) for tune scripts
that must not steal the dispatcher's ``sensor_send`` socket.

Examples
--------
Calibrate from a Jupyter cell while the sim is running::

    from bng_controller.vehicle_session import VehicleSession

    # Reuse dispatcher UDP client (safe for infrequent tune)
    VehicleSession.from_vehicle_name("EGO").calibrate({
        "gains": {"kp": 4000.0, "ki": 13.0},
    })

Dedicated tune connection (does not touch dispatcher cache)::

    with VehicleSession.from_vehicle_name("EGO", recreate=True) as session:
        session.calibrate({"command_timeout": 5.0})

Scalar control from Python (prefer ROS ``BngControlCmd`` for loops)::

    from bng_msgs.msg import BngControlCmd

    # Calibrated roadwheel [rad] (uses catalog steering_to_input)
    cmd = BngControlCmd()
    cmd.valid_fields = BngControlCmd.FIELD_TORQUE | BngControlCmd.FIELD_STEERING
    cmd.torque = 80.0
    cmd.steering = 0.05
    session.send_control_cmd(cmd)

    # Unknown steering_to_input — command BeamNG input directly [-1, 1]
    cal = BngControlCmd()
    cal.valid_fields = (
        BngControlCmd.FIELD_TORQUE | BngControlCmd.FIELD_STEERING_INPUT
    )
    cal.torque = 50.0
    cal.steering_input = 0.3
    session.send_control_cmd(cal)
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, Optional

from bng_msgs.msg import BngControlCmd

from bng_controller.vehicle_config import resolve_vehicle_io
from bng_controller.vehicle_io import VehicleIoClient, VehicleIoEndpoints

if TYPE_CHECKING:
    from bng_controller.vehicle_config import VehicleIoResolution

# Per-vehicle sessions registered by sensor_dispatcher (golden-path cache).
_VEHICLE_SESSIONS: Dict[str, "VehicleSession"] = {}


def get_vehicle_session(vehicle_name: str) -> Optional["VehicleSession"]:
    """Return the dispatcher-owned session for ``vehicle_name``, if registered."""
    return _VEHICLE_SESSIONS.get(vehicle_name)


def register_vehicle_session(vehicle_name: str, session: "VehicleSession") -> None:
    _VEHICLE_SESSIONS[vehicle_name] = session


def unregister_vehicle_session(vehicle_name: str) -> None:
    _VEHICLE_SESSIONS.pop(vehicle_name, None)


def clear_vehicle_sessions() -> None:
    _VEHICLE_SESSIONS.clear()


# (valid_fields bit, BngControlCmd attribute, LLC JSON data key)
_CONTROL_FIELD_SPECS = (
    (BngControlCmd.FIELD_THROTTLE, "throttle", "throttle"),
    (BngControlCmd.FIELD_BRAKE, "brake", "brake"),
    (BngControlCmd.FIELD_STEERING, "steering", "steering"),
    (BngControlCmd.FIELD_STEERING_INPUT, "steering_input", "steering_input"),
    (BngControlCmd.FIELD_TORQUE, "torque", "torque"),
    (BngControlCmd.FIELD_WHEEL_SPEED, "wheel_speed", "wheel_speed"),
    (BngControlCmd.FIELD_PARKING_BRAKE, "pbrake", "pbrake"),
    (BngControlCmd.FIELD_GEAR, "gear_index", "gear_index"),
)


def bng_control_cmd_payload(cmd: BngControlCmd) -> Optional[Dict[str, Any]]:
    """Map ``BngControlCmd`` → LLC scalar ``data`` dict (or None if no-op)."""
    if cmd.valid_fields == 0:
        return None

    payload: Dict[str, Any] = {}
    for bit, attr, key in _CONTROL_FIELD_SPECS:
        if cmd.valid_fields & bit:
            value = getattr(cmd, attr)
            if key == "gear_index":
                payload[key] = int(value)
            else:
                payload[key] = float(value)
    return payload


class VehicleSession:
    """Single-vehicle xlab UDP session for control + optional egress binds."""

    def __init__(self, io: VehicleIoClient) -> None:
        self._io = io

    @property
    def io(self) -> VehicleIoClient:
        return self._io

    @classmethod
    def from_resolution(
        cls,
        resolution: "VehicleIoResolution",
        *,
        bind_control_state: bool = False,
        bind_sensor: bool = False,
        **io_kwargs: Any,
    ) -> "VehicleSession":
        """Open a session from :func:`resolve_vehicle_io` (any process)."""
        return cls.from_endpoints(
            resolution.endpoints,
            bind_control_state=bind_control_state,
            bind_sensor=bind_sensor,
            **io_kwargs,
        )

    @classmethod
    def from_run_config(
        cls,
        vehicle_name: str,
        config_path: str,
        *,
        launch=None,
        bind_control_state: bool = False,
        bind_sensor: bool = False,
        **io_kwargs: Any,
    ) -> "VehicleSession":
        """Resolve I/O from composed run YAML + launch ports (no ROS discovery)."""
        from bng_controller.vehicle_config import LaunchIoParams, resolve_vehicle_io

        resolution = resolve_vehicle_io(
            vehicle_name,
            config_path,
            source="config",
            launch=launch or LaunchIoParams(),
        )
        return cls.from_resolution(
            resolution,
            bind_control_state=bind_control_state,
            bind_sensor=bind_sensor,
            **io_kwargs,
        )

    @classmethod
    def from_vehicle_name(
        cls,
        vehicle_name: str,
        node_ros=None,
        *,
        use_cached: bool = True,
        recreate: bool = False,
        config_path: Optional[str] = None,
        launch=None,
        source: str = "manager",
        fallback_to_config: bool = False,
        bind_control_state: bool = False,
        bind_sensor: bool = False,
        config_timeout_sec: float = 5.0,
        **io_kwargs: Any,
    ) -> "VehicleSession":
        """Build a session for ``vehicle_name``.

        Args:
            vehicle_name: Key in scenario ``vehicles`` (e.g. ``EGO``).
            node_ros: Optional existing rclpy node (reuses discovery context).
            use_cached: If True and dispatcher registered a session, return it.
            recreate: If True, always open a new session (never use cache).
            config_path: Run file — only for source='config' or fallback_to_config.
            launch: Base UDP ports — only for source='config' fallback.
            source: ``manager`` (default, live sim) or ``config`` (YAML compose).
            fallback_to_config: When manager fails, try config_path compose.
            bind_control_state: Bind ``control_state_send`` (exclusive).
            bind_sensor: Bind ``sensor_send`` (exclusive; conflicts with dispatcher).
        """
        if use_cached and not recreate:
            cached = get_vehicle_session(vehicle_name)
            if cached is not None:
                return cached

        from bng_controller.vehicle_config import LaunchIoParams

        resolution = resolve_vehicle_io(
            vehicle_name,
            config_path,
            source=source,  # type: ignore[arg-type]
            launch=launch or LaunchIoParams(),
            node_ros=node_ros,
            timeout_sec=config_timeout_sec,
            fallback_to_config=fallback_to_config,
        )
        return cls.from_resolution(
            resolution,
            bind_control_state=bind_control_state,
            bind_sensor=bind_sensor,
            **io_kwargs,
        )

    @classmethod
    def from_endpoints(
        cls,
        endpoints: VehicleIoEndpoints,
        *,
        bind_control_state: bool = False,
        bind_sensor: bool = False,
        **io_kwargs: Any,
    ) -> "VehicleSession":
        return cls(
            VehicleIoClient(
                command_addr=endpoints.command_addr,
                control_state_bind_addr=(
                    endpoints.control_state_bind_addr if bind_control_state else None
                ),
                sensor_bind_addr=endpoints.sensor_bind_addr if bind_sensor else None,
                **io_kwargs,
            )
        )

    @classmethod
    def from_llc_config(
        cls,
        llc_cfg: Dict[str, Any],
        *,
        bind_control_state: bool = False,
        bind_sensor: bool = False,
        **io_kwargs: Any,
    ) -> "VehicleSession":
        endpoints = VehicleIoEndpoints.from_low_level_controller_config(llc_cfg)
        return cls.from_endpoints(
            endpoints,
            bind_control_state=bind_control_state,
            bind_sensor=bind_sensor,
            **io_kwargs,
        )

    def calibrate(self, params: Dict[str, Any]) -> None:
        """Send LLC tune envelope (``controller_llc.onTune`` / ``calibrate``)."""
        self._io.send_tune(params)

    def send_control_cmd(self, cmd: BngControlCmd) -> None:
        """Forward scalar LLC command (``controller_llc.loadScalar``)."""
        payload = bng_control_cmd_payload(cmd)
        if payload is not None:
            self._io.send_command(payload)

    def close(self) -> None:
        self._io.close()

    def __enter__(self) -> "VehicleSession":
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()
