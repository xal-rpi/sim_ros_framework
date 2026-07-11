"""LLC scalar command envelopes for tests and smoke scripts.

Wire format: VehicleIoClient.send_command(data) wraps as {"type": "cmd", "data": ...}.
Brake is optional; torque, wheel_speed, steering, and throttle are the primary axes.
"""

from __future__ import annotations

from typing import Any, Dict, Mapping

# name -> cmd payload (no brake unless noted)
LLC_SCALAR_CMD_CASES: Dict[str, Dict[str, Any]] = {
    "torque_only": {"torque": 50.0},
    "torque_and_wheel_speed": {"torque": 50.0, "wheel_speed": 5.0},
    "torque_and_steering": {"torque": 50.0, "steering": 0.05},
    "wheel_speed_and_steering": {"wheel_speed": 5.0, "steering": 0.05},
    "throttle_and_steering": {"throttle": 0.3, "steering": 0.05},
    "steering_only": {"steering": 0.05},
    "steering_input_only": {"steering_input": 0.3},
    "torque_and_steering_input": {"torque": 50.0, "steering_input": 0.2},
    "throttle_only": {"throttle": 0.25},
    "torque_steering_brake": {"torque": 40.0, "steering": -0.03, "brake": 0.0},
}

_SCALAR_KEYS = frozenset({
    "torque", "wheel_speed", "steering", "steering_input", "throttle", "brake",
})


def is_valid_scalar_command(data: Mapping[str, Any]) -> bool:
    """True if at least one LLC scalar actuator field is present (brake alone is not enough)."""
    present = _SCALAR_KEYS.intersection(data.keys())
    if not present:
        return False
    return present - {"brake"} != set()


def cmd_envelope(data: Mapping[str, Any]) -> Dict[str, Any]:
    """Typed cmd envelope as sent on control_listen."""
    return {"type": "cmd", "data": dict(data)}
