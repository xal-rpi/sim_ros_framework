"""Plant / sensor calibration procedures (settle gate, steering_to_input, τ).

Thin BeamNG RPCs stay in vehicle_properties. This module waits on the live
sim and commands steering_input.
"""

import time
from logging import getLogger
from typing import Sequence

from beamngpy import Vehicle
from beamngpy.logging import LOGGER_ID

from bng_simulator.core.vehicle_properties import (
    control_vehicle,
    get_front_roadwheel_steer,
    get_settle_state,
)

logger = getLogger(f"{LOGGER_ID}.calibration")


def settle_vehicle(
    vehicle: Vehicle,
    beamng=None,
    timeout: float = 5.0,
    hold_s: float = 0.5,
    v_max: float = 0.02,
    omega_max: float = 0.15,
    z_walk_max: float = 0.005,
    attach_mode: bool = False,
    logger=None,
) -> dict:
    """Hold the vehicle and wait until it is at rest, then sample anyway.

    Parking brake on, then require ~hold_s of |v| < v_max, small ω, and a
    stable CG z. Timeout logs elapsed + |v| and returns the last sample.
    ``beamng`` is unused (sim is already running); kept for the shared helper.
    """
    _log = logger if logger is not None else getLogger(f"{LOGGER_ID}.calibration")
    _ = beamng

    first_speed = None
    try:
        peek = get_settle_state(vehicle)
        first_speed = float((peek or {}).get("speed") or 0.0)
    except Exception as exc:
        _log.warning(f"GetSettleState peek failed: {exc}")
        peek = None

    moving_attach = bool(attach_mode and first_speed is not None and first_speed > 1.0)
    if moving_attach:
        _log.warning(
            f"ATTACH settle: vehicle moving (|v|={first_speed:.3f} m/s); waiting, not snapping mid-drive"
        )
    else:
        control_vehicle(
            vehicle,
            throttle=0.0,
            brake=0.0,
            steering_input=0.0,
            parkingbrake=True,
        )

    t0 = time.time()
    stable_s = 0.0
    dt = 0.05
    last = peek or {"speed": None, "omega": None, "cogZ": None}
    last_z = None if last.get("cogZ") is None else float(last["cogZ"])

    while True:
        time.sleep(dt)
        try:
            last = get_settle_state(vehicle) or last
        except Exception as exc:
            _log.warning(f"GetSettleState failed: {exc}")
            break
        speed = float(last.get("speed") or 0.0)
        omega = float(last.get("omega") or 0.0)
        cog_z = last.get("cogZ")
        z_walk = 0.0 if last_z is None or cog_z is None else abs(float(cog_z) - last_z)
        if cog_z is not None:
            last_z = float(cog_z)
        if speed < v_max and omega < omega_max and z_walk < z_walk_max:
            stable_s += dt
            if stable_s >= hold_s:
                break
        else:
            stable_s = 0.0
        if (time.time() - t0) >= timeout:
            _log.warning(
                f"settle timeout after {timeout:.2f} s, sampling anyway "
                f"(|v|={speed:.4f}, |ω|={omega:.4f})"
            )
            break

    elapsed = time.time() - t0
    speed = float(last.get("speed") or 0.0)
    _log.info(
        f"settle done in {elapsed:.2f} s (|v|={speed:.4f} m/s, "
        f"|ω|={float(last.get('omega') or 0.0):.4f}, cogZ={last.get('cogZ')})"
    )
    return {
        "elapsed_s": elapsed,
        "speed": speed,
        "omega": last.get("omega"),
        "cogZ": last.get("cogZ"),
        "first_speed": first_speed,
        "timed_out": elapsed >= timeout and stable_s < hold_s,
    }


def _read_delta(vehicle: Vehicle) -> tuple[float, dict]:
    row = get_front_roadwheel_steer(vehicle)
    return float(row["delta"]), row


def wait_roadwheel_settled(
    vehicle: Vehicle,
    timeout: float = 4.0,
    rate_max: float = 0.015,
    hold_s: float = 0.4,
    dt: float = 0.05,
    logger=None,
) -> tuple[dict, float, bool]:
    """Wait until |dδ/dt| stays below ``rate_max`` [rad/s] for ``hold_s``.

    Returns (last_row, elapsed_s, timed_out).
    """
    _log = logger
    t0 = time.time()
    last_d = None
    stable_s = 0.0
    row = None
    while (time.time() - t0) < timeout:
        time.sleep(dt)
        d, row = _read_delta(vehicle)
        if last_d is not None:
            rate = abs(d - last_d) / dt
            if rate < rate_max:
                stable_s += dt
                if stable_s >= hold_s:
                    elapsed = time.time() - t0
                    if _log is not None:
                        _log.info(
                            f"roadwheel settled in {elapsed:.2f} s  δ={d:+.4f}  |dδ/dt|<{rate_max}"
                        )
                    return row, elapsed, False
            else:
                stable_s = 0.0
        last_d = d
    if row is None:
        _, row = _read_delta(vehicle)
    elapsed = time.time() - t0
    if _log is not None:
        _log.warning(
            f"roadwheel settle timeout after {elapsed:.2f} s  δ={float(row['delta']):+.4f}"
        )
    return row, elapsed, True


def measure_steering_tau(
    vehicle: Vehicle,
    u_step: float = 1.0,
    dt: float = 0.02,
    timeout: float = 5.0,
    logger=None,
) -> dict:
    """Worst-case first-order τ: hold 0, step to ``u_step``, parked on ground.

    τ = time to 63.2% of the observed δ span. Also reports 5% delay and t95.
    """
    _log = logger
    control_vehicle(vehicle, steering_input=0.0, parkingbrake=True, throttle=0.0)
    row0, _, _ = wait_roadwheel_settled(vehicle, timeout=4.0, logger=_log)
    d0 = float(row0["delta"])

    t_cmd = time.time()
    control_vehicle(vehicle, steering_input=float(u_step), parkingbrake=True, throttle=0.0)

    series = []
    last_d = d0
    stable_s = 0.0
    while (time.time() - t_cmd) < timeout:
        time.sleep(dt)
        t = time.time() - t_cmd
        d, _ = _read_delta(vehicle)
        series.append({"t": t, "delta": d})
        rate = abs(d - last_d) / dt
        if rate < 0.015:
            stable_s += dt
            if stable_s >= 0.35 and t > 0.15:
                break
        else:
            stable_s = 0.0
        last_d = d

    dss = series[-1]["delta"] if series else d0
    span = dss - d0
    t_delay = t_tau = t_95 = None
    if abs(span) >= 0.02:
        for sample in series:
            frac = (sample["delta"] - d0) / span
            if t_delay is None and frac >= 0.05:
                t_delay = sample["t"]
            if t_tau is None and frac >= 0.632:
                t_tau = sample["t"]
            if t_95 is None and frac >= 0.95:
                t_95 = sample["t"]
        if _log is not None:
            _log.info(
                f"steer τ (0→{u_step:g}): δ0={d0:+.4f} δss={dss:+.4f}  "
                f"delay5%={t_delay}  τ63%={t_tau}  t95={t_95}  n={len(series)}"
            )
    elif _log is not None:
        _log.warning(
            f"steer τ: δ span too small ({span:+.4f}); step may not have reached the roadwheel"
        )

    return {
        "steering_tau_s": t_tau,
        "steering_delay_s": t_delay,
        "steering_t95_s": t_95,
        "delta_0": d0,
        "delta_ss": dss,
        "u_step": float(u_step),
        "n_samples": len(series),
    }


def _hold_steering(vehicle: Vehicle, u: float, logger=None) -> dict:
    """Command ``steering_input`` and wait until hydros stop, then sample δ."""
    control_vehicle(vehicle, steering_input=float(u), parkingbrake=True, throttle=0.0)
    last, wait_s, timed_out = wait_roadwheel_settled(vehicle, timeout=5.0, logger=logger)
    delta = float(last["delta"])
    k = None if abs(u) < 1e-6 else delta / float(u)
    return {
        "u": float(u),
        "delta_l": float(last.get("delta_l", 0.0)),
        "delta_r": float(last.get("delta_r", 0.0)),
        "delta": delta,
        "k": k,
        "wait_s": wait_s,
        "timed_out": timed_out,
    }


def measure_steering_to_input(
    vehicle: Vehicle,
    beamng=None,
    slope_inputs: Sequence[float] = (-0.3, 0.3),
    lock_inputs: Sequence[float] = (-1.0, 1.0),
    hold_s: float = 1.0,
    logger=None,
) -> dict:
    """Measure mid-range slope and hydro lock after the roadwheel has stopped.

    ``steering_to_input`` is mean(δ_avg / u) at |u| ≈ 0.3, not at lock.
    Lock is the settled average at u = ±1 (Ackermann included).
    Also steps 0→1 (parked) for a first-order τ / 5% delay.
    ``hold_s`` is unused; wait is |dδ/dt| gated.
    """
    _log = logger if logger is not None else getLogger(f"{LOGGER_ID}.calibration")
    _ = beamng
    _ = hold_s
    settle = settle_vehicle(vehicle, beamng=beamng, logger=_log)

    details = []
    tau = {}
    steering = None
    try:
        row0 = _hold_steering(vehicle, 0.0, logger=_log)
        details.append(row0)
        d0 = row0["delta"]
        _log.info(
            f"steer u=0  δ_l={row0['delta_l']:+.4f}  "
            f"δ_r={row0['delta_r']:+.4f}  "
            f"δ={d0:+.4f}  (must be ~0; do not offset YAML)"
        )
        if abs(d0) > 0.03:
            _log.warning(
                f"|roadwheel_at_u0|={abs(d0):.4f} rad is not near zero — "
                "forward axis is still wrong; not writing an offset"
            )

        slope_k = []
        for u in slope_inputs:
            if abs(u) < 1e-6:
                continue
            row = _hold_steering(vehicle, u, logger=_log)
            details.append(row)
            slope_k.append(row["k"])
            _log.info(
                f"steer slope u={u:+.3f}  δ_l={row['delta_l']:+.4f}  "
                f"δ_r={row['delta_r']:+.4f}  δ={row['delta']:+.4f}  "
                f"k=δ/u={row['k']:+.6f}  wait={row['wait_s']:.2f}s"
            )

        lock_by_u = {}
        for u in lock_inputs:
            row = _hold_steering(vehicle, u, logger=_log)
            details.append(row)
            lock_by_u[float(u)] = row
            _log.info(
                f"steer lock  u={u:+.3f}  δ_l={row['delta_l']:+.4f}  "
                f"δ_r={row['delta_r']:+.4f}  δ={row['delta']:+.4f}  "
                f"(measured lock, not k)  wait={row['wait_s']:.2f}s"
            )

        umin = min(lock_by_u) if lock_by_u else -1.0
        umax = max(lock_by_u) if lock_by_u else 1.0
        at_min = lock_by_u.get(umin)
        at_max = lock_by_u.get(umax)
        if at_min is None or at_max is None:
            raise RuntimeError("lock samples at u=±1 missing")
        if not slope_k:
            raise RuntimeError("no mid-range steering_input samples")

        k_mean = sum(slope_k) / len(slope_k)
        steering = {
            "input_min": umin,
            "input_max": umax,
            "roadwheel_at_u0_rad": d0,
            "roadwheel_min_rad": at_min["delta"],
            "roadwheel_max_rad": at_max["delta"],
            "steering_to_input": k_mean,
            "roadwheel_FL_at_umin_rad": at_min["delta_l"],
            "roadwheel_FR_at_umin_rad": at_min["delta_r"],
            "roadwheel_FL_at_umax_rad": at_max["delta_l"],
            "roadwheel_FR_at_umax_rad": at_max["delta_r"],
        }
        _log.info(
            f"steering_to_input = {k_mean:+.6f}  (mean of {len(slope_k)} mid-range samples)  "
            f"lock δ(u={umin:g})={at_min['delta']:+.4f}  δ(u={umax:g})={at_max['delta']:+.4f}"
        )

        tau = measure_steering_tau(vehicle, u_step=1.0, logger=_log)

        return {
            "steering": steering,
            "steering_to_input": k_mean,
            "samples": details,
            "settle": settle,
            **tau,
        }
    except Exception as exc:
        _log.warning(
            f"steering_to_input measure failed: {exc} — run steering_input_sweep.py"
        )
        return {
            "steering": steering,
            "steering_to_input": None,
            "samples": details,
            "settle": settle,
            **tau,
            "error": str(exc),
            "hint": "run steering_input_sweep.py",
        }
    finally:
        try:
            control_vehicle(vehicle, steering_input=0.0, parkingbrake=True)
        except Exception:
            pass
