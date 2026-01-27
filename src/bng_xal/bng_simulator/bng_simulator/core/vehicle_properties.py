"""
Implement additional functionalities for BeamNGpy
"""

from typing import Optional
from beamngpy import Vehicle
from logging import getLogger
from beamngpy.logging import LOGGER_ID

logger = getLogger(f"{LOGGER_ID}.vehicle_properties")


def set_gearbox_index(vehicle: Vehicle, gear_index: int):
    """
    Set the gear index of a vehicle.

    Args:
        vehicle: The vehicle to set the gear index of.
        index (int): The index to set the gear to.
    """
    # Vehicle root node
    veh_root = vehicle._root
    data = dict(type="SetGearboxIndex", gearIndex=gear_index)
    veh_root._send(data).ack("SetGearboxIndex")


def get_gearbox_info(vehicle: Vehicle):
    """
    Get the gearbox infos of a vehicle.

    Args:
        vehicle: The vehicle to get the gearbox infos of.
    """
    # Vehicle root node
    veh_root = vehicle._root
    data = dict(type="GetGearboxInfo")
    return veh_root._send(data).recv()["data"]


def set_ABS(vehicle: Vehicle, enabled: bool = False):
    """
    Enable or disable ABS system
    Args:
        vehicle: Target vehicle
        enabled: True to enable ABS, False to disable
    """
    veh_root = vehicle._root
    data = dict(type="SetABS", enabled=enabled)
    veh_root._send(data).ack("SetABS")


def get_ABS(vehicle: Vehicle) -> dict:
    """
    Get current ABS configuration
    Returns:
        dict: ABS configuration with keys:
            - hasAbs (bool): True if vehicle has ABS capability
    """
    veh_root = vehicle._root
    data = dict(type="GetABS")
    return veh_root._send(data).recv()["data"]


def set_ESC(vehicle: Vehicle, enabled: bool = False):
    """
    Enable or disable ESC system
    Args:
        vehicle: Target vehicle
        enabled: True to enable ESC, False to disable
    """
    veh_root = vehicle._root
    data = dict(type="SetESC", enabled=enabled)
    veh_root._send(data).ack("SetESC")


def get_ESC(vehicle: Vehicle) -> dict:
    """
    Get current ESC configuration
    Returns:
        dict: ESC configuration with keys:
            - enabled (bool): Current ESC activation state
            - hasESC (bool): True if vehicle has ESC capability
    """
    veh_root = vehicle._root
    data = dict(type="GetESC")
    return veh_root._send(data).recv()["data"]


def set_4WD_mode(vehicle: Vehicle, mode: str = "", range_mode: str = ""):
    """
    Set 4WD mode and/or range box mode
    Args:
        vehicle: Target vehicle
        mode: '2WD' or '4WD' (optional)
        range_mode: 'low' or 'high' (optional)
    """
    veh_root = vehicle._root
    data = dict(type="Set4wdMode", mode=mode, rangeMode=range_mode)
    veh_root._send(data).ack("Set4wdMode")


def get_4WD_mode(vehicle: Vehicle) -> dict:
    """
    Get current 4WD configuration
    Returns:
        dict: 4WD configuration with keys:
            - mode (str): Current drive mode ('2WD'/'4WD')
            - range (str): Current range box mode ('low'/'high')
            - is4wdCapable (bool): True if vehicle has 4WD capability
    """
    veh_root = vehicle._root
    data = dict(type="Get4wdMode")
    return veh_root._send(data).recv()["data"]


def lock_diff(vehicle: Vehicle, diff: str = "front", lock: bool = True):
    """
    Lock/unlock differential
    Args:
        vehicle: Target vehicle
        diff: 'front' or 'rear'
        lock: True to lock, False to unlock
    """
    assert diff in ["front", "rear"], "Invalid diff type"
    veh_root = vehicle._root
    data = dict(type="LockDiff", diff=diff, lock=lock)
    veh_root._send(data).ack("LockDiff")


def get_diff_lock_state(vehicle: Vehicle, diff: str = "front") -> dict:
    """
    Get differential lock state
    Args:
        vehicle: Target vehicle
        diff: 'front' or 'rear'
    Returns:
        dict: Differential state with keys:
            - locked (bool): Current lock status
            - mode (str): Current differential mode
    """
    veh_root = vehicle._root
    data = dict(type="GetDiffLockState", diff=diff)
    return veh_root._send(data).recv()["data"]


def get_vehicle_properties(vehicle: Vehicle, world_space: bool = False) -> dict:
    """
    Get comprehensive vehicle properties.

    Args:
        vehicle (Vehicle): The target vehicle.
        world_space (bool): True to retrieve properties in world space; False for local space.

    Returns:
        dict: Vehicle properties containing:
            - vLength (float): Vehicle length (meters).
            - vWidth (float): Vehicle width (meters).
            - vHeight (float): Vehicle height (meters).
            - estCogGlobal (dict): Estimated global center of gravity in FLU space {x, y, z}.
            - relCog (dict): Relative center of gravity based on the reference node.
            - posRef (dict): Position of the reference node.
            - estCogGlobalV2 (dict): Secondary estimation of global center of gravity {x, y, z}.
            - currPos (dict): Current vehicle position {x, y, z}.
            - cogGlobal (dict): Center of gravity in game coordinates {x, y, z}.
            - currDirection (dict): Current vehicle direction vector {x, y, z}.
            - wheelBase (float): Distance between front and rear axles (meters).
            - totalMass (float): Total vehicle mass (kg).
            - cogToFrontAxle (float): Distance from COG to front axle (meters).
            - cogToRearAxle (float): Distance from COG to rear axle (meters).
            - cogToLeftWheelAxle (float): Lateral distance from COG to left wheel axle (meters).
            - cogToRightWheelAxle (float): Lateral distance from COG to right wheel axle (meters).
            - vectorForward (dict): Forward direction vector in the vehicle frame {x, y, z}.
            - vectorUp (dict): Upward direction vector in the vehicle frame {x, y, z}.
            - vectorLeft (dict): Left direction vector in the vehicle frame {x, y, z}.
            - vectorForwardWS (dict): Forward direction vector in world space {x, y, z}.
            - vectorUpWS (dict): Upward direction vector in world space {x, y, z}.
            - vectorLeftWS (dict): Left direction vector in world space {x, y, z}.
            - wheel_fr (dict): Per-wheel data for the front right wheel (includes mass, position, inertia, radius, width).
            - wheel_fl (dict): Per-wheel data for the front left wheel (includes mass, position, inertia, radius, width).
            - wheel_rr (dict): Per-wheel data for the rear right wheel (includes mass, position, inertia, radius, width).
            - wheel_rl (dict): Per-wheel data for the rear left wheel (includes mass, position, inertia, radius, width).
            - vehInertia (dict): Inertia tensor components with keys: xx, yy, zz, xy, xz, yz.
    """
    veh_root = vehicle._root
    data = dict(type="GetVehicleProperties", worldSpace=world_space)
    return veh_root._send(data).recv()["data"]


def get_vehicle_principal_axis(vehicle: Vehicle) -> dict:
    """
    Get vehicle's principal axis information
    Args:
        vehicle: Target vehicle
    Returns:
        dict: Principal axis data with keys:
            - cogPosStatic (dict): Global COG position {x,y,z}
            - vectorForward (dict): Forward direction vector {x,y,z}
            - vectorUp (dict): Up direction vector {x,y,z}
            - vectorLeft (dict): Left direction vector {x,y,z}
    """
    veh_root = vehicle._root
    data = dict(type="GetVehiclePrincipalAxis")
    return veh_root._send(data).recv()["data"]


def get_powertrain_properties(vehicle: Vehicle) -> dict:
    """
    Get detailed powertrain configuration
    Returns:
        dict: Powertrain devices with nested structures containing:
            - type (str): Device type (gearbox, differential, etc.)
            - mode (str): Current operation mode
            - gearRatio (float): Current ratio (if applicable)
            - availableModes (list): Supported modes (if available)
            - diffTorqueSplit (float): Torque distribution (differentials)
    """
    veh_root = vehicle._root
    data = dict(type="GetPowertrainProperties")
    return veh_root._send(data).recv()["data"]


def get_controller_infos(vehicle: Vehicle) -> dict:
    """
    Get list of active vehicle controllers
    Returns:
        dict: Mapping of controller names to their type strings
    """
    veh_root = vehicle._root
    data = dict(type="GetControllerInfos")
    return veh_root._send(data).recv()["data"]


def get_engine_infos(vehicle: Vehicle) -> dict:
    """
    Get engine-related information for a vehicle.

    Args:
        vehicle (Vehicle): The vehicle whose engine data is to be retrieved.

    Returns:
        dict: Engine information with the following keys:
            idleRPM (float): Engine idle RPM.
            maxRPM (float): Maximum engine RPM.
            fuelVolume (float): Current fuel volume.
            fuelCapacity (float): Total fuel capacity.
            turboBoostMax (float): Maximum turbo boost pressure in PSI, or -1 if not available.
            superchargerBoostMax (float): Maximum supercharger boost pressure in PSI, or -1 if not available.
    """
    veh_root = vehicle._root
    data = dict(type="EngineInfos")
    return veh_root._send(data).recv()["data"]


def stop_safety_features(vehicle: Vehicle) -> dict:
    """
    Stop all safety features on the vehicle.

    This command disables safety features (e.g., ABS, ESC) and unloads
    safety controllers (such as drivingDynamics controllers).

    Args:
        vehicle (Vehicle): The target vehicle whose safety features should be stopped.

    Returns:
        dict: A dictionary mapping the names of the safety controllers that were stopped
              to their corresponding type strings.
    """
    veh_root = vehicle._root
    data = dict(type="StopSafetyFeatures")
    return veh_root._send(data).recv()["data"]


def disable_safety_features(
    vehicle: Vehicle,
    abs: bool = True,
    esc: bool = True,
    stop_controllers: bool = True,
) -> dict:
    """Disable vehicle safety features in one call.

    This is a convenience wrapper used by higher-level tooling.

    Args:
        vehicle: Target vehicle.
        abs: If True, disable ABS (if supported).
        esc: If True, disable ESC (if supported).
        stop_controllers: If True, also call StopSafetyFeatures to unload safety controllers.

    Returns:
        dict: A summary of actions taken and best-effort status payloads.
    """
    out = {
        "requested": {"abs": bool(abs), "esc": bool(esc), "stop_controllers": bool(stop_controllers)},
        "abs": None,
        "esc": None,
        "stopped_controllers": None,
        "errors": [],
    }

    if abs:
        try:
            set_ABS(vehicle, enabled=False)
            out["abs"] = get_ABS(vehicle)
        except Exception as e:
            out["errors"].append(f"ABS disable failed: {e}")

    if esc:
        try:
            set_ESC(vehicle, enabled=False)
            out["esc"] = get_ESC(vehicle)
        except Exception as e:
            out["errors"].append(f"ESC disable failed: {e}")

    if stop_controllers:
        try:
            out["stopped_controllers"] = stop_safety_features(vehicle)
        except Exception as e:
            out["errors"].append(f"StopSafetyFeatures failed: {e}")

    return out


def control_vehicle(
    vehicle: Vehicle,
    filter: str = "Direct",
    throttle: Optional[float] = None,
    brake: Optional[float] = None,
    clutch: Optional[float] = None,
    steering: Optional[float] = None,
    parkingbrake: Optional[bool] = None,
    gear: Optional[int] = None,
):
    """
    Handle setting inputs for the vehicle.

    This command sends the provided inputs to the vehicle and acknowledges the request.

    Args:
        vehicle (Vehicle): The target vehicle.
        inputs (dict):
            filter: {Keyboard, Gamepad, Direct, KeyboardDrift,FILTER_AI}
            throttle,
            brake,
            clutch,
            steering,
            parkingbrake,
            gear.

    Returns:
        None
    """
    veh_root = vehicle._root
    dict_inputs = {}
    if throttle is not None:
        dict_inputs["throttle"] = throttle
    if brake is not None:
        dict_inputs["brake"] = brake
    if clutch is not None:
        dict_inputs["clutch"] = clutch
    if steering is not None:
        dict_inputs["steering"] = steering
    if parkingbrake is not None:
        dict_inputs["parkingbrake"] = parkingbrake
    if gear is not None:
        dict_inputs["gear"] = gear
    dict_inputs["filter"] = filter
    data = dict(type="SetInputs", **dict_inputs)
    veh_root._send(data).ack("Controlled")
