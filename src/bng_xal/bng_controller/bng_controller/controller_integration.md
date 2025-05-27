# BeamNG Controller (bng_controller) ROS 2 Package

## Purpose

The `bng_controller` is a ROS 2 package designed to provide high-level control for vehicles within the BeamNG.tech simulation environment. It allows users to send control commands, manage vehicle state, and follow predefined or generated paths. This package acts as an interface between ROS 2 applications and the BeamNG simulation, facilitating robotics research, autonomous driving development, and complex scenario execution.

## Prerequisites

Before using this package, ensure you have the following installed:

*   **ROS 2:** A working installation of ROS 2 (e.g., Foxy, Humble). The package is built using `ament_python`.
*   **BeamNG.tech:** The BeamNG.tech simulator must be installed and running.
*   **BeamNG ROS 2 Packages:**
    *   `bng_msgs`: Provides the ROS 2 message definitions necessary for communication with the BeamNG simulation.
    *   `bng_simulator`: The core BeamNG ROS 2 integration package that this controller depends on.
*   **Python Dependencies:** Specific Python libraries might be required by the controller scripts (typically managed by `rosdep` or listed in `setup.py`).

## Build Instructions

To build the `bng_controller` package, navigate to your ROS 2 workspace and use `colcon`:

1.  **Source your ROS 2 environment:**
    ```bash
    source /opt/ros/<your_ros_distro>/setup.bash
    ```
2.  **Navigate to your ROS 2 workspace:**
    ```bash
    cd /path/to/your/ros2_ws
    ```
3.  **Build the package:**
    ```bash
    colcon build --packages-select bng_controller
    ```
    If you want to build all packages in your workspace:
    ```bash
    colcon build
    ```
4.  **Source the workspace's local setup file:**
    ```bash
    source install/setup.bash
    ```

## Usage Instructions

The primary way to run the `bng_controller` is by using its provided launch file.

### Launching the Controller

To launch the main controller nodes:

```bash
ros2 launch bng_controller controller.launch.py
```

This will start:
*   `controller_interface`: Node responsible for direct communication with the BeamNG simulation. (Executable: `run_controller`)
*   `high_level_controller`: Node implementing the high-level control logic.

### Launch Arguments

The `controller.launch.py` file accepts several arguments:

*   `config_path`: Path to the BeamNG simulation YAML configuration file. Defaults to a basic scenario configuration from the `bng_simulator` package.
    *   Example: `ros2 launch bng_controller controller.launch.py config_path:=/path/to/your/custom_scenario.yaml`
*   `log_level`: Sets the log level for the nodes (e.g., `INFO`, `DEBUG`).
    *   Example: `ros2 launch bng_controller controller.launch.py log_level:=DEBUG`
*   `enable_path_viz`: Set to `true` to launch the path visualization node. This requires a `path_file_viz` argument to be set as well.
    *   Example: `ros2 launch bng_controller controller.launch.py enable_path_viz:=true path_file_viz:=/path/to/your/paths/your_path.csv`

### Path Visualization

An optional path visualization node (`path_viz`) can be launched to display a specified path in RViz or another visualization tool.

To launch the path visualization node along with the controllers:
```bash
ros2 launch bng_controller controller.launch.py enable_path_viz:=true path_file_viz:=$(ros2 pkg prefix bng_controller)/share/bng_controller/resource/paths/your_chosen_path.csv
```
Replace `your_chosen_path.csv` with one of the available pre-defined paths or a custom one.

## Paths

The controller can use paths for navigation. Paths are typically CSV files.

### Pre-defined Paths

Several pre-defined paths are available in the `resource/paths/` directory of the installed package. You can find them in `install/bng_controller/share/bng_controller/resource/paths/` after building, or directly in the source at `src/bng_xal/bng_controller/resource/paths/`:

*   `circle_R100_N200.csv`
*   `circle_R30_N50.csv`
*   `deer.csv`
*   `loop_700_500_30_50_1-2_20.csv`
*   `octogon.csv`

### Path Generation Scripts

Scripts to generate custom paths are located in the `bng_controller/scripts/` directory (e.g., `bng_controller/bng_controller/scripts/` in the source tree):

*   **`generate_circle_path.py`**: Generates a circular path.
*   **`generate_path.py`**: A more general script for path generation (its specific capabilities would need to be explored by examining the script).

These scripts can be run as standalone Python scripts to produce CSV path files that can then be used by the controller.

## Nodes

*   **`run_controller` (controller_interface):** Low-level interface to BeamNG.
*   **`high_level_controller`:** Implements advanced control logic (e.g., path following).
*   **`path_viz`:** Visualizes paths.

# Overview of the High-Level Control System

The `bng_controller` package provides a framework for implementing and running custom high-level control policies for vehicles within the BeamNG.tech driving simulator. It handles the communication with the simulator and allows you to focus on designing the core logic of your controller.

The system operates within a ROS 2 environment, where different parts of the architecture (like simulation management and the high-level controller itself) run as separate nodes. However, for the purpose of designing a new control policy, deep knowledge of ROS 2 is not strictly necessary as the `high_level_controller` node abstracts much of this complexity.

## Available Inputs to a Control Policy

When you implement a new control policy, it will receive several inputs at each time step. These inputs provide the necessary information about the vehicle's state and the simulation environment.

The primary input is a Python dictionary named `latest_sensor_data`. This dictionary is populated with data received from the BeamNG simulator via UDP. The exact contents of this dictionary can vary based on the BeamNG vehicle's sensor configuration, but typically include:

*   **`simtime` (float):** The current simulation time in seconds. This is often used for logging, calculations involving time, or time-dependent behaviors.
*   **`position` (dict):** The vehicle's current position in the world coordinate system.
    *   `'x'` (float): X-coordinate.
    *   `'y'` (float): Y-coordinate.
    *   `'z'` (float): Z-coordinate.
*   **`direction` (dict):** The vehicle's current orientation, represented by a forward vector. This indicates the direction the front of the vehicle is pointing.
    *   `'x'` (float): X-component of the direction vector.
    *   `'y'` (float): Y-component of the direction vector.
    *   `'z'` (float): Z-component of the direction vector.
*   **`velocity` (dict):** The vehicle's current velocity vector in world coordinates.
    *   `'x'` (float): X-component of velocity.
    *   `'y'` (float): Y-component of velocity.
    *   `'z'` (float): Z-component of velocity.

*Note: The BeamNG simulator is highly configurable. If your control policy requires additional sensor information (e.g., wheel speeds, G-forces, Lidar data), you will need to configure the active BeamNG vehicle scenario to output this data. The `bng_simulator` package and BeamNG documentation provide details on how to customize sensor configurations.*

In addition to `latest_sensor_data`, your control policy function will also receive two other arguments:

*   **`control_rate` (float):** The rate at which your control function is being called, in seconds (e.g., if the controller runs at 20 Hz, `control_rate` will be `0.05`). This is useful for predicting future states or for time-dependent calculations within your policy.
*   **`max_latency` (float):** The current maximum observed round-trip latency (in seconds) for messages between the controller and the simulator. This can be used to adjust the `time` field in your control commands to compensate for communication delays.

## Expected Outputs from a Control Policy

Your control policy function must return a Python dictionary containing the desired control commands. This dictionary is then serialized into JSON and sent to the Low-Level Controller (LLC) running in the BeamNG simulator via UDP.

There is one **required key** that must always be present in the returned dictionary:

*   **`time` (float):** This specifies the target simulation time at which the control commands should be applied by the simulator. Accurate timing is crucial for stable control. A common way to calculate this is:
    `target_sim_time = current_sim_time_from_sensor_data + control_rate + latency_compensation`
    where `latency_compensation` can be derived from `max_latency` (e.g., `max_latency + small_buffer`). The `high_level_controller.py` provides an example of this: `simt + control_rate + min(max_latency + 0.005, 0.1)`.

Besides `time`, you can include various control command keys. The specific keys and their effects depend on how the Low-Level Controller in your BeamNG vehicle scenario is configured to interpret them. Common examples (as used by the existing controllers) include:

*   **`road_wheel_angle` (float):** The desired angle of the road wheels in radians. Positive values typically mean steering to the right, and negative values to the left, but this can depend on the vehicle's setup.
*   **`engine_torque` (float):** The desired net engine torque in Newton-meters (Nm). Positive values request acceleration. Some LLCs might interpret negative values as engine braking, while others might require a separate braking command. The example `compute_control_multi_test` in C splits a raw torque into `wheel_torque` (always positive) and `brake_torque`.
*   **`wheel_torque` (float):** Often used for positive torque to the drive wheels (Nm).
*   **`brake_torque` (float):** The desired brake torque in Newton-meters (Nm), applied to the wheels. This is typically a positive value.

*Important Considerations:*
*   **Low-Level Controller (LLC) Dependence:** The set of recognized command keys (e.g., `engine_torque` vs. separate `throttle` and `brake` commands, or combined `wheel_torque`) and their precise interpretation is defined by the LLC running on the vehicle within the BeamNG simulation. You may need to consult or modify the LLC configuration within your BeamNG vehicle's files if you need to send different types of commands.
*   **Units:** Ensure your commands use the units expected by the LLC (e.g., radians for angles, Nm for torques).
*   **Command Limits:** The LLC or the vehicle model itself will likely have limits on achievable torques, steering angles, etc. Your policy should ideally operate within realistic bounds.

## Implementing and Integrating a New Control Policy

Integrating your custom control policy into the `bng_controller` framework involves implementing a function with a specific signature and then configuring the system to use it.

### 1. Implementing Your Policy Function

You can implement your policy in Python or C/C++.

**a) Python Policies:**

*   **Location:** You have a few options for placing your Python policy code:
    *   You can create a new Python file (e.g., `my_custom_policies.py`) and place it in the `core` folder.
    *   You can also use an absolute path to any Python file on your system when configuring the controller.
*   **Function Signature:** Your function must adhere to the following signature:
    ```python
    def my_new_policy_name(latest_sensor_data: dict, control_rate: float, max_latency: float) -> dict:
        # Your control logic here
        # ...
        # Process latest_sensor_data
        # Calculate control_commands (steering, torque, etc.)
        # Calculate target_time for the command
        # ...
        control_output = {
            "time": target_time,
            "road_wheel_angle": calculated_steering,
            # ... other commands
        }
        return control_output
    ```
    Replace `my_new_policy_name` with a descriptive name for your policy.

**b) C/C++ Policies:**

*   **Location:** Add your new C/C++ policy as a new `.c` file (e.g., `my_custom_c_policy.c`) within the `src/bng_xal/bng_controller/bng_controller/core/` directory.
*   **Implementation:**
    *   Write your control logic in C/C++. You will need to handle Python object conversion for the input dictionary (`latest_sensor_data`) and for the output dictionary.
    *   Each C file should be structured as a self-contained Python extension module. This means it needs to include `<Python.h>`, define its methods, and have a Python module initialization function (e.g., `PyMODINIT_FUNC PyInit_my_custom_c_policy(void)` if your filename is `my_custom_c_policy.c`).
    *   Refer to the existing `test_patterns.c` as an example of how to structure such a file, parse input `PyObject` arguments, and create a `PyDict` for the return value.
*   **Python Bindings (within your C file):**
    1.  Inside your C file, define a `PyMethodDef` array (e.g., `MyCustomCPolicyMethods[]`) listing the functions from this file that you want to expose to Python. For example:
        ```c
        static PyMethodDef MyCustomCPolicyMethods[] = {
            {"my_new_c_policy_name", my_c_function_ptr, METH_VARARGS, "Description of your C policy."},
            {NULL, NULL, 0, NULL} // Sentinel
        };
        ```
    2.  Ensure your C function (e.g., `my_c_function_ptr`) has the standard `(PyObject *self, PyObject *args)` signature and uses `PyArg_ParseTuple` to get the `sensor_data` PyObject, `control_rate`, and `max_latency`.
    3.  Your module initialization function (e.g., `PyInit_my_custom_c_policy`) will use this `PyMethodDef` array to create the module object.
*   **Compilation:** All `.c` files placed in the `src/bng_xal/bng_controller/bng_controller/core/` directory are automatically compiled into separate Python extension modules (e.g., `my_custom_c_policy.c` becomes importable as `bng_controller.core.my_custom_c_policy`) as part of the standard ROS 2 build process (e.g., `colcon build`). Any changes to your C code require recompilation.

### 2. Configuring the System to Use Your Policy

Once your policy function is implemented (and compiled, if in C/C++), you need to tell the `high_level_controller` node to use it.

*   **Configuration File:** This is done by setting the `control_fn` parameter in your main YAML configuration file (the one passed to the `controller.launch.py` script via the `config_path` argument).
*   **Setting `control_fn`:** The `control_fn` parameter uses a URI-like scheme to specify the source and name of your control function:

    *   **For a C or Python policy loaded from the `core` folder:**
        (e.g., `my_new_policy_name` in `bng_controller/core/my_module`)
        ```yaml
        high_level_controller:
          # ... other HLC settings ...
          control_fn: "core://my_module:my_new_policy_name"
          control_rate: 0.05 # Example: 20 Hz
        ```
        This scheme (`core://<module.name>:<function_name>`) is used for any function in a Python or C module that is inside the `core` directory.

    *   **For a policy loaded directly from a file:**
        (e.g., `my_new_policy_name` in a file `/opt/user_scripts/my_policy_file`)
        ```yaml
        high_level_controller:
          # ... other HLC settings ...
          control_fn: "file:///opt/user_scripts/my_policy_file:my_new_policy_name"
          control_rate: 0.05 # Example: 20 Hz
        ```
        The `file://<absolute_or_relative_path_to_file>:<function_name>` scheme loads a function directly from a specified Python module file. Absolute paths are recommended for clarity. If using relative paths, they are typically resolved from the current working directory of the `high_level_controller` node (usually the ROS2 `install` folder).

The `high_level_controller` node (`bng_controller/bng_controller/high_level_controller.py`) parses this URI at startup, loads the specified function, and calls it at the configured `control_rate`, passing the necessary sensor data and timing information.

## Examples of Existing Controllers

To further illustrate how control policies are implemented, we can look at two existing examples within the `bng_controller` package.

### 1. Path Following (Pure Pursuit) - Python Example

*   **Function:** `compute_control_follow`
*   **File:** `bng_controller/core/controller_core_py.py`
*   **Type:** Python

**Purpose:**
This controller makes the vehicle follow a predefined path, specified as a series of waypoints in a CSV file. It uses the Pure Pursuit algorithm.

**Inputs Used:**
*   From `latest_sensor_data`:
    *   `position`: Current 'x' and 'y' coordinates of the vehicle.
    *   `direction`: Current 'x' and 'y' components of the vehicle's forward vector (to determine heading).
    *   `simtime`: Used for calculating the output `time` command.
*   External Data:
    *   Waypoints: Loaded from a CSV file specified in the main YAML configuration (`high_level_controller.path_file`). These are pre-processed into segments and cumulative lengths.
*   Parameters from Config:
    *   `lookahead_distance`: The lookahead distance for the Pure Pursuit algorithm.
    *   `wheelbase`: Vehicle's wheelbase.
    *   `max_steer_rad`: Maximum steering angle.

**Outputs Produced:**
*   `road_wheel_angle`: The calculated steering angle to follow the path.
*   `time`: The target simulation time for applying the command.

**Core Logic:**
1.  Finds the closest point on the pre-defined path to the vehicle's current position.
2.  Determines a "goal point" on the path by looking ahead a certain `lookahead_distance` from this closest point.
3.  Calculates the steering angle required to direct the vehicle towards this goal point, based on the Pure Pursuit geometry.
4.  Clamps the steering angle to `max_steer_rad`.
5.  Packages the steering command along with the calculated `time`.

### 2. Test Waveform Generator - C Example

*   **Function:** `compute_control_multi_test`
*   **File:** `bng_controller/core/test_patterns.c` (with Python bindings in the same file)
*   **Type:** C (callable from Python)

**Purpose:**
This controller is primarily for testing the control pipeline and vehicle response. It generates a sequence of predefined torque and steering commands that vary over time.

**Inputs Used:**
*   From `latest_sensor_data`:
    *   `simtime`: The current simulation time is the primary driver for the waveform generation.
    *   `velocity['x']`: The vehicle's longitudinal speed is used to scale the maximum available torque (torque fades with speed).
*   Hardcoded Parameters:
    *   `base_maxT`: Base maximum torque.
    *   `max_speed`: Speed at which torque starts to fade.
    *   Waveform definitions (durations, shapes like steps, ramps, sines, chirps).

**Outputs Produced:**
*   `wheel_torque`: Calculated positive torque for acceleration.
*   `brake_torque`: Calculated positive torque for braking.
*   `road_wheel_angle`: Currently set to `0.0` in this specific example, but could be part of a waveform.
*   `time`: The target simulation time for applying the command.

**Core Logic:**
1.  Calculates an available torque that decreases as vehicle speed (`velx`) approaches a `max_speed`.
2.  Uses the `simtime` modulo 60 seconds to cycle through different phases of a test waveform:
    *   0-15s: Step torque.
    *   15-30s: Ramp torque.
    *   30-45s: Sine wave torque.
    *   45-60s: Chirp signal torque.
3.  Clamps the raw calculated torque to the available torque.
4.  Splits the raw torque into `wheel_torque` (if positive) and `brake_torque` (if negative, converted to positive).
5.  Packages these commands along with `road_wheel_angle` (fixed at 0) and the calculated `time`.

## Summary of Key Files for Controller Development

When developing or integrating a new high-level control policy, you will primarily interact with the following files:

*   **`src/bng_xal/bng_controller/bng_controller/high_level_controller.py`:**
    *   This is the main ROS 2 node that orchestrates the high-level control loop.
    *   It handles receiving sensor data, parsing the `control_fn` URI to load your chosen policy, calling your policy function at the specified rate, and sending commands back to the simulator.
    *   You typically won't need to modify this file unless you're changing the fundamental data flow, URI parsing, or communication mechanisms.

*   **`src/bng_xal/bng_controller/bng_controller/core/`:**
    *   If you choose to write your policy in the `core` folder, you should add your code as new `.c` or `.py` file within this directory (e.g., `my_custom_c_policy.c`).
    *   Each file must be structured as a self-contained Python extension module (see implementation details in the section above).
    *   All `.c` files in this directory are automatically compiled by `setup.py` into individual Python modules (e.g., `my_custom_c_policy.c` becomes `bng_controller.core.my_custom_c_policy`).
    *   The existing `test_patterns.c` serves as an example.

*   **Your Simulation's YAML Configuration File (e.g., `config/your_scenario.yaml`):**
    *   This file is crucial for configuring the `high_level_controller`.
    *   You'll edit this to:
        *   Select your active control policy using the `control_fn` parameter with the new URI schemes (e.g., `"core://my_module:my_function"`, `"file:///path/to/my_script.py:my_function"`).
        *   Set the `control_rate`.
        *   Provide any parameters specific to your policy (which your policy would then read from the config).
        *   Define UDP communication ports if they differ from defaults.

*   **`src/bng_xal/bng_controller/launch/controller.launch.py`:**
    *   This ROS 2 launch file is used to start the `controller_interface` and `high_level_controller` nodes.
    *   It's where the path to your YAML configuration file is typically passed as an argument (`config_path`).
    *   You might modify this if you need to change node names, add parameters at launch time, or integrate other nodes.

*   **`src/bng_xal/bng_controller/setup.py`:**
    *   The build script for the package.
    *   It's now configured to automatically find and compile all `.c` files in the `bng_controller/bng_controller/core/` directory into Python extension modules. You generally won't need to edit this file unless you're changing build procedures or package dependencies.
---

*License: TODO (as per package.xml)*
*Maintainer: comejv (as per package.xml)*
