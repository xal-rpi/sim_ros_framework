# BeamNG Simulator (bng_simulator) ROS 2 Package

## Purpose

The `bng_simulator` package is the core ROS 2 component for interfacing with the BeamNG.tech simulator. It provides the necessary tools to launch, manage, and interact with BeamNG.tech simulation instances from a ROS 2 environment. This package allows users to define simulation scenarios, spawn and control vehicles, and integrate BeamNG.tech's physics and environments into broader ROS 2 based robotics and autonomous systems applications.

## Prerequisites

Before using this package, ensure you have the following installed:

*   **ROS 2:** A working installation of ROS 2 (e.g., Foxy, Humble).
*   **BeamNG.tech:** The BeamNG.tech simulator must be installed. This package does not include the simulator itself.
*   **Python 3:** Along with common libraries (often handled by ROS 2 dependencies or `rosdep`).
*   **Dependent ROS 2 Packages:**
    *   `bng_msgs`: Provides the custom ROS 2 message and service definitions used by `bng_simulator`.
    *   `rclpy`: The ROS 2 Python client library.
    *   `std_msgs`: Standard ROS 2 messages.
    *   `geometry_msgs`: ROS 2 messages for geometric primitives.

## Build Instructions

The `bng_simulator` package is built using `colcon`, the standard ROS 2 build tool.

1.  **Source your ROS 2 environment:**
    ```bash
    source /opt/ros/<your_ros_distro>/setup.bash
    ```
2.  **Navigate to your ROS 2 workspace:**
    ```bash
    cd /path/to/your/ros2_ws
    ```
3.  **Build the package:**
    It's recommended to build `bng_msgs` first or ensure it's available in your environment.
    ```bash
    colcon build --packages-select bng_simulator
    ```
    To build all packages in your workspace:
    ```bash
    colcon build
    ```
4.  **Source the workspace's local setup file:**
    After building, source the local setup file to make the package's nodes and launch files available:
    ```bash
    source install/setup.bash
    ```

## Launching the Simulator

The primary way to start the BeamNG.tech simulation interface is by using the provided launch file:

```bash
ros2 launch bng_simulator simulator.launch.py
```

This launch file starts the `sim_manager_node`, which is the main node responsible for managing the simulation.

### Key Launch Arguments

*   **`config_path`**:
    *   Description: Path to the main simulation scenario configuration YAML file. This file defines the level to load, vehicles to spawn, and other simulation parameters.
    *   Default: `$(ros2 pkg prefix bng_simulator)/share/bng_simulator/config/scenarios/basic_scenario.yaml`
    *   Example: `ros2 launch bng_simulator simulator.launch.py config_path:=/path/to/your/custom_scenario.yaml`
*   **`log_level`**:
    *   Description: Sets the logging level for the `sim_manager_node` (e.g., `DEBUG`, `INFO`, `WARN`, `ERROR`, `CRITICAL`).
    *   Default: `INFO`
    *   Example: `ros2 launch bng_simulator simulator.launch.py log_level:=DEBUG`

## Architecture Overview

The `bng_simulator` package has several key components:

*   **Simulation Manager (`sim_manager_node` / `core/simulation_manager.py`):**
    The central component that manages the connection to the BeamNG.tech simulator, loads scenarios, and oversees the overall simulation lifecycle. It handles requests for starting, stopping, and pausing the simulation.
*   **Vehicle Manager (`vehicle/manager.py`):**
    Responsible for spawning, despawning, and managing individual vehicles within the simulation. It interfaces with the `Simulation Manager` and handles vehicle-specific configurations and sensors.
*   **Vehicle Properties (`core/vehicle_properties.py`):**
    Defines and manages properties and configurations associated with vehicles.
*   **Controllers and Sensors (`vehicle/controllers/`, `vehicle/sensors/`):**
    Modules related to vehicle control mechanisms and sensor data acquisition from the simulation.

## Configuration (`config/` directory)

The `config/` directory contains YAML files and other configurations that define how the simulation behaves. You can customize these or create your own. The installed path for these files will be `install/bng_simulator/share/bng_simulator/config/`.

*   **`scenarios/`**:
    *   Contains YAML files defining different simulation scenarios (e.g., `basic_scenario.yaml`, `path_follow.yaml`, `nn_scenario.yaml`).
    *   These files specify parameters like the BeamNG.tech level to load, vehicle models, spawn locations, and potentially other environmental factors.
    *   To use a custom scenario, modify an existing file or create a new one and pass its path via the `config_path` launch argument.
*   **`vehicles/`**:
    *   Contains YAML files for specific vehicle configurations (e.g., `bolide_350.yaml`, `utv_wild.yaml`). These define vehicle models, parts, colors, and sensor setups.
    *   These are typically referenced within scenario configuration files.
*   **`plotjuggler/`**:
    *   Contains layout files (e.g., `basic_layout.xml`) for use with PlotJuggler, a ROS 2 tool for visualizing time-series data.
*   **`rviz/`**:
    *   Contains configuration files (e.g., `path.rviz`) for RViz, the ROS 2 3D visualization tool. These can be used to display vehicle paths, sensor data, etc.

## Examples (`examples/` directory)

The `examples/` directory provides a wealth of practical demonstrations and tools for interacting with the simulator:

*   **Jupyter Notebooks:**
    *   Interactive examples covering various functionalities like:
        *   `check_service_requests.ipynb`: Demonstrates how to call and test services.
        *   `create_vehicle_config.ipynb`: Shows how to programmatically create vehicle configuration files.
        *   `steering_mapping.ipynb`, `torque_mapping.ipynb`, `torque_model_training*.ipynb`: Examples related to vehicle dynamics, control, and model training.
        *   `step_response_check.ipynb`: For analyzing vehicle responses.
*   **Python Scripts:**
    *   `data_utils.py`, `logging_utils.py`: Utility scripts for handling data and logging.
    *   `low_level_model/`: A sub-directory containing a more complex example for training a low-level vehicle engine model, including data loading, environment setup, and RL training scripts.

These examples serve as excellent starting points for developing custom applications and understanding the capabilities of the `bng_simulator`.

## Important Scripts (`bng_simulator/bng_simulator/scripts/`)

While the primary interaction is through the `sim_manager_node` via the launch file, the following scripts are also present:

*   **`run_simulator.py`**: This script is likely the underlying Python entry point that `sim_manager_node` (which is an executable generated by ROS 2 from `setup.py`) uses. It might also be runnable directly for some use cases.
*   **`sim_shell.py`**: Suggests an interactive command-line shell for more direct interaction with the simulation, potentially for debugging or manual control.
*   **`start_logs.py`**: A utility script related to initiating data logging.
*   **`find_test_ema_gtstate.py`**: Appears to be a specific test or utility script.

---

*License: Apache-2.0 (as per package.xml)*
*Maintainer: user (as per package.xml)*
