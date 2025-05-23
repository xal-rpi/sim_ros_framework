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

---

*License: TODO (as per package.xml)*
*Maintainer: comev (as per package.xml)*
