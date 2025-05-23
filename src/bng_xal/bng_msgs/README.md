# BeamNG Messages (bng_msgs) ROS 2 Package

## Purpose

The `bng_msgs` package provides the custom ROS 2 message and service definitions required for the `beamng_xal` ROS 2 interface to BeamNG.tech. These definitions facilitate communication between various ROS 2 nodes and the BeamNG.tech simulation environment, enabling the exchange of data related to vehicle state, control commands, and simulation management.

This package is a foundational component for operating BeamNG.tech with ROS 2, as other ROS 2 nodes in the `beamng_xal` ecosystem (like `bng_controller` and `bng_simulator`) will use these message and service types for communication.

## Prerequisites

Before building and using this package, ensure you have the following installed:

*   **ROS 2:** A working installation of ROS 2 (e.g., Foxy, Humble).
*   **`rosidl_default_generators`:** This package is essential for generating language-specific code (like Python and C++) from the `.msg` and `.srv` definitions. It's a common ROS 2 dependency for message packages.
*   **`std_msgs` and `geometry_msgs`:** These are standard ROS 2 message packages that `bng_msgs` depends on for some of its message field types.

These dependencies are typically installed as part of a standard ROS 2 desktop installation.

## Build Instructions

The `bng_msgs` package is built using `colcon`, the standard ROS 2 build tool. As it's an `ament_cmake` package that generates message headers and code, it should be built before other packages that depend on its messages.

1.  **Source your ROS 2 environment:**
    ```bash
    source /opt/ros/<your_ros_distro>/setup.bash
    ```
2.  **Navigate to your ROS 2 workspace:**
    ```bash
    cd /path/to/your/ros2_ws
    ```
3.  **Build the package:**
    To build only `bng_msgs`:
    ```bash
    colcon build --packages-select bng_msgs
    ```
    To build all packages in your workspace (which will correctly build `bng_msgs` in order if other packages depend on it):
    ```bash
    colcon build
    ```
4.  **Source the workspace's local setup file:**
    After building, source the local setup file to make the messages and services available to ROS 2 tools and other nodes:
    ```bash
    source install/setup.bash
    ```

## Message and Service Definitions

This package primarily consists of `.msg` and `.srv` files, which define the structure of data exchanged between ROS 2 nodes.

### Messages

*   **`BasicStateMsg.msg`**
    *   **Purpose:** Likely contains fundamental vehicle state information such as position, orientation, velocity, and other basic telemetry.
*   **`GtStateMsg.msg`**
    *   **Purpose:** "Gt" may stand for Ground Truth. This message probably carries more detailed or simulation-specific state information, potentially including data not directly available from real-world sensors but useful for simulation and analysis.
*   **`HLCMsg.msg`**
    *   **Purpose:** "HLC" likely stands for High-Level Controller. This message is probably used for communication with, or commands from, a high-level vehicle control system (e.g., sending driving commands like target speed or steering angle).

### Services

*   **`ExecuteRequest.srv`**
    *   **Purpose:** A general-purpose service to request the execution of a specific action, command, or function within the BeamNG simulation or a connected ROS 2 node. The request likely contains details of the command, and the response would indicate success or failure.
*   **`StartLogger.srv`**
    *   **Purpose:** A service to initiate a data logging process. The request might specify parameters for logging, such as the data topics to record or the output file name.
*   **`StopLogger.srv`**
    *   **Purpose:** A service to stop an active data logging process that was previously started, likely via the `StartLogger` service.

---

*License: Apache-2.0 (as per package.xml)*
*Maintainer: user (as per package.xml)*
