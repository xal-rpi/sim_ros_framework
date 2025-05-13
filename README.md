# BeamNG-ROS2 Bridge: Autonomous Vehicle Simulation Framework

A comprehensive framework connecting ROS2 with the BeamNG vehicle simulator, enabling high-fidelity physics-based simulation for autonomous driving research and development.

## Features

- **High-fidelity vehicle simulation** with BeamNG's realistic physics engine
- **Advanced sensor suite** including ground truth state, IMU, GPS, and more
- **Dual-level controller architecture** (high-level and low-level)
- **ROS2 integration** with custom messages, services, and publishers
- **YAML-based configuration** for easy scenario and vehicle setup
- **Interactive shell** for real-time simulation control
- **Data logging and replay** capabilities for experiment analysis

## System Architecture

![System architecture](./figures/all_flow_chart.png)

The system consists of these key components:

| Component | Description |
|-----------|-------------|
| **SimulationManager** | Core component managing BeamNG instances, scenarios, and vehicles |
| **VehicleManager** | Handles individual vehicles and their configurations |
| **Sensors** | Various sensor types for vehicle state and environment perception |
| **Controllers** | Both low-level actuator control and high-level decision making |
| **ROS2 Interface** | Bridge between simulation and ROS2 ecosystem |

## Prerequisites

Without Nix:
- ROS2 (Humble or newer)
- BeamNG.tech simulator
- Python 3.8+
- Operating System:
  - Windows with WSL2, or
  - Ubuntu 24.04 LTS (beta support for BeamNG.tech)

With Nix:
- Working Nix installation
- Flakes enabled

## Installation

> [!NOTE]
> Step 2 assumes you have a working ROS2 environment.
> The provided flake.nix will install ROS2 and dependencies, allowing you to skip step 2.
> Simply run `nix develop` to install the ROS2 environment and all dependencies.

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/xal-rpi/sim_ros_framework
   ```

2. **Install dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   colcon build
   ```

4. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Configuration

### BeamNG Setup

1. Ensure BeamNG.tech is installed and configured according to the BeamNG documentation
2. Set up network communication:
   - For WSL2, find the correct IP address and update the IP in your scenario configuration files:
     ```bash
     ip route show | grep -i default | awk '{ print $3}'
     ```
   - On linux using the default `127.0.0.1` config should work.

### YAML Configuration Structure

Simulation scenarios and vehicles are configured via YAML files located in the `config` directory:

```yaml
# Example configuration snippet
beamng:
  host: 172.26.32.1
  port: 64256

scenario:
  level: smallgrid
  name: basic

vehicles:
  ego:
    model: utv
    sensors:
      gtstate:
        type: GtState
        gfx_update_time: 0.15
        physics_update_time: 0.005
        num_physics_steps_for_gfx_save: 1
    controllers:
      LowLevelController: # The python class used
        type: default # The lua controller used
        control_rate: 0.1
        listen_ip: 127.0.0.1
        listen_port: 64257
        send_ip: 127.0.0.1
        send_port: 64258
        gt_state_name: gtstate
        calibration:
          steeringP: 1.2
          throttleP: 1.0
          brakeP: 1.0
```

## Usage

### Launch Commands

1. **Start only the simulator with sensors:**
   ```bash
   ros2 launch bng_simulator simulator.launch.py
   ```

2. **Launch with controller:**
   ```bash
   ros2 launch bng_controller controller.launch.py
   ```

Available options :
- Custom configuration: `config_path:=/path/to/config.yaml `
- Log level: `log_level:={FATAL,ERROR,WARN,INFO,DEBUG,FULL}` 

### Launch File Parameters

#### Simulator Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `config_path` | `[pkg_share]/config/basic_scenario.yaml` | Path to the simulation configuration file |
| `log_level` | `INFO` | Logging level (FULL, DEBUG, INFO, WARNING, ERROR, FATAl). FULL also shows the debug info of external libraries such as rclpy and beamngpy|

#### Controller Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `config_path` | `[pkg_share]/config/basic_scenario.yaml` | Path to the simulation configuration file |
| `log_level` | `INFO` | Logging level (FULL, DEBUG, INFO, WARNING, ERROR, FATAl). FULL also shows the debug info of external libraries such as rclpy and beamngpy|

### Interactive Shell

Access the interactive simulation shell for direct control:

```bash
ros2 run bng_simulator sim_shell
```

**Available commands:**
- `vehicles` - List available vehicles
- `teleport vehicle_name=ego pos=[0,0,0] yaw_angle=90` - Teleport a vehicle
- `control vehicle_name=ego steering=0.5 throttle=0.7 brake=0` - Send control inputs
- `logger start /path/to/logs` - Start data logging
- `exec get_vehicle_properties vehicle_name=ego` - Execute custom commands

### ROS2 Services

Interact with the simulator using ROS2 services:

```bash
# Execute a command
ros2 service call /execute_request bng_msgs/srv/ExecuteRequest "{function_name: 'teleport_vehicle', arguments: 'vehicle_name: ego\npos: [0, 0, 0]\nyaw_angle: 90'}"

# Start logging
ros2 service call /start_logger bng_msgs/srv/StartLogger "{save_location: '/tmp/logs', max_queue_size: 1000, flush_interval: 0.5}"
```

## Troubleshooting

### Common Issues

1. **BeamNG Focus Issue**
   **Problem:** BeamNG.tech requires focus when managing scenarios
   **Solution:** Ensure the BeamNG window is focused, not minimized

2. **IP Configuration**
   **Problem:** Incorrect IP address prevents communication
   **Solution:** Verify the IP in scenario config matches WSL2 IP

3. **Vehicle Control Instability**
   **Problem:** Vehicles may behave erratically after teleportation
   **Solution:** Reset vehicle state with `teleport vehicle_name=ego reset=true`

4. **Sensor Data Missing**
   **Problem:** Sensors not publishing data
   **Solution:** Check sensor configuration and poll rates

5. **Request not handled by BNG:**
   **Problem:** The controller crashes with `The request was not handled by BeamNG.tech` error after having hot reloaded the mod
   **Solution:** Restart BNG

6. **Torque target not applied properly:**
   **Problem:** The reported torque is different from the target torque
   **Solution:** Ensure units are metric in the GUI settings of BNG

## Acknowledgments

- BeamNG.tech team for providing the simulation environment
- ROS2 community for the robotics framework
- All contributors to this project
