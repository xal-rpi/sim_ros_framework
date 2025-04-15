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

```mermaid
flowchart TD
    %% Define styles
    classDef userInput fill:#e1f5fe,stroke:#01579b
    classDef pythonNodes fill:#e8f5e9,stroke:#2e7d32
    classDef rosNodes fill:#fff3e0,stroke:#ff6f00
    classDef luaNodes fill:#f3e5f5,stroke:#6a1b9a
    classDef beamngNodes fill:#ffebee,stroke:#b71c1c
    classDef controllerNodes fill:#f8bbd0,stroke:#880e4f
    classDef dataFlow stroke:#2196f3,stroke-width:2px
    classDef commandFlow stroke:#ff5722,stroke-width:2px

    %% User Input at the top
    User((User)):::userInput
    
    %% ROS Environment
    subgraph ROSEnv ["ROS Environment"]
        direction TB
        
        %% Left side: Simulation components
        subgraph SimComponents ["Simulation Components"]
            direction TB
            SimNode["sim_manager_node.py"]:::rosNodes
            SimManager["simulation_manager.py"]:::pythonNodes
            VehicleManager["manager.py"]:::pythonNodes
            SensorRegistry["SensorRegistry"]:::pythonNodes
        end
        
        %% Right side: Controller components
        subgraph CtrlComponents ["Controller Components"]
            direction TB
            ControllerInterface["controller_interface.py"]:::pythonNodes
            HighLevelController["high_level_controller.py"]:::controllerNodes
            HighLevelCompute["controller_core.c"]:::controllerNodes
            ControllerRegistry["ControllerRegistry"]:::pythonNodes
            LowLevelController["low_level_controller.py"]:::controllerNodes
        end
        
        %% Bottom: Common ROS components
        subgraph ROSCommon ["ROS Services & Messages"]
            ExecuteReq["ExecuteRequest.srv"]:::rosNodes
            ROSMsgs["ROS Messages"]:::rosNodes
            LoggerProcess["logger_process.py"]:::pythonNodes
        end
        
        %% Sensors
        subgraph Sensors ["Sensors"]
            GtStateSensor["GtState.py"]:::pythonNodes
            BasicStateSensor["basic_state.py"]:::pythonNodes
        end
    end
    
    %% Command interface
    SimShell["sim_shell.py"]:::pythonNodes
    
    %% BeamNG Environment
    subgraph BeamNGEnv ["BeamNG Environment"]
        direction TB
        
        %% Python Interface
        BeamNGPy["BeamNGpy Interface"]:::pythonNodes
        
        %% Lua components grouped by role
        subgraph LuaSystem ["Lua System"]
            XlabCore["xlabCore.lua"]:::luaNodes
            SensorsLua["sensors.lua"]:::luaNodes
        end
        
        subgraph VehicleLua ["Vehicle Lua"]
            GtStateModule["gtState.lua"]:::luaNodes
            LowLevelCtrl["lowLevelController.lua"]:::luaNodes
        end
        
        %% Physics Engine
        BeamNGPhysics["BeamNG Physics Engine"]:::beamngNodes
    end

    %% Main user interactions
    User -->|"Launch"|SimNode & ControllerInterface
    User -->|"Commands"|SimShell
    
    %% Main command flowa
    SimShell -->|"execute_request"|ExecuteReq
    ExecuteReq -->|"handle request"|SimNode
    SimNode -->|"process"|SimManager
    SimManager -->|"control vehicle"|BeamNGPy
    
    %% Controller flow
    ControllerInterface -->|"config & initialize"|SimNode
    ControllerInterface -->|"config & initialize"|HighLevelController
    HighLevelController -->|"UDP socket:<br/>send target"|LowLevelCtrl
    HighLevelController <-->|"compute target"|HighLevelCompute
    
    %% Vehicle setup
    SimManager -->|"setup"|VehicleManager
    VehicleManager -->|"register"|SensorRegistry & ControllerRegistry
    ControllerRegistry -->|"create"|LowLevelController
    SensorRegistry -->|"create"|GtStateSensor & BasicStateSensor
    
    %% BeamNG interactions
    LowLevelController -->|"open controller"|BeamNGPy
    GtStateSensor -->|"open sensor"|BeamNGPy
    BeamNGPy -->|"send commands"|XlabCore
    
    %% Lua interactions
    XlabCore -->|"manage sensors"|SensorsLua
    XlabCore -->|"setup"|LowLevelCtrl
    SensorsLua -->|"sensor data"|GtStateModule
    
    %% Physics interactions
    LowLevelCtrl & GtStateModule <-->|"update"|BeamNGPhysics
    
    %% Data flow back to ROS
    BeamNGPy -->|"poll data"|GtStateSensor & BasicStateSensor
    GtStateSensor & BasicStateSensor -->|"convert"|ROSMsgs
    ROSMsgs -->|"publish"|SimNode
    SimNode -->|"log data"|LoggerProcess
    
    %% Results back to user
    ROSMsgs -->|"results"|SimShell
    SimShell -->|"display"|User

    %% Style links for command flow
    linkStyle 0,1,2,3,4,5,6 stroke:#ff5722,stroke-width:2px
    %% Style links for data flow
    linkStyle 7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24 stroke:#2196f3,stroke-width:2px
```

The system consists of these key components:

| Component | Description |
|-----------|-------------|
| **SimulationManager** | Core component managing BeamNG instances, scenarios, and vehicles |
| **VehicleManager** | Handles individual vehicles and their configurations |
| **Sensors** | Various sensor types for vehicle state and environment perception |
| **Controllers** | Both low-level actuator control and high-level decision making |
| **ROS2 Interface** | Bridge between simulation and ROS2 ecosystem |

## Prerequisites

- ROS2 (Humble or newer)
- BeamNG.tech simulator
- Python 3.8+
- Operating System:
  - Windows with WSL2, or
  - Native Linux (in beta for BeamNG.tech)

## Installation

> [!NOTE]
> Step 2 assumes you have a working ROS2 environment.
> The provided flake.nix will install ROS2 and dependencies, allowing you to skip step 2.

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your-organization/bng_xal.git
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
   - For WSL2, find the correct IP address:
     ```bash
     ip route show | grep -i default | awk '{ print $3}'
     ```
   - Update the IP in your scenario configuration files

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
```

## Usage

### Launch Commands

1. **Start the simulator:**
   ```bash
   ros2 launch bng_simulator simulator.launch.py
   ```

2. **Launch with controller:**
   ```bash
   ros2 launch bng_controller controller.launch.py
   ```

3. **Custom configuration:**
   ```bash
   ros2 launch bng_simulator simulator.launch.py config_path:=/path/to/config.yaml
   ```

### Launch File Parameters

#### Simulator Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `config_path` | `[pkg_share]/config/basic_scenario.yaml` | Path to the simulation configuration file |
| `log_level` | `INFO` | Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL) |

#### Controller Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `config_path` | `[pkg_share]/config/basic_scenario.yaml` | Path to BeamNG simulation configuration file |
| `log_level` | `INFO` | Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL) |

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

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- BeamNG.tech team for providing the simulation environment
- ROS2 community for the robotics framework
- All contributors to this project

## Further Documentation

For detailed documentation, API references, and examples, please visit:
[https://your-organization.github.io/bng_xal/](https://your-organization.github.io/bng_xal/)
