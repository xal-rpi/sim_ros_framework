# XLab Low-Level Controller Mod for BeamNG.tech

## Purpose

This repository contains the XLab low-level controller mod for BeamNG.tech, which allows external control of vehicles via TCP commands. It provides a bridge for custom AI or robotics applications to interact with the simulated vehicle physics, including sensor data retrieval, vehicle state management, and controller execution.

## Prerequisites

Before building and using this mod, ensure you have the following installed:

*   **BeamNG.tech:** The simulation environment this mod is designed for.
*   **Git:** For cloning the repository and for the build script to list files.
*   **C Compiler:** A C compiler (e.g., mingw32-gcc, GCC, Clang, referred to as `cc` in the build script) is required to compile a neural network utility (`nn.c`).
*   **Zip Utility:** For packaging the mod files.
*   **Bash Environment:** The build script is designed to run in a Bash shell (common on Linux and macOS, available on Windows via WSL or Git Bash).

## Build Instructions

1.  **Clone the Repository:**
    If you haven't already, clone this repository to your local machine.
    ```bash
    # git clone <repository_url>
    # cd <repository_directory>/luamod
    ```

2.  **Run the Build Script:**
    The `build.bash` script automates the process of collecting necessary Lua files, compiling a C utility, and packaging the mod. This Bash script automates packaging your BeamNG.drive mod (`xlab.zip`) from tracked or all source files in your Git repository. It ensures platform-specific compilation, includes relevant Lua and JSON files, and outputs a ready-to-use zip file.

    When using with WSL/Windows, make sure to set the `BEAMNG_MOD_DIR` environment variable to your BeamNG.drive mods directory in Windows (e.g., `C:\Users\<YourUsername>\AppData\Local\BeamNG.drive\0.35\mods`). You can add this to your `.bashrc` or `.bash_profile` for convenience:
    ```bash
    export BEAMNG_MOD_DIR="/mnt/c/Users/<YourUsername>/AppData/Local/BeamNG.drive/0.35/mods"
    echo 'export BEAMNG_MOD_DIR=\"/mnt/c/Users/<your-username>/AppData/Local/BeamNG.drive/0.35/mods\"' >> ~/.bashrc # More definitive solution
    ```

    Navigate to the `luamod` directory within the repository and execute the script:
    ```bash
    cd /path/to/your/repository/luamod
    ./build.bash [--platform=windows|--win] [--platform=linux|--linux] [--models=tracked|--tracked] [--models=all|--all]
    ```

    To call this script from anywhere, ensure it is executable:
    ```bash
    chmod +x build.bash
    ln -s build.bash ~/.local/bin/xlab-build # Make sure ~/.local/bin is in your PATH
    ```

    Then, you can run it directly fro anywhere in your terminal:

    ```bash
    xlab-build
    xlab-build --platform=windows --models=all
    xlab-build --all # same as xlab-build --platform=linux --models=all
    ```

3.  **Output:**
    The script will create a zip file named `xlab.zip` in your BeamNG.tech mods directory. On Linux, this is typically:
    `~/.local/share/BeamNG.drive/0.35/mods/xlab.zip`
    The script will print the exact path and number of files included upon successful completion.

## Installation & Usage

1.  **Build the Mod:** Follow the "Build Instructions" above to create the `xlab.zip` file.
2.  **Install the Mod:**
    *   Ensure the `xlab.zip` file is located in your BeamNG.tech mods folder (e.g., `~/.local/share/BeamNG.drive/0.35/mods/` on Linux, or `Documents/BeamNG.drive/mods` on Windows if manually moved). The build script attempts to place it in the Linux path by default. If you are on Windows or use a custom mods path, you might need to move `xlab.zip` from the `luamod` directory (where it might be created if the script can't find the BeamNG user path) to your correct BeamNG.drive mods folder.
3.  **Activate the Mod:** Launch BeamNG.tech, go to the Mod Manager, and ensure `xlab` is activated. You might need to restart the game or Lua engine.
4.  **Interaction:**
    *   The mod listens for TCP commands, as indicated in the "Call path" information below. Client applications (e.g., in Python using `BeamNGpy`) can connect to the game and send XlabCommands to interact with the vehicle's controller and state.

## Developer Information

This section details the files modified by this mod and the call paths for key operations, primarily for developers working on or extending the mod.

### Original files modified

Added a XlabCommand handler in both techCore:
- ./ge/extensions/tech/techCore.lua
- ./vehicle/extensions/tech/techCore.lua

### New files

- ./ge/extensions/xlab/sensors.lua
- ./ge/extensions/xlab/xlabCore.lua
- ./vehicle/extensions/xlab/controller.lua
- ./vehicle/extensions/xlab/gtState.lua
- ./vehicle/extensions/xlab/xlabCore.lua
- ./vehicle/controller/xlab/controller_manager.lua
- ./vehicle/controller/xlab/controller_default.lua
- ./vehicle/controller/xlab/gtState.lua
- ./lua/vehicle/controller/xlab/lib/libnn.so (compiled)
- ./lua/vehicle/controller/xlab/models/test.json

### Call path

- First TCP connection between python and lua:
    1. ControllerInterface
    2. SimulationManager
    3. BeamNGpy.connect
- GtState initialization:
    1. ./ge/extensions/tech/techCore.lua:handleXlabCommand
    2. ./ge/extensions/xlab/xlabCore.lua:openGtState
    3. ./ge/extensions/xlab/sensors.lua:createGtState
    4. ./vehicle/extensions/xlab/gtState.lua:create
    5. ./vehicle/controller/xlab/gtState.lua:init
- Controller initialization:
    1. ./ge/extensions/tech/techCore.lua:handleXlabCommand
    2. ./ge/extensions/xlab/xlabCore.lua:openController
    3. ./vehicle/extensions/xlab/controller.lua:create
    4. ./vehicle/controller/xlab/controller_manager.lua:init
    5. ./vehicle/controller/xlab/controller_default.lua:init
