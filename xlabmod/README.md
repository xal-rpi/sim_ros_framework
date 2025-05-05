# Low level controller mod for beamng.tech

## Original files modified

Added a XlabCommand handler in both techCore:
- ./ge/extensions/tech/techCore.lua
- ./vehicle/extensions/tech/techCore.lua

## New files

- ./ge/extensions/xlab/sensors.lua
- ./ge/extensions/xlab/xlabCore.lua
- ./vehicle/extensions/xlab/controller.lua
- ./vehicle/extensions/xlab/gtState.lua
- ./vehicle/extensions/xlab/xlabCore.lua
- ./vehicle/controller/xlab/controller_manager.lua
- ./vehicle/controller/xlab/controller_default.lua
- ./vehicle/controller/xlab/gtState.lua

## Call path

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

