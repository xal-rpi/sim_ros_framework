local logTag = 'CtrlExt'
local M = {}

-- Module state variables
local controllers = {} -- Collection of active Groundtruth state sensors

M.create = function(data)
  local decoded = lpack.decode(data)
  local controllerData = {}
  for k, v in pairs(decoded) do
    controllerData[k] = v
  end

  controllers[decoded.controllerId] = {
    data = controllerData,
    controller = controller.loadControllerExternal(
      'xlab/controller_manager',
      'controller' .. decoded.controllerId,
      controllerData
    ),
  }
end

--- Removes a ground truth state sensor
-- @param sensorId ID of the sensor to remove
function M.remove(controllerId)
  controller.unloadControllerExternal('lowLevelController' .. controllerId)
  controllers[controllerId] = nil
end

return M
