local logTag = 'CtrlExt'
local M = {}

-- Module state variables
local activeController = nil

M.create = function(data)
  local decoded = lpack.decode(data)
  local controllerData = {}
  for k, v in pairs(decoded) do
    controllerData[k] = v
  end

  activeController = controller.loadControllerExternal(
    'xlab/controller_manager',
    'xlabControllerManager',
    controllerData
  )
end

--- Removes a ground truth state sensor
-- @param sensorId ID of the sensor to remove
function M.remove(controllerId)
  controller.unloadControllerExternal('lowLevelController' .. controllerId)
  activeController = nil
end

return M
