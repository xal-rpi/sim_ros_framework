local logTag = 'CtrlExt'
local M = {}

-- Module state variables
local controllers = {} -- Collection of active Groundtruth state sensors

function M.create(data)
  -- Create a controller instance for this GtState sensor
  local decodedData = lpack.decode(data)
  local controllerData = {
    controllerId = decodedData.controllerId,
    listenIp = decodedData.listenIp,
    listenPort = decodedData.listenPort,
    sendIp = decodedData.sendIp,
    sendPort = decodedData.sendPort,
    gtStateSensorId = decodedData.gtStateSensorId,
    controllerType = decodedData.controllerType,
  }

  controllers[decodedData.controllerId] = {
    data = controllerData,
    controller = controller.loadControllerExternal(
      'xlab/controller_manager',
      'controller' .. decodedData.controllerId,
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
