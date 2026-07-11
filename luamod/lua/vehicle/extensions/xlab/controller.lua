-- Vehicle extension shim: GE OpenController → controller_manager on this vehicle.
local logTag = 'CtrlExt'
local M = {}

M.create = function(data)
  local decoded = lpack.decode(data)
  local controllerData = {}
  for k, v in pairs(decoded) do
    controllerData[k] = v
  end

  controller.loadControllerExternal(
    'xlab/controller_manager',
    'xlabControllerManager',
    controllerData
  )
end

function M.remove(_controllerId)
  controller.unloadControllerExternal('xlabControllerManager')
end

-- Runtime gtState rebind (GE handleSetControllerGtState).
function M.setGtStateSensor(sensorId)
  local mgr = controller.getController('xlabControllerManager')
  if mgr and mgr.setGtStateSensor then
    mgr.setGtStateSensor(sensorId)
  else
    log('E', logTag, 'xlabControllerManager not loaded; cannot set gtState sensor')
  end
end

return M
