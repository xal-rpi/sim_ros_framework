-- Optional setpoint feedback: no-op passthrough (returns resolved setpoints unchanged).
-- See luamod/FEEDBACK_CONTRACT.md
local M = {}

local logTag = 'feedback_passthrough'

function M.init(_common, cfg)
  log('I', logTag, 'Passthrough feedback init (stem=' .. tostring(cfg and cfg.stem) .. ')')
  return true
end

function M.transform(ctx)
  if not ctx or type(ctx.resolved) ~= 'table' then
    log('W', logTag, 'transform: missing ctx.resolved')
    return nil
  end
  return ctx.resolved
end

function M.onTune(_data)
  return true
end

function M.reset()
end

function M.stop()
end

return M
