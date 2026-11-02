-- Cartesian TVLQR: target = previewed_nominal - K * chassis_error13.
-- LLC owns plan decoding/selection; this module owns paired correction and expiry.
-- Steering [rad], drive torque [Nm], optional brake [0,1]; wheel-speed PI stays in LLC.
-- Work arrays are private/reused; transform returns a fresh complete setpoint table.
local abs, sqrt, acos, sin, atan2 = math.abs, math.sqrt, math.acos, math.sin, math.atan2
local min, max, floor, huge = math.min, math.max, math.floor, math.huge
local M = {requiresReferencePacket = true}
local common, cfg, gt, controller, plan
local report = {}
local measured, referenceQuat, err, correction = {}, {}, {}, {0, 0, 0}
local TARGET_FIELDS = {'steer_des_rad', 'torque_des', 'brake_des'}
local FIELDS = {'tvlqr_status', 'tvlqr_sequence', 'tvlqr_station', 'tvlqr_source_age',
  'tvlqr_steer_correction', 'tvlqr_torque_correction', 'tvlqr_saturated'}
local KEYS = {frame_id=true, epoch_id=true, max_source_age_s=true, max_state_age_s=true}

local function finite(v) return type(v) == 'number' and v == v and abs(v) < huge end
local function uint32(v) return finite(v) and v >= 0 and v < 4294967296 and v == floor(v) end
local function publish(status)
  report.tvlqr_status = status
  for _,name in ipairs(FIELDS) do gt.setCustomField(name, report[name] or 0) end
end
local function release(status)
  if controller then controller.clearCommand() end
  plan = nil
  if report.tvlqr_status ~= status then log('W', 'feedback_tvlqr', 'Released external control: '..status) end
  report.tvlqr_steer_correction, report.tvlqr_torque_correction = 0, 0
  publish(status)
  return false
end

-- Validate a full startup configuration or a partial tune before committing it.
local function settings(values, previous)
  assert(type(values)=='table', 'feedback_config mapping required')
  for key in pairs(values) do assert(KEYS[key], 'Unknown TVLQR setting: '..tostring(key)) end
  local next = {}
  for key in pairs(KEYS) do
    next[key] = values[key]
    if next[key]==nil and previous then next[key]=previous[key] end
  end
  assert(uint32(next.frame_id) and uint32(next.epoch_id), 'Explicit TVLQR frame_id/epoch_id required')
  if previous then
    assert(next.frame_id==previous.frame_id and next.epoch_id==previous.epoch_id,
      'TVLQR frame/epoch changes require receiver restart')
  end
  if next.max_source_age_s==nil then next.max_source_age_s=.3 end
  if next.max_state_age_s==nil then next.max_state_age_s=.05 end
  assert(finite(next.max_source_age_s) and next.max_source_age_s>0
    and finite(next.max_state_age_s) and next.max_state_age_s>0, 'Invalid TVLQR age limits')
  return next
end

-- Apply age-limit changes atomically, preserving the accepted plan and source clock.
function M.onTune(data)
  local ok, next = pcall(function()
    assert(cfg, 'TVLQR is not initialized')
    return settings(data.feedback_config, cfg)
  end)
  if not ok then log('E', 'feedback_tvlqr', 'Tune rejected: '..tostring(next));return false end
  cfg=next
  return true
end

function M.init(c, config)
  assert(not common, 'TVLQR already initialized')
  cfg=settings(config.settings)
  gt = assert(c.gtStateManager.getGtStateController(c.gtStateSensorId), 'GTState required')
  assert(gt.registerCustomField and gt.setCustomField, 'GTState custom fields required')
  for _,name in ipairs(FIELDS) do gt.registerCustomField(name, 0) end
  common = c
  M.reset()
  return true
end

function M.reset()
  if controller then controller.clearCommand() end
  plan, controller = nil, nil
  report = {accepted=0, rejected=0}
  if gt then publish(0) end
end
function M.stop()
  M.reset()
  common, cfg, gt = nil, nil, nil
end
function M.getStatus()
  local out = {}
  for key,value in pairs(report) do out[key] = value end
  return out
end

-- Called only after the LLC validates and accepts the paired nominal plan.
function M.setPlan(received, ctl)
  controller=ctl
  plan=received.enabled and received.gains and received or nil
  report.accepted=report.accepted+1
  report.tvlqr_sequence=received.sequence
  report.tvlqr_steer_correction,report.tvlqr_torque_correction=0,0
  publish(plan and 1 or 0)
end

-- Hamilton xyzw, shortest-arc SLERP; q and -q describe the same attitude.
local function quatAt(values, a, b, f)
  local dot = 0
  for k = 1, 4 do dot = dot + values[a + 4 + k] * values[b + 4 + k] end
  local sign = dot < 0 and -1 or 1
  dot = abs(dot)
  local w0, w1 = 1 - f, f
  if dot < .9995 then
    local theta = acos(min(1, dot))
    w0, w1 = sin((1 - f) * theta) / sin(theta), sin(f * theta) / sin(theta)
  end
  local norm = 0
  for k = 1, 4 do
    referenceQuat[k] = w0 * values[a + 4 + k] + w1 * sign * values[b + 4 + k]
    norm = norm + referenceQuat[k]^2
  end
  local scale = 1 / sqrt(norm)
  for k = 1, 4 do referenceQuat[k] = referenceQuat[k] * scale end
  return referenceQuat
end

local function interpolate(values, a, b, f, offset)
  return values[a + offset] + f * (values[b + offset] - values[a + offset])
end

-- Validate timing/pairing, form tangent-space error, correct, then clip live rails.

function M.transform(ctx)
  if not plan then return ctx.resolved end
  local now = ctx.sim_t
  assert(finite(now), 'Invalid TVLQR vehicle clock')
  report.tvlqr_source_age = now-plan.source
  if now < plan.source-.001 or now-plan.source > cfg.max_source_age_s then return release(-2) end
  assert(ctx.raw and ctx.raw.trajectory and ctx.raw.trajectory.x == plan.native.x,
    'TVLQR reference/gain pairing lost')
  local pick = assert(ctx.raw.selection, 'LLC selection required')
  local i,f = pick.llc_segment,pick.llc_fraction
  assert(uint32(i) and i >= 1 and i < plan.n and finite(f) and f >= 0 and f <= 1,
    'Invalid LLC interpolation selection')
  if (i == plan.n-1 and f >= 1) or pick.llc_clamp_steering == 1 or pick.llc_clamp_torque == 1 then
    return release(-3)
  end
  local state = common.gtStateManager.geGtStateReading(common.gtStateSensorId)
  if not state or not finite(state.time) or state.time > now+.001 or now-state.time > cfg.max_state_age_s then
    return release(-4)
  end
  assert(state.pos and state.quat and state.vel and state.angVel and state.wheelRL and state.wheelRR,
    'Incomplete TVLQR GTState measurement')
  for k = 1, 3 do
    measured[k] = state.pos[k]
    measured[k + 7] = state.vel[k]
    measured[k + 10] = state.angVel[k]
  end
  for k = 1, 4 do measured[k + 3] = state.quat[k] end
  measured[14] = .5 * (assert(state.wheelRL.speed) + assert(state.wheelRR.speed))
  for k=1,14 do assert(finite(measured[k]), 'Invalid TVLQR GTState channel '..k) end
  local qnorm = 0
  for k=4,7 do qnorm = qnorm+measured[k]^2 end
  assert(abs(qnorm-1) < 1e-4, 'Nonunit measured TVLQR quaternion')
  for k=4,7 do measured[k] = measured[k]/sqrt(qnorm) end
  local v = plan.values
  local a,b = (i-1)*plan.width,i*plan.width
  report.tvlqr_station = interpolate(v, a, b, f, 0)
  local qr = quatAt(v,a,b,f)
  local x,y,z,w = measured[4],measured[5],measured[6],measured[7]
  -- conjugate(qref) * qmeasured
  local dx = qr[4]*x-qr[1]*w-qr[2]*z+qr[3]*y
  local dy = qr[4]*y+qr[1]*z-qr[2]*w-qr[3]*x
  local dz = qr[4]*z-qr[1]*y+qr[2]*x-qr[3]*w
  local dw = qr[4]*w+qr[1]*x+qr[2]*y+qr[3]*z
  if dw < 0 then dx,dy,dz,dw = -dx,-dy,-dz,-dw end
  local length = sqrt(dx*dx+dy*dy+dz*dz)
  local factor = length < 1e-9 and 2 or 2*atan2(length,dw)/length
  for k = 1, 3 do err[k] = measured[k] - interpolate(v, a, b, f, k + 1) end
  err[4], err[5], err[6] = dx * factor, dy * factor, dz * factor
  for k=8,14 do err[k-1] = measured[k]-interpolate(v, a, b, f, k + 1) end
  local out={}
  for key,value in pairs(ctx.resolved) do out[key]=value end
  correction[1], correction[2], correction[3] = 0, 0, 0
  for row,bit in ipairs(plan.rows) do
    local delta=0
    for col=0,plan.nx-1 do delta=delta-interpolate(v, a, b, f, plan.gainOffset + (row - 1) * plan.nx + col) * err[col + 1] end
    local field=TARGET_FIELDS[bit+1]
    assert(finite(delta) and finite(out[field]),'Invalid feedback nominal/correction')
    out[field]=out[field]+delta
    correction[bit+1]=delta
  end
  local steeringMin,steeringMax=controller.getSteeringCapability()
  local steer,torque,brake=out.steer_des_rad,out.torque_des,out.brake_des
  out.steer_des_rad=max(steeringMin,min(steeringMax,steer))
  out.torque_des=max(state.torque_min,min(state.torque_max,torque))
  if brake then out.brake_des=max(0,min(1,brake)) end
  report.tvlqr_saturated=(out.steer_des_rad~=steer or out.torque_des~=torque or out.brake_des~=brake) and 1 or 0
  -- Preserve wheel-speed target; the LLC's existing pi_enable owns PI policy.
  report.tvlqr_steer_correction = correction[1]
  report.tvlqr_torque_correction = correction[2]
  publish(1)
  return out
end

return M
