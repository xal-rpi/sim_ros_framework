-- Received terrain patch and physical model parameters for native vehicle queries.
-- Owned by controller_manager; uses its existing packet dispatcher and lifecycle.
-- No socket, actuation, observer or clock policy is implemented here.
-- Native ve_* functions are handwritten in drive_trajopt/embedded/native/vehicle_environment.h.
local ffi = require('ffi')
local abs = math.abs
local floor = math.floor
local max = math.max
local huge = math.huge
local M = {}

-- ============================================================================
-- Native storage and module state (one vehicle Lua VM, one owning thread)
-- ============================================================================
local lib, context, settings, modelParameters
local declarations -- FFI types persist for the VM lifetime, including after stop().
local received, accepted, rejected, replacements = 0, 0, 0, 0
local projectionStatus,projectionResidual,projectionIterations,extrapolatedCount = -1,nil,0,0
local receiveStatus, prepareStatus = 1, 1
local CONFIG_KEYS = {library_path=true, ffi_header_path=true, frame_id=true, epoch_id=true,
  projection_tolerance_m=true, projection_max_iterations=true}

local function finite(value)
  return type(value) == 'number' and value == value and abs(value) < huge
end

local function uint32(value)
  return finite(value) and value >= 0 and value < 4294967296 and value == floor(value)
end

-- ============================================================================
-- Initialization and lifecycle
-- ============================================================================
-- config: library_path/ffi_header_path are explicit BeamNG-readable paths to a
-- matching generated library/header; frame_id/epoch_id are configured uint32 IDs.
-- projection_tolerance_m [m], projection_max_iterations [1..24] control projection.
-- These are context settings, not physical model parameters or actuator settings.
function M.init(_common, config)
  assert(not lib, 'Environment module already initialized')
  assert(type(config) == 'table', 'Explicit environment configuration required')
  for key in pairs(config) do assert(CONFIG_KEYS[key], 'Unknown environment setting: '..tostring(key)) end
  assert(type(config.library_path) == 'string' and config.library_path ~= '', 'library_path required')
  assert(type(config.ffi_header_path) == 'string' and config.ffi_header_path ~= '', 'ffi_header_path required')
  assert(uint32(config.frame_id) and uint32(config.epoch_id), 'Invalid frame_id/epoch_id')
  assert(finite(config.projection_tolerance_m) and config.projection_tolerance_m > 0,
    'Positive projection_tolerance_m required')
  local iterations = config.projection_max_iterations
  assert(finite(iterations) and iterations == floor(iterations) and iterations >= 1 and iterations <= 24,
    'projection_max_iterations must be an integer in [1,24]')

  local header = assert(readFile(config.ffi_header_path), 'Cannot read native FFI header')
  if declarations then
    assert(header == declarations, 'Native type declarations changed; restart vehicle Lua')
  else
    ffi.cdef(header)
    declarations = header
  end
  local native = ffi.load(config.library_path)
  assert(native.ve_abi_version() == 1, 'Unsupported native environment ABI')
  assert(native.ve_scalar_bytes() == ffi.sizeof('vehicle_real'), 'Native precision mismatch')
  local count = tonumber(native.ve_parameter_count())
  local storage = ffi.new('ve_context[1]')
  assert(ffi.sizeof(storage[0].parameters) == ffi.sizeof('vehicle_real') * max(1, count),
    'Parameter storage differs from native export')
  assert(ffi.sizeof(storage[0]) == tonumber(native.ve_context_size()), 'Native context size mismatch')
  assert(tonumber(native.ve_packet_size()) == 216 + 4 * count, 'Native packet size mismatch')

  local cfg = ffi.new('ve_config[1]')
  cfg[0].frame_id, cfg[0].epoch_id = config.frame_id, config.epoch_id
  cfg[0].projection_tolerance_m = config.projection_tolerance_m
  cfg[0].projection_max_iterations = iterations
  assert(finite(tonumber(cfg[0].projection_tolerance_m)) and cfg[0].projection_tolerance_m > 0,
    'Projection tolerance is not representable in native precision')
  local parameters = ffi.new('vehicle_model_parameters[1]')
  parameters[0].values, parameters[0].count = storage[0].parameters, count
  lib, context, settings, modelParameters = native, storage, cfg, parameters
  M.reset()
  return true
end

-- Forget the received snapshot and diagnostics. The next periodic packet restores
-- context; frame/epoch and native types remain as configured by init().
function M.reset()
  assert(lib, 'Environment module is not initialized')
  lib.ve_reset(context)
  received, accepted, rejected, replacements = 0, 0, 0, 0
  projectionStatus,projectionResidual,projectionIterations,extrapolatedCount = -1,nil,0,0
  receiveStatus, prepareStatus = 1, 1 -- VE_NO_CONTEXT
end

function M.stop()
  if not lib then return end
  M.reset()
  lib, context, settings, modelParameters = nil, nil, nil, nil
end

-- ============================================================================
-- Packet reception (called by the existing manager dispatcher)
-- ============================================================================
-- bytes: one complete XLE1 binary string. C copies accepted data and retains no
-- pointer to the string. Invalid packets return ve_status and preserve context.
function M.onPacket(bytes)
  assert(lib and type(bytes) == 'string', 'Initialized environment and binary string required')
  received = received + 1
  local previousRevision, hadContext = context[0].revision, context[0].valid ~= 0
  receiveStatus = lib.ve_receive(context, settings, ffi.cast('const unsigned char *', bytes), #bytes)
  if receiveStatus == 0 then
    accepted = accepted + 1
    if not hadContext or previousRevision ~= context[0].revision then
      replacements = replacements + 1
    end
  else
    rejected = rejected + 1
  end
  return receiveStatus
end

-- ============================================================================
-- Synchronous model/filter access
-- ============================================================================
-- state/query: caller-owned vehicle_state[1]/vehicle_query[1]. Validates parameter
-- bounds and projects onto the received patch, allowing surface extrapolation.
-- No absolute time is needed. Returns ve_status; consume query only on VE_OK (0).
function M.prepare(state, query)
  assert(lib, 'Environment module is not initialized')
  prepareStatus = lib.ve_prepare(context, settings, state, query)
  projectionStatus,projectionResidual,projectionIterations=-1,nil,0
  if prepareStatus==0 or prepareStatus==8 then
    projectionStatus=tonumber(query[0].projection_status)
    projectionResidual=tonumber(query[0].residual_m)
    if not finite(projectionResidual) then projectionResidual=nil end
    projectionIterations=tonumber(query[0].iterations)
  end
  if prepareStatus==0 and projectionStatus==4 then extrapolatedCount=extrapolatedCount+1 end
  return prepareStatus
end

-- Received physical model values (e.g. friction), NOT steering/torque/brake commands.
-- Returns a borrowed vehicle_model_parameters[1]: pointer + export-fixed count.
-- Use synchronously on this thread while initialized; do not mutate or retain
-- across stop(). Requires a received context; use prepare() before evaluation.
function M.getModelParameters()
  assert(lib and context[0].valid ~= 0, 'No accepted model parameter context')
  return modelParameters
end

-- Borrow the already loaded native library for a synchronous model/filter consumer.
-- Declarations are shared; never ffi.cdef them again. Do not retain after stop().
-- Scalar revision for prepared-query cache invalidation, without allocating status.
function M.getRevision()
  assert(lib, 'Environment module is not initialized')
  return context[0].valid ~= 0 and tonumber(context[0].revision) or nil
end

function M.getNativeLibrary()
  assert(lib, 'Environment module is not initialized')
  return lib
end

-- ============================================================================
-- Diagnostics (included by controller_manager.getStatus())
-- ============================================================================
-- Snapshot timestamps below are vehicle seconds for display only. Incoming wire
-- timestamps remain integer microseconds in native metadata; no clock is invented.
function M.getStatus()
  if not lib then return {enabled = false} end
  return {
    enabled = true, received = received, accepted = accepted, rejected = rejected,
    replacements = replacements, receive_status = receiveStatus, prepare_status = prepareStatus,
    projection_status=projectionStatus,projection_residual_m=projectionResidual,
    projection_iterations=projectionIterations,extrapolated_preparations=extrapolatedCount,
    has_context = context[0].valid ~= 0, revision = tonumber(context[0].revision),
    geometry_revision = tonumber(context[0].geometry_revision),
    parameter_revision = tonumber(context[0].parameter_revision),
    state_time_s = tonumber(context[0].state_time_us) * 1e-6,
    frame_id = tonumber(settings[0].frame_id), epoch_id = tonumber(settings[0].epoch_id),
    parameter_count = tonumber(lib.ve_parameter_count()), packet_bytes = tonumber(lib.ve_packet_size()),
  }
end

return M
