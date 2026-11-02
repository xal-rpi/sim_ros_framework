-- XLP1/XLG1 v2: decode and validate a whole datagram before nominal installation.
-- Receipt path only; owns no sockets, command history or actuation state.
local ffi = require('ffi')
local abs, floor, huge = math.abs, math.floor, math.huge
local M = {}
local HEADER_BYTES, MAX_BYTES = 56, 1200
local NOMINAL_WIDTH, ERROR_WIDTH = 22, 13

local function finite(value)
  return type(value) == 'number' and value == value and abs(value) < huge
end

-- Byte offsets follow the wire header; Lua strings themselves are 1-based.
local function u16(bytes, offset)
  local low, high = bytes:byte(offset + 1, offset + 2)
  return low + 256 * high
end

local function u32(bytes, offset)
  return u16(bytes, offset) + 65536 * u16(bytes, offset + 2)
end

local function u64(bytes, offset)
  local value = ffi.new('uint64_t[1]')
  ffi.copy(value, bytes:sub(offset + 1, offset + 8), 8)
  return value[0]
end

local function has(mask, bit)
  return floor(mask / 2^bit) % 2 == 1
end

-- Returns a private decoded plan or raises without changing accepted state.
-- now/config ages use vehicle seconds; source is uint64 microseconds on the wire.
-- Example: received.native.steer[i] is the nominal roadwheel angle [rad].
function M.decode(bytes, now, config, previousSource, previousSequence, active)
  assert(type(bytes) == 'string' and #bytes >= HEADER_BYTES and #bytes <= MAX_BYTES,
    'Invalid plan size')
  local magic = bytes:sub(1, 4)
  assert((magic == 'XLP1' or magic == 'XLG1') and u16(bytes, 4) == 2
    and u16(bytes, 42) == 1, 'Unsupported execution-plan layout')
  local enabled = u32(bytes, 8)
  assert(u16(bytes, 6) == 0 and u32(bytes, 48) == 0 and enabled <= 1,
    'Invalid plan reserved fields/flags')
  assert(u32(bytes, 12) == config.frame_id and u32(bytes, 16) == config.epoch_id,
    'Plan frame/epoch mismatch')

  local sequence = u32(bytes, 20)
  local source = tonumber(u64(bytes, 32)) * 1e-6
  assert(not previousSource or source >= previousSource, 'Regressed plan source clock')
  -- A new publisher may restart its counter only after external authority ends.
  local restart = not active and previousSource and source > previousSource
  assert(not previousSequence or sequence > previousSequence or restart,
    'Old/conflicting plan sequence')
  assert(finite(now) and source <= now + .001, 'Future/invalid plan source time')
  assert(enabled == 0 or now - source <= config.max_source_age_s, 'Stale plan source')

  local n, mask, desiredMask = u16(bytes, 40), u32(bytes, 44), u32(bytes, 52)
  assert(n >= 2 and n <= 64 and mask >= 1 and mask <= 7 and desiredMask <= 7,
    'Invalid plan fields')
  local rows = {}
  for bit = 0, 2 do
    if has(mask, bit) then rows[#rows + 1] = bit end
    assert(not has(desiredMask, bit) or has(mask, bit), 'Inactive desired field')
  end
  assert(has(mask, 0) and has(mask, 1), 'LLC requires active steering and torque')
  local gains = magic == 'XLG1'
  local width = NOMINAL_WIDTH + (gains and #rows * ERROR_WIDTH or 0)
  assert(#bytes == HEADER_BYTES + 4 * n * width, 'Execution-plan length mismatch')

  local values = ffi.new('float[?]', n * width)
  ffi.copy(values, bytes:sub(HEADER_BYTES + 1), 4 * n * width)
  for k = 0, n * width - 1 do
    assert(finite(tonumber(values[k])), 'Nonfinite plan field')
  end
  local native = {x={}, y={}, z={}, t={}, wr={}, steer={}, torque={}}
  if has(mask, 2) then native.brake = {} end
  -- Node slots: s,time,position3,quat4,velocity3,angular_velocity3,wr,targets3,desired3.
  for i = 1, n do
    local j = (i - 1) * width
    if i > 1 then
      assert(values[j] > values[j - width] and values[j + 1] > values[j - width + 1],
        'Plan s/time not increasing')
    end
    local norm = 0
    for k = 5, 8 do norm = norm + values[j + k]^2 end
    assert(abs(norm - 1) < 1e-4, 'Nonunit plan quaternion')
    for bit = 0, 2 do
      assert(has(mask, bit) or values[j + 16 + bit] == 0, 'Nonzero inactive actuator')
      assert(has(desiredMask, bit) or values[j + 19 + bit] == 0,
        'Nonzero inactive desired input')
    end
    native.x[i], native.y[i], native.z[i] =
      tonumber(values[j + 2]), tonumber(values[j + 3]), tonumber(values[j + 4])
    native.t[i], native.wr[i] = source + tonumber(values[j + 1]), tonumber(values[j + 15])
    native.steer[i], native.torque[i] = tonumber(values[j + 16]), tonumber(values[j + 17])
    if native.brake then native.brake[i] = tonumber(values[j + 18]) end
  end
  return {
    values=values, n=n, native=native, source=source, sequence=sequence,
    referenceId=u64(bytes, 24), enabled=enabled == 1, width=width, nx=ERROR_WIDTH,
    rows=rows, gains=gains, gainOffset=NOMINAL_WIDTH,
  }
end

return M
