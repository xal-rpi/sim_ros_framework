-- lua/common/extensions/xlab/nn.lua
local ffi = require('ffi')
local logTag = 'xlab_nn'

ffi.cdef([[
  typedef enum {
    ACT_NONE=0, ACT_RELU, ACT_SIGMOID, ACT_TANH, ACT_SOFTMAX
  } Activation;
  typedef struct Model Model;
  Model*  nn_create_model(int num_layers);
  int     nn_set_dense_layer(Model* m,
                             int idx,
                             int in_f,
                             int out_f,
                             const float* weights,
                             const float* biases,
                             Activation act);
  float*  nn_run(Model* m,
                 const float* input,
                 int input_size,
                 int* out_size);
  void    nn_free_model(Model* m);
  void    nn_free_buffer(float* buf);
]])

local libnn = nil

local ACT = {
  NONE = ffi.C.ACT_NONE,
  RELU = ffi.C.ACT_RELU,
  SIGMOID = ffi.C.ACT_SIGMOID,
  TANH = ffi.C.ACT_TANH,
  SOFTMAX = ffi.C.ACT_SOFTMAX,
}

local M = {}

function M.loadModel(path)
  -- read + parse
  local def = jsonReadFile(path)

  -- create the C model
  local model = libnn.nn_create_model(def.num_layers)
  assert(model ~= nil, 'nn_create_model failed')

  for layer_i, L in ipairs(def.layers) do
    assert(L.type == 'dense', 'only dense supported')

    local in_f = L.in_features
    local out_f = L.out_features
    local wf = in_f * out_f

    -- 1) weights
    local wbuf = ffi.new('float[?]', wf)
    for j = 1, wf do
      -- copy each Lua number → C array slot
      local v = L.weights[j]
      assert(type(v) == 'number', 'weight[' .. j .. '] is not a number')
      wbuf[j - 1] = v
    end

    -- 2) biases
    local bbuf = ffi.new('float[?]', out_f)
    for j = 1, out_f do
      local v = L.biases[j]
      assert(type(v) == 'number', 'bias[' .. j .. '] is not a number')
      bbuf[j - 1] = v
    end

    local act = ACT[L.activation]
    assert(act, 'unknown activation: ' .. tostring(L.activation))

    local ok = libnn.nn_set_dense_layer(model, layer_i - 1, in_f, out_f, wbuf, bbuf, act)
    assert(ok == 0, 'nn_set_dense_layer failed at layer ' .. layer_i)
  end

  return model
end

function M.run(model, input_tbl)
  assert(model, 'model is nil')
  local n = #input_tbl
  -- allocate C buffer
  local in_buf = ffi.new('float[?]', n)
  for i = 1, n do
    local v = input_tbl[i]
    assert(type(v) == 'number', 'input_tbl[' .. i .. '] is not a number but ' .. type(v))
    in_buf[i - 1] = v
  end

  local out_sz = ffi.new('int[1]')
  local out_buf = libnn.nn_run(model, in_buf, n, out_sz)
  assert(out_buf ~= nil, 'nn_run returned NULL')

  local out = {}
  for i = 0, out_sz[0] - 1 do
    out[i + 1] = tonumber(out_buf[i])
  end

  libnn.nn_free_buffer(out_buf)
  return out
end

function M.freeModel(model)
  if model then libnn.nn_free_model(model) end
end

function M.init()
  if libnn == nil then
    local libnnPath = ''
    while libnnPath == '' do
      libnnPath = obj:getLastMailbox('libnnPath')
    end
    libnn = ffi.load(libnnPath)
    os.remove(libnnPath)
    log('I', logTag, 'Loaded libnn.so')
  end
end

-- optional hook
function M.onExtensionLoaded() log('D', logTag, 'NN extension loaded') end

return M
