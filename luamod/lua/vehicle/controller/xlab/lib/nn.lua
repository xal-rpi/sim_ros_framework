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

local ACT = {
  NONE = ffi.C.ACT_NONE,
  RELU = ffi.C.ACT_RELU,
  SIGMOID = ffi.C.ACT_SIGMOID,
  TANH = ffi.C.ACT_TANH,
  SOFTMAX = ffi.C.ACT_SOFTMAX,
}

local libnn = nil
-- weak table to hold per‐model scaling metadata
local model_meta = setmetatable({}, { __mode = 'k' })

local M = {}

function M.init()
  if not libnn then
    local libnnPath = ''
    while libnnPath == '' do
      libnnPath = obj:getLastMailbox('libnnPath')
    end
    libnn = ffi.load(libnnPath)
    os.remove(libnnPath)
    log('I', logTag, 'Loaded nn library from ' .. libnnPath)
  end
end

function M.loadModel(path)
  local def = jsonReadFile(path)
  assert(def, 'Could not load model json')

  -- 1) build the C model
  local model = libnn.nn_create_model(def.num_layers)
  assert(model ~= nil, 'nn_create_model failed')

  -- 2) set layers
  for layer_i, L in ipairs(def.layers) do
    assert(L.type == 'dense', 'only dense supported')
    local in_f = L.in_features
    local out_f = L.out_features
    local wf = in_f * out_f

    local wbuf = ffi.new('float[?]', wf)
    for i = 1, wf do
      wbuf[i - 1] = L.weights[i]
    end

    local bbuf = ffi.new('float[?]', out_f)
    for i = 1, out_f do
      bbuf[i - 1] = L.biases[i]
    end

    local act = ACT[L.activation]
    assert(act, 'unknown activation: ' .. tostring(L.activation))
    local ok = libnn.nn_set_dense_layer(model, layer_i - 1, in_f, out_f, wbuf, bbuf, act)
    assert(ok == 0, 'nn_set_dense_layer failed at layer ' .. layer_i)
  end

  -- 3) stash any input/output‐scaling arrays from the JSON
  model_meta[model] = {
    input_scaling = def.nn_input_scaling or {},
    output_scaling = def.nn_output_scaling or nil,
    input_vars = def.nn_input_var or nil,
    output_vars = def.nn_output_var or nil,
  }

  return model
end

function M.run(model, input_tbl)
  assert(model, 'model is nil')
  local n = #input_tbl
  local meta = model_meta[model]

  -- build & scale input buffer
  local in_buf = ffi.new('float[?]', n)
  if meta and meta.input_scaling and #meta.input_scaling == n then
    for i = 1, n do
      local v = input_tbl[i]
      assert(type(v) == 'number', 'input_tbl[' .. i .. '] is not a number')
      in_buf[i - 1] = v / meta.input_scaling[i]
    end
  else
    for i = 1, n do
      in_buf[i - 1] = input_tbl[i]
    end
  end

  -- run the network
  local out_sz = ffi.new('int[1]')
  local out_buf = libnn.nn_run(model, in_buf, n, out_sz)
  assert(out_buf ~= nil, 'nn_run returned NULL')

  -- copy result to Lua table
  local out = {}
  for i = 0, out_sz[0] - 1 do
    out[i + 1] = tonumber(out_buf[i])
  end
  libnn.nn_free_buffer(out_buf)

  -- apply output de‐scaling if present
  if meta and meta.output_scaling then
    for i = 1, #out do
      out[i] = out[i] * meta.output_scaling[i]
    end
  end

  return out
end

function M.getModelMeta(model)
  return model_meta[model]
end

function M.getModelInputScaling(model)
  local meta = model_meta[model]
  return meta.input_scaling
end

function M.freeModel(model)
  if model then
    libnn.nn_free_model(model)
    model_meta[model] = nil
  end
end

function M.onExtensionLoaded() log('D', logTag, 'NN extension loaded') end

return M
