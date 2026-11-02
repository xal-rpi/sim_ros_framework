-- Bidirectional conversion between measured BeamNG input and mean roadwheel rad.
local M = {}
local abs, huge = math.abs, math.huge
local function finite(v) return type(v)=='number' and v==v and abs(v)<huge end
local function interpolate(rows, query)
  assert(finite(query), 'Steering query must be finite')
  if query<=rows[1][1] then return rows[1][2] end
  if query>=rows[#rows][1] then return rows[#rows][2] end
  for i=2,#rows do
    local a,b=rows[i-1],rows[i]
    if query<=b[1] then return a[2]+(query-a[1])*(b[2]-a[2])/(b[1]-a[1]) end
  end
end

-- Samples: {{u=-1,delta=...}, ..., {u=1,delta=...}}, strictly monotone.
-- angle(u) and input(delta) saturate at measured endpoints. No fitting occurs.
function M.new(samples)
  assert(type(samples)=='table' and #samples>=2, 'Measured steering_map with at least two samples required')
  local inputs,angles={},{}
  local direction
  for i,sample in ipairs(samples) do
    assert(type(sample)=='table' and finite(sample.u) and finite(sample.delta), 'Invalid steering map sample')
    assert(sample.u>=-1 and sample.u<=1, 'Steering map input must be in [-1,1]')
    if i>1 then
      local previous=samples[i-1]
      assert(sample.u>previous.u, 'Steering map inputs must strictly increase')
      local change=sample.delta-previous.delta
      assert(change~=0, 'Steering map angles must be strictly monotone')
      direction=direction or (change>0 and 1 or -1)
      assert(change*direction>0, 'Steering map angles must be strictly monotone')
    end
    inputs[i]={sample.u,sample.delta}
  end
  assert(inputs[1][1]==-1 and inputs[#inputs][1]==1, 'Steering map must include both full-lock inputs (-1,+1)')
  for i=1,#inputs do
    local row=inputs[direction>0 and i or (#inputs-i+1)]
    angles[i]={row[2],row[1]}
  end
  return {
    count=#inputs, minimum=angles[1][1], maximum=angles[#angles][1],
    angle=function(u) return interpolate(inputs,u) end,
    input=function(delta) return interpolate(angles,delta) end,
  }
end
return M
