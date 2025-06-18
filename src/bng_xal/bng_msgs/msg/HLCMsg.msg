# HLCMsg.msg

std_msgs/Header header       # stamped with compute time

float32 controller_latency   # average control loop latency (s)

# dynamic, per‐target data:
#  - target_labels[i] is the key/name
#  - target_values[i] is the corresponding value
string[]          target_labels
float64[]         target_values
