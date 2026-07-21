# High-rate chart localization + world pose (numeric only).
# Static identity (artifact / chart / labels) → GetRefTrajInfo service.

std_msgs/Header header
float64 t

float64 s
float64 e
float64 dphi

float64 x
float64 y
float64 z
float64 yaw          # body yaw [rad]
float64 course       # yaw + beta (chart course) [rad]
float64 beta         # sideslip [rad]
