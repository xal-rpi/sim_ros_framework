# Ground Truth State Message
std_msgs/Header header

# Time of the reading (seconds)
float64 time

# Orientation vectors in the global frame (unit vectors)
geometry_msgs/Vector3 dir_x
geometry_msgs/Vector3 dir_y

# Linear velocity (m/s) and acceleration (m/s^2) in the local frame
geometry_msgs/Vector3 vel
geometry_msgs/Vector3 accel
geometry_msgs/Vector3 accel_raw

# Angular velocity (rad/s) and acceleration (rad/s^2) in the local frame
geometry_msgs/Vector3 ang_vel
# geometry_msgs/Vector3 ang_accel
geometry_msgs/Vector3 ang_vel_raw
geometry_msgs/Vector3 ang_accel_raw

# Position in the global frame (meters)
geometry_msgs/Vector3 pos

# Orientation quaternion in the global frame (unitless)
geometry_msgs/Quaternion quat

# Wheel information
# Speed (m/s), Angular velocity from brake torque (rad/s), Angular velocity (rad/s)
# Brake torque (Nm), Propulsion torque (Nm)
# Wheel angles are in radians.
# - wheel_*_angle: current/default angle (kept for backward compatibility)
# - wheel_*_angle_legacy: legacy acos(cos)+steering-sign estimate
# - wheel_*_angle_atan2: signed atan2 estimate
float64 wheel_fr_speed
float64 wheel_fr_ang_vel_b
float64 wheel_fr_ang_vel
float64 wheel_fr_brake_torque
float64 wheel_fr_prop_torque
float64 wheel_fr_angle
float64 wheel_fr_angle_legacy
float64 wheel_fr_angle_atan2
float64 wheel_fr_downforce

float64 wheel_fl_speed
float64 wheel_fl_ang_vel_b
float64 wheel_fl_ang_vel
float64 wheel_fl_brake_torque
float64 wheel_fl_prop_torque
float64 wheel_fl_angle
float64 wheel_fl_angle_legacy
float64 wheel_fl_angle_atan2
float64 wheel_fl_downforce

float64 wheel_rr_speed
float64 wheel_rr_ang_vel_b
float64 wheel_rr_ang_vel
float64 wheel_rr_brake_torque
float64 wheel_rr_prop_torque
float64 wheel_rr_angle
float64 wheel_rr_angle_legacy
float64 wheel_rr_angle_atan2
float64 wheel_rr_downforce

float64 wheel_rl_speed
float64 wheel_rl_ang_vel_b
float64 wheel_rl_ang_vel
float64 wheel_rl_brake_torque
float64 wheel_rl_prop_torque
float64 wheel_rl_angle
float64 wheel_rl_angle_legacy
float64 wheel_rl_angle_atan2
float64 wheel_rl_downforce

# Control inputs (unitless)
float64 steering
float64 throttle
float64 brake
float64 clutch
float64 pbrake

# Desired control inputs (unitless)
float64 steering_input
float64 throttle_input
float64 brake_input
float64 clutch_input

# Drive mode status (boolean)
bool esc
bool abs
bool tcs
bool engine_running
bool is_realistic_drive
bool mode_4wd
bool mode_range_box
bool is_front_diff_locked
bool is_rear_diff_locked

# Engine information
# Engine load (unitless), Engine torque (Nm), RPM (revolutions per minute)
# Flywheel torque (Nm), Turbo boost (PSI), Supercharger boost (PSI)
float64 engine_load
float64 engine_torque
float64 rpm
float64 flywheel_torque
float64 turbo_boost
float64 supercharger_boost

# Gearbox information
# Gearbox torque (Nm), Gear ratio (unitless), Gear index (integer)
float64 gearbox_torque
float64 gear_ratio
int32 gear_index
