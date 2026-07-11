# Reduced Ground Truth State Message
# Lightweight version for high-frequency streaming from Lua controller_manager
std_msgs/Header header

# Time of the reading (seconds)
float64 time

# Pose (global frame)
float64 x
float64 y
float64 yaw

# Velocity (body frame)
float64 vel_mag     # magnitude [m/s]
float64 vx          # longitudinal velocity [m/s]
float64 vy          # lateral velocity [m/s]

# Dynamics
float64 beta    # sideslip angle [rad]
float64 r       # yaw rate [rad/s]

# Steering & wheels
float64 delta   # average front wheel angle [rad]
float64 wr      # rear wheel speed [rad/s]
float64 wf      # front wheel speed [rad/s]
float64 we      # engine speed [rad/s]

# Actuation
float64 pb       # turbo boost pressure [Pa]
float64 throttle # throttle input [0-1]
float64 brake    # brake input [0-1]

# Acceleration (body frame)
float64 accel_x  # longitudinal acceleration [m/s^2]
float64 accel_y  # lateral acceleration [m/s^2]

# Torque estimation
float64 rear_wheel_torque_est  # estimated rear wheel torque [Nm]