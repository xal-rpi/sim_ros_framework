# Live vehicle plant state from xlab control_state / control_state_send (controlStateOut).

std_msgs/Header header          # ROS time when observation was received/published
float64 sim_time                # BeamNG simulation time [s] (observation.t / data.t)

# Pose (global / map frame)
geometry_msgs/Point position    # x, y, z [m]
float64 yaw                     # [rad]
float64 pitch                   # [rad]
float64 roll                    # [rad]
float64 phi                     # Euler Phi [rad] (luamod: Phi)
float64 beta                    # sideslip [rad]
geometry_msgs/Quaternion orientation  # body attitude; use for RViz / tf-style viz

# Velocity (body frame)
geometry_msgs/Vector3 linear_velocity   # x=vx, y=vy, z=vz [m/s]
float64 v                       # speed magnitude [m/s] (luamod: V)

# Angular rates (body frame)
geometry_msgs/Vector3 angular_velocity   # x=p, y=q, z=r [rad/s]

# Acceleration (body frame, force specific in body frame)
geometry_msgs/Vector3 linear_acceleration  # x/y/z = accel_x/y/z [m/s^2]

# Wheels
float64 w_fl                    # front-left linear wheel speed [m/s]
float64 w_fr                    # front-right [m/s]
float64 w_rl                    # rear-left [m/s]
float64 w_rr                    # rear-right [m/s]
float64 delta_l                 # front-left roadwheel angle [rad]
float64 delta_r                 # front-right roadwheel angle [rad]

# Drivetrain / actuation (measured)
float64 throttle                # pedal [0, 1]
float64 brake                   # pedal [0, 1]
float64 pbrake                  # parking brake [0, 1]
int32 gear_index
float64 gear_ratio
float64 we                      # engine angular speed [rad/s]
float64 pb                      # boost pressure [Pa]

# Torque envelope (from torque_map forward estimate + bracket)
float64 rear_wheel_torque_est   # [N·m]
float64 torque_min              # [N·m] at throttle=0
float64 torque_max              # [N·m] at throttle=1
