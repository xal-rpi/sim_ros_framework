# Live IMU observation from xlab sensor_broadcast (roof_imu / AdvancedIMU path).
# Slim alternative to sensor_msgs/Imu — no unused orientation covariance.

std_msgs/Header header          # ROS time when observation was received/published
float64 sim_time                # BeamNG simulation time [s]

# Sensor position (vehicle/body frame as provided by BeamNG tech IMU)
geometry_msgs/Vector3 position

# Linear acceleration [m/s^2]
geometry_msgs/Vector3 accel

# Angular velocity [rad/s]
geometry_msgs/Vector3 gyro

# Angular acceleration [rad/s^2] (when available from IMU reading)
geometry_msgs/Vector3 ang_accel
