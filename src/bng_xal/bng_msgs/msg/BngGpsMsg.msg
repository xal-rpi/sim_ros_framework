# Live GPS observation from xlab sensor_broadcast.
# BeamNG tech GPS: time, world x/y, lon/lat only (no altitude).

std_msgs/Header header          # ROS time when observation was received/published
float64 sim_time                # BeamNG simulation time [s]

# Local/scenario coordinates [m]
float64 x
float64 y

# Geographic coordinates [deg] (relative to sensor ref_lon/ref_lat)
float64 latitude
float64 longitude
