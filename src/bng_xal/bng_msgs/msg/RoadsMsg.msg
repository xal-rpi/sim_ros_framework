std_msgs/Header header
float64 time                 # Time of reading in seconds
float64 heading_angle        # Heading angle between vehicle forward direction and road reference line
float64 half_width           # Local road half-width
float64 radius               # Local road radius
float64 curvature            # Local road curvature (kappa)
int32 lane_count             # Number of lanes in the current travel direction
float64 dist_centerline      # Distance from vehicle front-axle-midpoint to centerline
float64 dist_left_edge       # Distance to road left edge
float64 dist_right_edge      # Distance to road right edge