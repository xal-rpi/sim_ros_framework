std_msgs/Header header

string path_name

float64 s
float64 e
float64 e_z             # NEW: Vertical deviation/error (if applicable)
float64 dphi
float64 proj_dist

float64 x_vehicle
float64 y_vehicle
float64 z_vehicle       # NEW: Vehicle elevation
float64 yaw_vehicle
float64 pitch_vehicle   # NEW: Vehicle pitch (th_c)
float64 roll_vehicle    # NEW: Vehicle roll (phi_c)

float64 x_ref
float64 y_ref
float64 z_ref           # NEW: Reference elevation
float64 yaw_ref
float64 pitch_ref       # NEW: Reference pitch
float64 roll_ref        # NEW: Reference roll