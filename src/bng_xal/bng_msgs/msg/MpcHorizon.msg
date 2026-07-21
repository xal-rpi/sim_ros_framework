# MPC primal horizon + targets for viz / logging / offline analysis.
# Labels and artifact identity → GetRefTrajInfo (once), not repeated here.

std_msgs/Header header
float64 t_state
bool ok
int32 solver_status
float64 solve_time
float64 total_cost

int32 n_nodes          # N+1
int32 n_stages         # N
int32 nx
int32 nu

float64[] s            # len = n_nodes
float64[] x            # row-major (n_nodes, nx)
float64[] u            # row-major (n_stages, nu)
float64[] x_tgt        # row-major (n_nodes, nx)
float64[] u_tgt        # row-major (n_stages, nu)

# World pose along horizon (pose_from_se)
float64[] pose_x
float64[] pose_y
float64[] pose_z
float64[] pose_psi     # course along horizon
