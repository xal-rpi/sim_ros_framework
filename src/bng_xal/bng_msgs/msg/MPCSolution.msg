# MPCSolution.msg - Contains MPC solution data

float32[] traj               # Trajectory data (length = N * len(fields))
float32 total_cost           # Total cost of the MPC solution
int32 solver_status          # Status code from the solver
float32 solve_time           # Time taken to compute the solution (seconds)
float32 curr_time            # Current time when solution was generated

# Additional fields that might be useful
# string[] field_names         # Names of the fields in the trajectory
# int32 num_points             # Number of points in the trajectory
# int32 num_fields             # Number of fields per point
