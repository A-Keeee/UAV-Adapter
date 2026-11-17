# Sample of the original planner trajectory, in ENU / FLU, simulation time.
# The controller never reconstructs a new spline from these samples.
std_msgs/Header header
uint64 trajectory_id
builtin_interfaces/Time valid_until
float64[3] position
float64[3] velocity
float64[3] acceleration
float64[3] jerk
float64 yaw
float64 yaw_rate
