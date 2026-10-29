# Whole trajectory, evaluated by GanYu at its control rate. Coordinates are PX4 NED.
# No TrajectorySetpoint is sent to PX4. All knot values are seconds from start_time.
std_msgs/Header header
uint64 trajectory_id
builtin_interfaces/Time start_time
uint8 degree
float64[] knots
# Flattened triples [north, east, down, ...].
float64[] control_points
