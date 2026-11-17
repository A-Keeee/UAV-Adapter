# Unmodified native ENU trajectory. No resampling or fitting in transport.
uint8 BSPLINE=0
uint8 POLYNOMIAL=1
std_msgs/Header header
uint64 trajectory_id
builtin_interfaces/Time start_time
uint8 representation
uint8 degree
# BSPLINE: flattened XYZ control points and original knots (possibly negative).
float64[] knots
float64[] control_points
# POLYNOMIAL: positive piece durations; XYZ coefficients in descending powers,
# six coefficients per axis per piece, as in EGO-v2 Piece<5>.
float64[] durations
float64[] coefficients
