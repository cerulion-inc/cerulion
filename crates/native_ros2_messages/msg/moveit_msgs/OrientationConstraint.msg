# This message contains the definition of an orientation constraint.

std_msgs/Header header

# The desired orientation of the robot link specified as a quaternion
geometry_msgs/Quaternion orientation

# The robot link this constraint refers to
string link_name

# Tolerance on the three vector components of the orientation error (optional)
float64 absolute_x_axis_tolerance
float64 absolute_y_axis_tolerance
float64 absolute_z_axis_tolerance

# Defines how the orientation error is calculated
# The error is compared to the tolerance defined above
uint8 parameterization

# The different options for the orientation error parameterization
# - Intrinsic xyz Euler angles (default value)
uint8 XYZ_EULER_ANGLES=0
# - A rotation vector. This is similar to the angle-axis representation,
# but the magnitude of the vector represents the rotation angle.
uint8 ROTATION_VECTOR=1

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight
