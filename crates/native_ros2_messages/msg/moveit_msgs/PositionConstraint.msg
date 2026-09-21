# This message contains the definition of a position constraint.

std_msgs/Header header

# The robot link this constraint refers to
string link_name

# The offset (in the link frame) for the target point on the link we are planning for
geometry_msgs/Vector3 target_point_offset

# The volume this constraint refers to
moveit_msgs/BoundingVolume constraint_region

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight
