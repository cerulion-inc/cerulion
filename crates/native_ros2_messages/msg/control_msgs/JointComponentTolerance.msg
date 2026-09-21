# Version of JointTolerance.msg with added component field for joints with multiple degrees of freedom
# The difference between two MultiDOFJointTrajectoryPoint cannot be represented as a single number,
# hence we use the component field to represent how to calculate the difference in a way that can
# be represented as a single number.

# Since each joint has multiple degrees of freedom,
# there can be multiple tolerances for each joint, each looking
# at different components.

# If the component is X_AXIS, Y_AXIS, or Z_AXIS, then the tolerance
# is only applied for the specific axis.
# However, if the component is TRANSLATION, then the tolerance is for
# the overall Euclidean distance.
# For these components, the units are meters, meters/sec and meters/sec^2.
# Z_AXIS is only valid with a floating joint, not planar.

# If the component is ROTATION the tolerance is measured in
# radians, radians/sec and radians/sec^2, computed
# between the difference in quaternions.

uint16 X_AXIS=1
uint16 Y_AXIS=2
uint16 Z_AXIS=3
uint16 TRANSLATION=4
uint16 ROTATION=5

string joint_name
uint16 component
float64 position
float64 velocity
float64 acceleration
