# This message contains a list of motion planning constraints.
# All constraints must be satisfied for a goal to be considered valid

string name

moveit_msgs/JointConstraint[] joint_constraints

moveit_msgs/PositionConstraint[] position_constraints

moveit_msgs/OrientationConstraint[] orientation_constraints

moveit_msgs/VisibilityConstraint[] visibility_constraints
