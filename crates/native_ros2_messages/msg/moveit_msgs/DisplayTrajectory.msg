# The model id for which this path has been generated
string model_id

# The representation of the path contains position values for all the joints that are moving along the path;
# a sequence of trajectories may be specified
moveit_msgs/RobotTrajectory[] trajectory

# The robot state is used to obtain positions for all/some of the joints of the robot.
# It is used by the path display node to determine the positions of the joints that are not specified
# in the joint trajectory message above. The positions in the joint trajectory message
# take precedence (for joints that the message covers).
moveit_msgs/RobotState trajectory_start
