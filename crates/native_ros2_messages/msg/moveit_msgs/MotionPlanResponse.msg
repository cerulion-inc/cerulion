# The representation of a solution to a planning problem

# The corresponding robot state
moveit_msgs/RobotState trajectory_start

# The group used for planning (usually the same as in the request)
string group_name

# A solution trajectory, if found
moveit_msgs/RobotTrajectory trajectory

# Planning time (seconds)
float64 planning_time

# Error code - encodes the overall reason for failure
moveit_msgs/MoveItErrorCodes error_code
