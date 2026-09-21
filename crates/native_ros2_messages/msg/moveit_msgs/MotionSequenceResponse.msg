# An error code reflecting what went wrong
moveit_msgs/MoveItErrorCodes error_code

# The full starting state of the robot at the start of the sequence
moveit_msgs/RobotState sequence_start

# The trajectories that the planner produced for execution
moveit_msgs/RobotTrajectory[] planned_trajectories

# The amount of time it took to complete the motion plan
float64 planning_time
