# This message presents current controller state of JTC

# Header timestamp should be update time of controller state
std_msgs/Header header

string[] joint_names
# The set point, that is, desired state.
trajectory_msgs/JointTrajectoryPoint reference
# Current value of the process (ie: latest sensor measurement on the controlled value).
trajectory_msgs/JointTrajectoryPoint feedback
# The error of the controlled value, essentially reference - feedback (for a regular PID implementation).
trajectory_msgs/JointTrajectoryPoint error
# Current output of the controller.
trajectory_msgs/JointTrajectoryPoint output

string[] multi_dof_joint_names
# The set point, that is, desired state.
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_reference
# Current value of the process (ie: latest sensor measurement on the controlled value).
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_feedback
# The error of the controlled value, essentially reference - feedback (for a regular PID implementation).
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_error
# Current output of the controller.
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_output
# The speed scaling factor the trajectory is currently being executed with
float64 speed_scaling_factor
