# A generic trajectory message that may either encode a joint- or cartesian-space trajectory, or both
# Trajectories encoded in this message are considered semantically equivalent
std_msgs/Header header
trajectory_msgs/JointTrajectory[] joint_trajectory
moveit_msgs/CartesianTrajectory[] cartesian_trajectory
