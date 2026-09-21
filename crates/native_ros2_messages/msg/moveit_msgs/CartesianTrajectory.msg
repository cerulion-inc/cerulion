# This message describes the trajectory of a tracked frame in task-space
std_msgs/Header header

# The name of the Cartesian frame being tracked with respect to the base frame provided in header.frame_id
string tracked_frame

moveit_msgs/CartesianTrajectoryPoint[] points
