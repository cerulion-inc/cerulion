# The header is used to specify the reference time for the trajectory durations
std_msgs/Header header

# The names of the active joints in each trajectory point. These names are
# ordered and must correspond to the values in each trajectory point.
string[] joint_names

# Array of trajectory points, which describe the joint positions, velocities,
# accelerations and/or efforts, and task space wrenches at each time point.
control_msgs/JointWrenchTrajectoryPoint[] points
