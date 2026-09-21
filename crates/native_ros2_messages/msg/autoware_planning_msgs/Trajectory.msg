# Faithful reconstruction of autoware_planning_msgs/Trajectory
# (AutowareFoundation/autoware_msgs). The motion-planner output consumed by
# the trajectory follower: a stamped, dense array of trajectory points. The
# bounded array `[<=10000]` is wire-equivalent to a dynamic array.

std_msgs/Header header
autoware_planning_msgs/TrajectoryPoint[<=10000] points
