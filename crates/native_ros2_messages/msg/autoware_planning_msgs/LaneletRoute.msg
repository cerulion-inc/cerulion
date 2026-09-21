# Faithful reconstruction of autoware_planning_msgs/LaneletRoute
# (AutowareFoundation/autoware_msgs). The headline output of
# `mission_planner`: start/goal poses + the ordered lanelet segments that
# make up the route, a UUID, and a modification flag.

std_msgs/Header header
geometry_msgs/Pose start_pose
geometry_msgs/Pose goal_pose
autoware_planning_msgs/LaneletSegment[] segments
unique_identifier_msgs/UUID uuid
bool allow_modification
