# A name for this grasp
string id

# The internal posture of the hand for the grasp
# positions and efforts are used
trajectory_msgs/JointTrajectory post_place_posture

# The position of the end-effector for the grasp relative to a reference frame
# (that is always specified elsewhere, not in this message)
geometry_msgs/PoseStamped place_pose

# The estimated probability of success for this place, or some other
# measure of how "good" it is.
float64 quality

# The approach motion
moveit_msgs/GripperTranslation pre_place_approach

# The retreat motion
moveit_msgs/GripperTranslation post_place_retreat

# an optional list of obstacles that we have semantic information about
# and that can be touched/pushed/moved in the course of grasping
string[] allowed_touch_objects
