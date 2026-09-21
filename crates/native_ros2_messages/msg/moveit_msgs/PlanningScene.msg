# name of planning scene
string name

# full robot state
moveit_msgs/RobotState robot_state

# The name of the robot model this scene is for
string robot_model_name

#additional frames for duplicating tf (with respect to the planning frame)
geometry_msgs/TransformStamped[] fixed_frame_transforms

#full allowed collision matrix
moveit_msgs/AllowedCollisionMatrix allowed_collision_matrix

# all link paddings
moveit_msgs/LinkPadding[] link_padding

# all link scales
moveit_msgs/LinkScale[] link_scale

# Attached objects, collision objects, even the octomap or collision map can have
# colors associated to them. This array specifies them.
moveit_msgs/ObjectColor[] object_colors

# the collision map
moveit_msgs/PlanningSceneWorld world

# Flag indicating whether this scene is to be interpreted as a diff with respect to some other scene
bool is_diff
