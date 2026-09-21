# This service contains the definition for a request to the motion
# planner and the output it provides

# Parameters for the workspace that the planner should work inside
moveit_msgs/WorkspaceParameters workspace_parameters

# Starting state updates. If certain joints should be considered
# at positions other than the current ones, these positions should
# be set here
moveit_msgs/RobotState start_state

# The possible goal states for the model to plan for. Each element of
# the array defines a goal region. The goal is achieved
# if the constraints for a particular region are satisfied
moveit_msgs/Constraints[] goal_constraints

# No state at any point along the path in the produced motion plan will violate these constraints (this applies to all points, not just waypoints)
moveit_msgs/Constraints path_constraints

# The constraints the resulting trajectory must satisfy
moveit_msgs/TrajectoryConstraints trajectory_constraints

# A set of trajectories that may be used as reference or initial trajectories for (typically optimization-based) planners
# These trajectories do not override start_state or goal_constraints
moveit_msgs/GenericTrajectory[] reference_trajectories

# The name of the planning pipeline to use. If no name is specified,
# the configured planning pipeline will be used
string pipeline_id

# The name of the planning algorithm to use. If no name is specified,
# the default planner of the planning pipeline will be used
string planner_id

# The name of the group of joints on which this planner is operating
string group_name

# The number of times this plan is to be computed. Shortest solution
# will be reported.
int32 num_planning_attempts

# The maximum amount of time the motion planner is allowed to plan for (in seconds)
float64 allowed_planning_time

# Scaling factors for optionally reducing the maximum joint velocities and
# accelerations.  Allowed values are in (0,1].  The maximum joint velocity and
# acceleration specified in the robot model are multiplied by thier respective
# factors.  If either are outside their valid ranges (importantly, this
# includes being set to 0.0), the factor is set to the default value of 1.0
# internally (i.e., maximum joint velocity or maximum joint acceleration).
float64 max_velocity_scaling_factor
float64 max_acceleration_scaling_factor

# Maximum cartesian speed for the given link.
# If max_cartesian_speed <= 0 the trajectory is not modified.
# These fields require the following planning request adapter: default_planner_request_adapters/LimitMaxCartesianLinkSpeed
string cartesian_speed_limited_link
float64 max_cartesian_speed # m/s
