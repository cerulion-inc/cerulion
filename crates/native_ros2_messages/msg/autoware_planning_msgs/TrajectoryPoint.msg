# Faithful reconstruction of autoware_planning_msgs/TrajectoryPoint
# (AutowareFoundation/autoware_msgs). One sample of a planned trajectory: a
# time-from-start, a pose, and the longitudinal/lateral kinematics +
# steering targets. Fully fixed (no variable-length fields).

builtin_interfaces/Duration time_from_start
geometry_msgs/Pose pose
float32 longitudinal_velocity_mps
float32 lateral_velocity_mps
float32 acceleration_mps2
float32 heading_rate_rps
float32 front_wheel_angle_rad
float32 rear_wheel_angle_rad
