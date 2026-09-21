# A joint-wrench trajectory point defines joint positions, joint velocities, joint accelerations, joint efforts, and a task space wrench, typically used by controllers.
# Note: The time_from_start field is included in the JointTrajectoryPoint.
trajectory_msgs/JointTrajectoryPoint point

# The task-space wrench to apply at this trajectory point.
control_msgs/WrenchFramed wrench
