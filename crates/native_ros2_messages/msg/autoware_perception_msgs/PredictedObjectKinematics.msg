# Faithful reconstruction of
# autoware_perception_msgs/PredictedObjectKinematics
# (AutowareFoundation/autoware_msgs). Current pose/twist/acceleration (with
# covariance) plus the set of predicted future paths for the object. The
# bounded array `[<=100]` is wire-equivalent to a dynamic array.

geometry_msgs/PoseWithCovariance initial_pose_with_covariance
geometry_msgs/TwistWithCovariance initial_twist_with_covariance
geometry_msgs/AccelWithCovariance initial_acceleration_with_covariance
autoware_perception_msgs/PredictedPath[<=100] predicted_paths
