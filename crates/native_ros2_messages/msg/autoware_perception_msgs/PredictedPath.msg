# Faithful reconstruction of autoware_perception_msgs/PredictedPath
# (AutowareFoundation/autoware_msgs). One predicted future trajectory of an
# object: up to 100 poses sampled at a fixed time step, with a confidence.
# The bounded array `[<=100]` is wire-equivalent to a dynamic array.

geometry_msgs/Pose[<=100] path
builtin_interfaces/Duration time_step
float32 confidence
