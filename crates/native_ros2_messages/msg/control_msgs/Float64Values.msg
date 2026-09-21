std_msgs/Header header

# List of values corresponding to the interfaces or joints defined in Keys.msg
# The size and order must match that of Keys.msg
# Each value corresponds to the key at the same index in Keys.msg
#
# For example. if Keys.msg has keys = ["joint1/position", "joint2/velocity"], then
# InterfacesValues.msg could have values = [1.57, 0.0]. The first value (1.57) corresponds to "joint1/position"
# and the second value (0.0) corresponds to "joint2/velocity"
#
# This message is used by the interfaces_state_broadcaster to publish the values of the interfaces via
# ROS 2 topics ~/names (Keys.msg) and ~/values (InterfacesValues.msg)
float64[] values
