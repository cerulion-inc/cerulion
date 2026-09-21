# This message contains the command for a specific interface across multiple joints.

std_msgs/Header header

# List of joint names to command.
string[] joint_names

# The specific interface type to command (e.g., "position", "velocity", "effort").
string interface_name

# The commanded values. The length of this array must exactly match the length of joint_names.
float64[] values
