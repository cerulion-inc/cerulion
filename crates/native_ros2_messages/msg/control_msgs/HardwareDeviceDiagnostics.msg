# This message presents current hardware device status information

# Header timestamp should be update time of device diagnostics
std_msgs/Header header

# Unique identifier for the device, e.g. "base_motor", "gripper_joint_1"
string device_id

# --- Diagnostic Entries --------------------------------------------
# Key-value pairs representing diagnostic information for the device.
diagnostic_msgs/KeyValue[] entries
