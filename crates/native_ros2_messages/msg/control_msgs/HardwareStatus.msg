# This message presents current hardware component status information

# Header timestamp should be update time of hardware status
std_msgs/Header header

# Unique identifier per hardware component, ideally the name of the hardware derived from HardwareInfo e.g. "pal_arm"
string hardware_id

# --- Device Status Aggregation ---------------------------------
# An array containing the status of individual devices in the hardware component
control_msgs/HardwareDeviceStatus[] hardware_device_states
