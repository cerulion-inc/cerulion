# This message presents current hardware diagnostics

# Header timestamp should be update time of device diagnostics
std_msgs/Header header

# Unique identifier per hardware component, ideally the name of the hardware derived from HardwareInfo e.g. "pal_arm"
string hardware_id

# --- Device Diagnostics Aggregation ---------------------------------
# Diagnostics for individual devices within this hardware set
control_msgs/HardwareDeviceDiagnostics[] hardware_device_diagnostics
