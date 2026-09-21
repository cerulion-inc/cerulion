# This message presents current hardware device status information

# Header timestamp should be update time of device status
std_msgs/Header header

# Unique identifier for the device, e.g. "base_motor", "gripper_joint_1"
string device_id

# --- Standard-Specific States --------------------------------------
# States populated based on the standards relevant to this device.
# A device will only fill the arrays for the standards it implements, rest will be empty

# Generic hardware status information applicable to any device, it is suggested to fill this for all devices
control_msgs/GenericHardwareState[] hardware_status

# CANopen-specific device states (DS-301/DS-402 compliant devices)
control_msgs/CANopenState[] canopen_states

# EtherCAT-specific device states (CoE, FoE, etc.)
control_msgs/EtherCATState[] ethercat_states

# VDA5050-specific states for AGV/mobile robot devices
control_msgs/VDA5050State[] vda5050_states
