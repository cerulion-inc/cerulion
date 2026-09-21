std_msgs/Header header

# List of resource names, e.g. ["arm_joint_1", "arm_joint_2", "gripper_joint"]
string[] joint_names
# Key-value pairs representing interfaces and their corresponding values for each joint listed in `joint_names`
control_msgs/InterfaceValue[] interface_values
