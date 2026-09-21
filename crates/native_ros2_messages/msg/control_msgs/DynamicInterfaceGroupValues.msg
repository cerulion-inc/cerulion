std_msgs/Header header

# List of interface group names , e.g. ["flange_analog_IOs", "flange_vacuum"]
string[] interface_groups
# Key-value pairs representing interfaces and their corresponding values for each interface group listed in `interface_groups`
control_msgs/InterfaceValue[] interface_values
