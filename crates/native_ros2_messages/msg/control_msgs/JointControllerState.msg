# This message presents current controller state of one joint.

# It is deprecated as of Humble in favor of SingleDOFStateStamped.msg

# Header timestamp should be update time of controller state
std_msgs/Header header

# The set point, that is, desired state.
float64 set_point

# Current value of the process (ie: latest sensor measurement on the controlled value).
float64 process_value

# First time-derivative of the process value.
float64 process_value_dot

# The error of the controlled value, essentially process_value - set_point (for a regular PID implementation).
float64 error

# Time between two consecutive updates/execution of the control law.
float64 time_step

# Current output of the controller.
float64 command

# Current PID parameters of the controller.
float64 p
float64 i
float64 d
float64 i_clamp
bool antiwindup
