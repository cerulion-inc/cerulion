# This message presents current controller state of one degree of freedom.

# DoF name, e.g., joint or Cartesian axis name
string name

# The set point, that is, desired state.
float64 reference

# Current value of the process (ie: latest sensor measurement on the controlled value).
float64 feedback

# First time-derivative of the process value. E.g., velocity.
float64 feedback_dot

# The error of the controlled value, essentially reference - feedback (for a regular PID implementation).
float64 error

# First time-derivative of the error of the controlled value.
float64 error_dot

# Time between two consecutive updates/execution of the control law.
float64 time_step

# Current output of the controller.
float64 output
