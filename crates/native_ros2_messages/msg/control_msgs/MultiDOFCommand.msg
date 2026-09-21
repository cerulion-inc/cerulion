# The message defines command for multiple degrees of freedom (DoF) typically used by many controllers.
# The message intentionally avoids 'joint' nomenclature because it can be generally use for command with
# different semantic meanings, e.g., joints, Cartesian axes, or have abstract meaning like GPIO interface.

# names of degrees of freedom
string[] dof_names

# values used by most of the controller
float64[] values

# First derivation of the values, e.g., velocity if values are positions.
# This is useful for PID and similar controllers.
float64[] values_dot
