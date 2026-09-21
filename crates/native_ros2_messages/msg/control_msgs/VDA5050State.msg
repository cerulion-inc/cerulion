# This message presents VDA5050-specific vehicle state information
# For AGVs and AMRs compliant with VDA5050, this provides a snapshot of the vehicle's high-level status
# Refer Official Specification document for the VDA 5050 - https://github.com/VDA5050/VDA5050

# --- Order and Action Status ---------------------------------------
# ID of the currently executed order
string order_id

# Current action execution status, see ActionStatus constants below
string action_status

# ID of the last reached node in the topology
uint32 last_node_id

# --- Vehicle State -------------------------------------------------
# True if the vehicle's drives are active and vehicle is in motion
bool driving

# Current battery charge level in percent (0.0 - 100.0)
float64 battery_charge

# Current operating mode of the vehicle, see OperatingMode constants below
string operating_mode

# --- Error Reporting -----------------------------------------------
# Type/category of the current error, empty string if no error
string error_type

# Human-readable description of the current error, empty string if no error
string error_description

# --- Action Status Constants ---------------------------------------
# VDA5050 action execution states
string ACTION_WAITING    = "WAITING"     # Action is waiting to be executed
string ACTION_INITIALIZING = "INITIALIZING" # Action is being initialized
string ACTION_RUNNING    = "RUNNING"     # Action is currently executing
string ACTION_PAUSED     = "PAUSED"      # Action execution is paused
string ACTION_FINISHED   = "FINISHED"    # Action completed successfully
string ACTION_FAILED     = "FAILED"      # Action execution failed

# --- Operating Mode Constants -------------------------------------
# VDA5050 vehicle operating modes
string MODE_AUTOMATIC    = "AUTOMATIC"   # Vehicle operates autonomously
string MODE_SEMI_AUTOMATIC = "SEMIAUTOMATIC" # Vehicle requires operator confirmation
string MODE_MANUAL       = "MANUAL"      # Vehicle is under manual control
string MODE_SERVICE      = "SERVICE"     # Vehicle is in service/maintenance mode
string MODE_EMERGENCY    = "EMERGENCY"   # Vehicle is in emergency mode
string MODE_TEACHIN      = "TEACHIN"     # Vehicle is in teach-in mode for route learning
