# This message encapsulates the general-purpose status fields, serving as a baseline for any hardware component

# --- Health & Error ----------------------------------------------
# Overall health status of the device, see HealthStatus constants below
uint8 health_status

# Array of device errors by category, see ErrorDomain constants below
uint8[] error_domain

# --- Operational State -------------------------------------------
# Current operational mode of the device, see ModeStatus constants below
uint8 operational_mode

# Current power state of the device, see PowerState constants below
uint8 power_state

# Current connectivity status of the device, see ConnectivityStatus constants below
uint8 connectivity_status

# --- Vendor & Version Info ----------------------------------------
# Device manufacturer name, e.g. "Bosch"
string manufacturer

# Device model identifier, e.g. "Lidar-XYZ-v2"
string model

# Current firmware version, e.g. "1.2.3"
string firmware_version

# --- Optional Details for Context ---------------------------------
# Provides specific quantitative values related to the enums above.
# e.g., for power_state, could have {key: "voltage", value: "24.1"}
# e.g., for connectivity, could have {key: "signal_strength", value: "-55dBm"}
diagnostic_msgs/KeyValue[] state_details

# --- Health Status Constants -------------------------------------
# High-level health indicators
uint8 HEALTH_UNKNOWN  = 0
uint8 HEALTH_OK       = 1
uint8 HEALTH_DEGRADED = 2
uint8 HEALTH_WARNING  = 3
# Hardware stops publishing state when it returns ERROR/FATAL, how are these set/updated?
uint8 HEALTH_ERROR    = 4
uint8 HEALTH_FATAL    = 5

# --- Error Domain Constants --------------------------------------
# Error categories
uint8 ERROR_NONE           = 0
uint8 ERROR_UNKNOWN        = 1
uint8 ERROR_HW             = 2  # generic hardware fault/error
uint8 ERROR_SW             = 3  # generic software fault/error
uint8 ERROR_OVER_TRAVEL    = 4  # Hardware stopped motion because position is over limits

# Hardware/Software emergency and safety states
uint8 EMERGENCY_STOP_HW    = 5  # state of the emergency stop hardware (i.e. e-stop button state)
uint8 EMERGENCY_STOP_SW    = 6  # state of the emergency stop software system (over travel, pinch point)
uint8 PROTECTIVE_STOP_HW   = 7  # state of the protective stop hardware (i.e. safety field state)
uint8 PROTECTIVE_STOP_SW   = 8  # state of the software protective stop
uint8 SAFETY_STOP          = 9
uint8 CALIBRATION_REQUIRED = 10

# --- Operational Mode Constants ----------------------------------
# Mode of operation
uint8 MODE_UNKNOWN              = 0
uint8 MODE_MANUAL               = 1
uint8 MODE_AUTO                 = 2  # automatic mode when the driver is remote controlling the hardware
uint8 MODE_SAFE                 = 3  # what is the expected use case for this mode?
uint8 MODE_MAINTENANCE          = 4
uint8 MODE_JOG_MANUAL           = 5
uint8 MODE_ADMITTANCE           = 6
uint8 MODE_MONITORED_STOP       = 7
uint8 MODE_HOLD_TO_RUN          = 8
uint8 MODE_CARTESIAN_TWIST      = 9
uint8 MODE_CARTESIAN_POSE       = 10
uint8 MODE_TRAJECTORY_FORWARDING = 11
uint8 MODE_TRAJECTORY_STREAMING  = 12

# --- Power State Constants ---------------------------------------
# Power states
uint8 POWER_UNKNOWN         = 0
uint8 POWER_OFF             = 1
uint8 POWER_STANDBY         = 2
uint8 POWER_ON              = 3
uint8 POWER_SLEEP           = 4
uint8 POWER_ERROR           = 5
# Battery power states see BatteryState.msg
uint8 POWER_LEVEL_LOW       = 6
uint8 POWER_LEVEL_CRITICAL  = 7
uint8 POWER_CHARGING        = 8
uint8 POWER_CHARGING_ERROR  = 9

# --- Connectivity Status Constants -------------------------------
# Connectivity states
uint8 CONNECT_UNKNOWN    = 0
uint8 CONNECT_UP         = 1
uint8 CONNECT_DOWN       = 2
uint8 CONNECT_FAILURE    = 3
uint8 CONNECTION_SLOW    = 4  # to tell the controlling system it is struggling to communicate at rate
