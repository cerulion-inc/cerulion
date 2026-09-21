# This message presents CANopen-specific device state information
# Refer CiA 301 and CiA 402 application layer documentation

# The CANopen node ID of the device (1-127)
uint8 node_id

# --- CiA 301 State -------------------------------------------------
# Network Management state, see NMTState constants below
uint8 nmt_state

# --- CiA 402 State (for drives) ------------------------------------
# Drive state machine state for motion control devices, see DSP402State constants below
uint8 dsp_402_state

# --- Error Reporting -----------------------------------------------
# Last Emergency (EMCY) error code received from the device
uint32 last_emcy_code

# --- NMT State Constants -------------------------------------------
# CiA 301 Network Management states
uint8 NMT_INITIALISING      = 0   # Device is initializing
uint8 NMT_PRE_OPERATIONAL   = 127 # Device is in pre-operational state
uint8 NMT_OPERATIONAL       = 5   # Device is operational
uint8 NMT_STOPPED           = 4   # Device is stopped

# --- DSP 402 State Constants ---------------------------------------
# CiA 402 Drive state machine states
uint8 DSP402_NOT_READY_TO_SWITCH_ON    = 0  # Not ready to switch on
uint8 DSP402_SWITCH_ON_DISABLED        = 1  # Switch on disabled
uint8 DSP402_READY_TO_SWITCH_ON        = 2  # Ready to switch on
uint8 DSP402_SWITCHED_ON               = 3  # Switched on
uint8 DSP402_OPERATION_ENABLED         = 4  # Operation enabled
uint8 DSP402_QUICK_STOP_ACTIVE         = 5  # Quick stop active
uint8 DSP402_FAULT_REACTION_ACTIVE     = 6  # Fault reaction active
uint8 DSP402_FAULT                     = 7  # Fault state
