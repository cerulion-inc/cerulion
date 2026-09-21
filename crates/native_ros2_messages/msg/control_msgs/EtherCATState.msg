# This message presents EtherCAT-specific device state information
# Refer ETG.1000 EtherCAT Specification

# Position of the slave on the bus (0, 1, 2...)
uint16 slave_position

# Unique vendor identifier from the device's ESI file
string vendor_id

# Unique product code for the device from the device's ESI file
string product_code

# --- EtherCAT State Machine (ESM) ----------------------------------
# Application Layer state, see ALState constants below
uint8 al_state

# True if the slave is in an error state
bool has_error

# AL Status Code indicating the reason for an error (0 = no error)
uint16 al_status_code

# --- Application Layer State Constants -----------------------------
# EtherCAT slave states according to ETG.1000
uint8 AL_STATE_INIT    = 1   # INIT state - slave initialization
uint8 AL_STATE_PREOP   = 2   # PREOP state - pre-operational, mailbox communication
uint8 AL_STATE_BOOTSTRAP = 3 # BOOTSTRAP state - firmware update mode
uint8 AL_STATE_SAFEOP  = 4   # SAFEOP state - safe operational, inputs updated
uint8 AL_STATE_OP      = 8   # OP state - operational, full communication
