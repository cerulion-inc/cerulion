# Diagnostic status levels
uint8 OK=0
uint8 WARN=1
uint8 ERROR=2
uint8 STALE=3

uint8 level
string name
string message
string hardware_id
diagnostic_msgs/KeyValue[] values

