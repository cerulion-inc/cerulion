# Interactive marker control
uint8 INHERIT=0
uint8 FIXED=1
uint8 VIEW_FACING=2

uint8 NONE=0
uint8 MENU=1
uint8 BUTTON=2
uint8 MOVE_AXIS=3
uint8 MOVE_PLANE=4
uint8 ROTATE_AXIS=5
uint8 MOVE_ROTATE=6
uint8 MOVE_3D=7
uint8 ROTATE_3D=8
uint8 MOVE_ROTATE_3D=9

string name
geometry_msgs/Quaternion orientation
uint8 orientation_mode
uint8 interaction_mode
bool always_visible
visualization_msgs/Marker[] markers
bool independent_marker_orientation
string description

