# Interactive marker feedback
uint8 KEEP_ALIVE=0
uint8 POSE_UPDATE=1
uint8 MENU_SELECT=2
uint8 BUTTON_CLICK=3
uint8 MOUSE_DOWN=4
uint8 MOUSE_UP=5

std_msgs/Header header
string client_id
string marker_name
string control_name
uint8 event_type
geometry_msgs/Pose pose
uint32 menu_entry_id
geometry_msgs/Point mouse_point
bool mouse_point_valid

