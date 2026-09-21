# Interactive marker update
uint8 KEEP_ALIVE=0
uint8 UPDATE=1

string server_id
uint64 seq_num
uint8 type
visualization_msgs/InteractiveMarker[] markers
visualization_msgs/InteractiveMarkerPose[] poses
string[] erases

