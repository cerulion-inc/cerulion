# Image marker types
int32 CIRCLE=0
int32 LINE_STRIP=1
int32 LINE_LIST=2
int32 POLYGON=3
int32 POINTS=4

int32 ADD=0
int32 REMOVE=1

std_msgs/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/Point position
float32 scale
std_msgs/ColorRGBA outline_color
uint8 filled
std_msgs/ColorRGBA fill_color
builtin_interfaces/Duration lifetime
geometry_msgs/Point[] points
std_msgs/ColorRGBA[] outline_colors

