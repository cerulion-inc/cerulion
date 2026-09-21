# Range sensor reading
uint8 ULTRASOUND=0
uint8 INFRARED=1

std_msgs/Header header
uint8 radiation_type
float32 field_of_view
float32 min_range
float32 max_range
float32 range
float32 variance

