# Faithful reconstruction of autoware_perception_msgs/ObjectClassification
# (AutowareFoundation/autoware_msgs). A single class label + probability;
# the label constants below mirror the upstream enum.

uint8 UNKNOWN = 0
uint8 CAR = 1
uint8 TRUCK = 2
uint8 BUS = 3
uint8 TRAILER = 4
uint8 MOTORCYCLE = 5
uint8 BICYCLE = 6
uint8 PEDESTRIAN = 7

uint8 label
float32 probability
