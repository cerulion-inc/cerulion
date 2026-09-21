# Faithful reconstruction of autoware_perception_msgs/Shape
# (AutowareFoundation/autoware_msgs). The object footprint polygon plus a
# bounding-box/cylinder/polygon discriminant and 3D dimensions.

uint8 BOUNDING_BOX=0
uint8 CYLINDER=1
uint8 POLYGON=2

uint8 type
geometry_msgs/Polygon footprint
geometry_msgs/Vector3 dimensions
