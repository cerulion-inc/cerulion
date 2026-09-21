# Provides meta-information about a visual pipeline.
#
# This message is meant to allow converting data from vision pipelines that
#   return id based classifications back to human readable string class names.

# Used for sequencing
std_msgs/Header header

# An array of uint16 keys and string values containing the association
#   between class identifiers and their names. According to the amount
#   of classes and the datatype used to store their ids internally, the
#   maxiumum class id allowed (65535 for uint16 and 255 for uint8) belongs to
#   the "UNLABELED" class.
vision_msgs/VisionClass[] class_map

# The value between 0-1 used as confidence threshold for the inference.
float32 threshold
