# Provides meta-information about a visual pipeline.
#
# This message serves a similar purpose to sensor_msgs/CameraInfo, but instead
#   of being tied to hardware, it represents information about a specific
#   computer vision pipeline. This information stays constant (or relatively
#   constant) over time, and so it is wasteful to send it with each individual
#   result.

# Used for sequencing
std_msgs/Header header

# Name of the vision pipeline. This should be a value that is meaningful to an
#   outside user.
string method

# Location where the metadata database is stored. The recommended location is
#   as an XML string on the ROS parameter server, but the exact implementation
#   and information is left up to the user.
string database_location

# Metadata database version. This counter is incremented
#   each time the pipeline begins using a new version of the database.
int32 database_version
