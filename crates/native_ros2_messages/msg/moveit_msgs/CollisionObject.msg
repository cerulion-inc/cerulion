# A header, used for interpreting the poses
std_msgs/Header header

# The object's pose relative to the header frame.
# The shapes and subframe poses are defined relative to this pose.
geometry_msgs/Pose pose

# The id of the object (name used in MoveIt)
string id

# The object type in a database of known objects
object_recognition_msgs/ObjectType type

# The collision geometries associated with the object.
# Their poses are with respect to the object's pose

# Solid geometric primitives
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses

# Meshes
shape_msgs/Mesh[] meshes
geometry_msgs/Pose[] mesh_poses

# Bounding planes (equation is specified, but the plane can be oriented using an additional pose)
shape_msgs/Plane[] planes
geometry_msgs/Pose[] plane_poses

# Named subframes on the object. Use these to define points of interest on the object that you want
# to plan with (e.g. "tip", "spout", "handle"). The id of the object will be prepended to the subframe.
# If an object with the id "screwdriver" and a subframe "tip" is in the scene, you can use the frame
# "screwdriver/tip" for planning.
# The length of the subframe_names and subframe_poses has to be identical.
string[] subframe_names
geometry_msgs/Pose[] subframe_poses

# Adds the object to the planning scene. If the object previously existed, it is replaced.
byte ADD=0

# Removes the object from the environment entirely (everything that matches the specified id)
byte REMOVE=1

# Append to an object that already exists in the planning scene. If the object does not exist, it is added.
byte APPEND=2

# If an object already exists in the scene, new poses can be sent (the geometry arrays must be left empty)
# if solely moving the object is desired
byte MOVE=3

# Operation to be performed
byte operation
