# Solid primitive shape types
uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4
uint8 PRISM=5

# Box dimension indices
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2

# Sphere dimension index
uint8 SPHERE_RADIUS=0

# Cylinder dimension indices
uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1

# Cone dimension indices
uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1

# Prism dimension index
uint8 PRISM_HEIGHT=0

uint8 type
# At no point will dimensions have a length > 3.
float64[<=3] dimensions
geometry_msgs/Polygon polygon
