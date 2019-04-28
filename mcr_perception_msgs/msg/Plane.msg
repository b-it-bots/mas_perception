########################
# Plane Geometry Info

Header header

# Plane coefficients as described in http://mathworld.wolfram.com/HessianNormalForm.html
# first three values are the normalized normal vector
float32[4] coefficients

# Plane's convex hull
geometry_msgs/Point32[] convex_hull

# Plane center point, average of convex hull
geometry_msgs/Point32 plane_point

# Range of x, y coordinates
mcr_perception_msgs/BoundingBox2D limits

########################
# Plane name
string name

########################
# Objects associated with plane
mcr_perception_msgs/ObjectList object_list

