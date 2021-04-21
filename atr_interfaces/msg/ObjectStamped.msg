# Header with Time and frame_id
std_msgs/Header header

# Polygon with the 2D points forming the object
geometry_msgs/Polygon polygon

# Object Centre (Polygon Centroid)
geometry_msgs/Point32 centroid

# Object class, e.g. forklift, human, unknown, etc.
atr_interfaces/ObjectClass object_c

# Type of object, e.g., static, dynamic, non-accesible area (nonaa), etc
atr_interfaces/ObjectType object_t

# Object ID
int16 object_id

# Index of this object in the array of Objects. This variable can be use to create an <index, objec_id> map 
int16 object_idx