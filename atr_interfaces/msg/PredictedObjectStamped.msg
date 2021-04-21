# Header with Time and frame_id
std_msgs/Header header

# Array of Polygons for the N predictions of the object
geometry_msgs/Polygon[] p_polygon

# Object class, e.g. forklift, human, unknown, etc.
atr_interfaces/ObjectClass object_c

# Type of object, e.g., static, dynamic, non-accesible area (nonaa), etc
atr_interfaces/ObjectType object_t

# Object ID
int8 object_id

# Index of this object in the array of Objects. This variable can be use to create an <index, objec_id> map 
int8 object_idx

# Prediction Horizon sample Time
float64 delta_t
