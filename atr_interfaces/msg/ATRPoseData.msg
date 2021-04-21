# The pose of the ATR has three sources

# Increments in the Pose (Delta Pose) of the ATR from the odometry (propioceptive sensors)
geometry_msgs/Pose odom

# Pose of the ATR from photogrammetry (vision-based pose)
geometry_msgs/Pose optom

# Estimated pose of the ATR (using for example sensor fusion, e.g. Kalman Filter)
geometry_msgs/Pose fused_odom

# Discrete Pose (Global pose transformed into Node position --Directed Graph--)
# The nodes describe stations in the plant, which have a global position
# This info must be generated using the plant description (the global pose of the nodes)
# pose ID number (is it pose_node better name?)
int16 pose_id

# name of pose
string name                 

# additional details useful for human operator
string description          

# Pose type, e.g. waiting, loading, etc
atr_interfaces/ATRPoseType type


