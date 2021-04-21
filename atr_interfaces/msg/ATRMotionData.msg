# The motion (velocity or acceleration) of the ATR has three sources

# Motion of the ATR from the odometry (propioceptive sensors)
geometry_msgs/Twist odom

# Motion of the ATR from photogrammetry (vision-based Motion)
geometry_msgs/Twist optom

# Estimated Motion of the ATR (using for example sensor fusion, e.g. Kalman Filter)
geometry_msgs/Twist fused_odom

