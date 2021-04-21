# ATR ID
int8 atr_id

# ATR Name
string atr_name

# ATR Type
atr_interfaces/ATRType atr_type

# ATR Node initial position. Current position of the ATR in terms of the Node ID. This discrete pose is obtained from the ATRPose and the definition of the Nodes areas
int8 atr_init_node_pose

# ATR current pose (discrete relative to the Node ID)
int8 atr_node_pose

# ATR goal pose (discrete). This goal pose is defined by the path generated with the planner. It will be the last element of the node array. 
int8 atr_node_goal
