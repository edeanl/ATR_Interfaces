# ATRNode
# This is the id of the target node. This IDs represent discrete information, e.g. Node 1, this information has to be transformed into global pose using the plant description. For example, it has to be tranformed into Pose(position+orientation), area (when the robot is within that area it will be cosidered as "in Node X")
int8 node

# time step to reach the node, measured from the time when the path was received.
# This is a relative time and not a delta time. For example:
# node=5, time=0 -> ATR is at Node 5 at time step 0
# node=0, time=2 -> ATR travels to Node 2 and reach it at time step 2 (Delta time = 2 time steps)
# node=3, time=3 -> ATR travels to Node 3 and reach it at time step 3 (Delta time = 1 time step)
float64 time_step
