# Connected nodes. The pair of nodes that this edge connects
int8[2] nodes

# Distance. Cost to travel this edge
float64 dist

# Capacity. Number of ATRs that can be allocated in this edge. For example, wider corridors can allocate two or three ATRs at the same time, while norrower corridors can only admit one ATR. 
int8 capacity