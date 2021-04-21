# Distance between ATRs(m)
float64 d 0.0

# Tolerance [0-1]
# max_distance = d * (1 + tolerance)
# min_distance = d * (1 - tolerance)
# tolerance = -1 is reserved to indicate that there is no constraint (d should be ignored). This value is set as default
float64 tolerance -1.0