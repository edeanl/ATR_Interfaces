# Defines the collision state of an ATR detected by a sensor

# collision state, see types below
int8 status

# If the sensor provides a distance to collision value in [m], this can be stored here
# If theres many sensors which provides distance, this variable can store the minimal (critical)
# distance
float32 distance

# Human readable description message
string description

# Sensor Type
atr_interfaces/ATRCollisionSensor[] sensors 

# Collision state types 
# There's no potential collision
int8 NONE=0
# There's an eminent collision
int8 CLOSE_COLLISION=1
# The ATR is already in collision
int8 IN_COLLISION=2
