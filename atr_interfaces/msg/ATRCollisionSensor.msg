# Defines the collission sensor information

# Sensor type, see below
int8 type

# Sensor id
int8 id

# Data: for distance sensors this is the value measured by the sensor.
#       for bumpers 0=off, 1=on (there's a collision)
float32 data 

# Available sensor types
int8 BUMPER=0
int8 ULTRA_SONIC=1
int8 LIDAR=2