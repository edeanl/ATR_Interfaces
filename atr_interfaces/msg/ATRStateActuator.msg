# Actuator Information

# Type of actuator in the ATR, see Actuator Types below
int8 type

# Actuator State, see Actuator States below
int8 status
float32 value 


# Actuator Types
int8 LINEAR=0
int8 LIFTER=1

# Actuator States
int8 IDLE=0
int8 OPEN=1
int8 CLOSED=2
int8 EXTENDED=3
int8 RETRACTED=4

# Actuator Error
# atr_interfaces/ATRErrorActuator error

# int8 id 
# string message

# # Error types
# int8 NONE=0
# int8 MALFUNCTION=1
# int8 NON_RESPONDING=2
# int8 UNKNOWN=3