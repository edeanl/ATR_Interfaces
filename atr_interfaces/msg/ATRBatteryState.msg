# Battery state type, defines the state of the battery
# see Battery state types below
int8 status 

# State of the battery charge. Provides the remaining battery in %, e.g. 0.8 = 80%.
float32 charge_state

# Battery life time. Estimation of how long the battery will last in [hrs]
float32 life_time

# Defines the total capacity of the battery. In a new battery, this number is equal to 1 (100%)
float32 health_state


# Battery state types
# The battery state cannot be determined
int8 UNDEFINED=0
# The battery is fully charged, between (100%-80%)
int8 FULL=1
# The battery is charged between (79%-60%)
int8 HIGH=2
# The battery is charged between (59%-40%)
int8 MID=3
# The battery is charged between (39%-15%)
int8 LOW=4
# The battery is charged between (15%-0%)
int8 EMPTY=5



