# Overall state of the ATR, general Action of the ATR
# The atr can have multiple actions at the same time, e.g. [6,11] -> the atr is on a mission and following a formation. 
int8[] status

# Overall actions
int8 UNDEFINED=0                # state has not be defined yet
int8 OUT_OF_SERVICE=1           # ATR intentionally removed from system
int8 RETURNING_TO_STATION=2     # go to charger command is active
int8 CHARGING=3                 # in station actively charging
int8 DOCKED=4                   # in station, but not charging
int8 AVAILABLE=5                # on the factory floor, but not currently in use
int8 ON_MISSION=6               # robot has been claimed and is performing a series of tasks
int8 ERROR=7                    # an error has been identified - robot needs attenion from an operator
int8 COLLISSION_AVOIDANCE=8     # The atr is in collision and the on-board system is taking control of the atr          
                                # command
int8 DECOUPLING=9               # The atr is decoupling from the formation (is moving away from the formation)
int8 COUPLING=10                # The atr is coupling to the formation (is moving towards the formation)
int8 FORMATION=11               # The atr is in formation 