# Auxiliary1 signal action, see types below
int8 signal 


# Level 1 signal types
int8 NONE=0
int8 RIGHT=1
int8 LEFT=2
# Indicated when the actuator is activated, e.g. clamping 
int8 ACTUATOR_ON=3
# Warning signal (used for the buzzer)
int8 WARNING=4

# if singal=WARNING, this values defines the level, see available values below
int8 warning_level

# Warning levels
# level=1 (high-0,1s low-0,1s)
# level=2 (high-0,5s low-0,5s)
# level=3 (high-0,5s low-1,0s)
# level=4 (high only)
int8 LEVEL_1=0
int8 LEVEL_2=1
int8 LEVEL_3=2
int8 LEVEL_4=3







 