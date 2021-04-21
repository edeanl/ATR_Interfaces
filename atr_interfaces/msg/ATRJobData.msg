# TODO: add more detailed info about the fields

# Big number
int8 big_number

# Autonomy
int8 autonomy

# Charging Coefficient
int8 charging_coefficient

# Total number of nodes 
int8 total_nodes

# Start list. Initial condition of the plant 
atr_interfaces/ATRData[] atrs

# Charging stations. Node IDs where the charging stations are located
int8[] charging_stations

# Hub nodes. 
int8[] hub_nodes