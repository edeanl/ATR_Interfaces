# List of ATRs ids. An array with the ids of all the ATR which are in Formation. The formation is defined by the constraints
int8[] atr_list

# Formation ID. To request an especific formation.
int8 formation_id

# Constraints
atr_interfaces/FormationConstraints constraints

# Formation goals
atr_interfaces/FormationPoses[] poses
