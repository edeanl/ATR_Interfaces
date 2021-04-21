# Task ID
int8 task_id

# Location, Node where the task take place (Discrete location)
int8 task_location

# List of task that have to be executed before this task
int8[] precedence 

# Time window, minimum and maximum time this task can use
float64[] time_window
