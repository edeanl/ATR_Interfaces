# Error Types (We should extend this)
uint8 NO_ERROR = 0
uint8 DATA_NOT_AVAILABLE = 1
uint8 CONNECTIVITY = 2
uint8 INVALID_ARGUMENT = 3
uint8 TIMEOUT = 4
uint8 TRANSFORM = 5


# Error ID. See Error Types. (By Default  NO_ERROR)
uint8 id 0

# Error message. In case of error, we can provide human-readable information about the error
string message