# Error ID. See Error Types below. (By Default  NO_ERROR)
uint8 id 

# Error message. In case of error, we can provide human-readable information about the error
string message

# Error Types (We should extend this)
uint8 NO_ERROR = 0
uint8 CRITICAL = 1
uint8 POSITION_NOT_REACHED=2
uint8 FORCE_NOT_REACHED=3