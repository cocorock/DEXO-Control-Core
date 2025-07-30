# CrutchCommand message for unified crutches communication
# Replaces separate stop_trigger and manual_calibration_trigger topics

Header header

# Command types enumeration (use constants below)
uint8 command

# Command constants
uint8 ST_CALIBRATION_TRIG = 1
uint8 ST_WALKING_TRIG = 2  
uint8 STOP_TRIG = 3
uint8 SHUTDOWN = 4
uint8 DISABLE_MOTORS = 5  # temporary disabled
uint8 BREAK_MOTORS = 6    # temporary disabled

# Optional command data (for future extensibility)
string command_data