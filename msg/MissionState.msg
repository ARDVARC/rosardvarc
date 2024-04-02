## Constants

# The different possible values of mission_state
# and what they mean:

# Generate setpoints that follow a search pattern
# (either uninformed or based on last known location of
# RGV 1).
uint8 FIND_RGV_1 = 0

# Generate setpoints based off of sensor readings of RGV
# 1. The setpoints will try to get and keep RGV 1 in the
# camera's field of view even as it moves.
uint8 TRACK_RGV_1 = 1

# Generate setpoints based off of sensor readings of RGV 1.
# The setpoints will try to orbit RGV 1 and keep RGV 1 in
# the camera's field of view.
uint8 LOCALIZE_RGV_1 = 2

# Generate setpoints that follow a search pattern (either
# uninformed or based on last known location of RGV 2).
uint8 FIND_RGV_2 = 3

# Generate setpoints based off of sensor readings of RGV 2.
# The setpoints will try to get and keep RGV 2 in the
# camera's field of view even as it moves.
uint8 TRACK_RGV_2 = 4

# Generate setpoints based off of sensor readings of RGV 2.
# The setpoints will try to orbit RGV 2 and keep RGV 2 in
# the camera's field of view.
uint8 LOCALIZE_RGV_2 = 5

# Generate setpoints based off of sensor readings for both
# RGVs. The setpoints will try to orbit the midpoint of the
# RGV positions unless that is not well defined in whach case
# they will try to orbit the mission area center. The camera
# will alternate which RGV is being kept in the camera's
# field of view.
uint8 JOINT_LOCALIZE = 6

## Fields
# The time that this mission state is for
time timestamp

# The mission state, as defined by the constants above
uint8 mission_state