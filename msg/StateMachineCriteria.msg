# The time that the state machine criteria represent
time timestamp

# TODO: These conditions could be put into a bit field
# to be more space efficient
# TODO: Exact definitions for these criteria need to be determined

# "Estimate RGV State" produced an estimate of the position of RGV 1
# with a confidence of at least TBD within the past TBD seconds
bool recent_rgv_1_estimate

# "Estimate RGV State" produced an estimate of the position of RGV 2
# with a confidence of at least TBD within the past TBD seconds
bool recent_rgv_2_estimate

# TODO: This will likely be determined a different way
# The TBD most recent estimates of RGV 1's position are all within
# TBD meters of each other
bool rgv_1_is_moving

# TODO: This will likely be determined a different way
# The TBD most recent estimates of RGV 2's position are all within
# TBD meters of each other
bool rgv_2_is_moving

# The UAS has just spent TBD continuous seconds in the state
# "Localize RGV 1"
bool rgv_1_localized

# The UAS has just spent TBD continuous seconds in the state
# "Localize RGV 2"
bool rgv_2_localized

# The UAS has just spent TBD continuous seconds in the state "Joint"
bool joint_localized

# The camera has detected RGV 1 at least TBD times within the last
# TBD seconds
bool rgv_1_sighted

# The camera has detected RGV 2 at least TBD times within the last
# TBD seconds
bool rgv_2_sighted

# The UAS has just spent TBD continuous seconds in the same Localize
# state, where this TBD is less than the TBD in rgv_1_localized and
# rgv_2_localized
bool minimum_localize_time_reached