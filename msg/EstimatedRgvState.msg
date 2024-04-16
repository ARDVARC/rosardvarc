## Fields
# Time that these estimates are for
time timestamp
# Estimated position of RGV 1 in the local inertial frame [meters]
float64[3] rgv1_position_local
# Whether RGV 1 is moving
bool rgv1_moving
# RGV 1's estimated speed
float64 rgv1_speed
# TODO: Uncertainty placeholder
# Confidence associated with the RGV 1 estimation (0: low, 1: high)
float64 rgv1_confidence
# Estimated position of RGV 2 in the local inertial frame [meters]
float64[3] rgv2_position_local
# Whether RGV 2 is moving
bool rgv2_moving
# RGV 2's estimated speed
float64 rgv2_speed
# TODO: Uncertainty placeholder
# Confidence associated with the RGV 2 estimation (0: low, 1: high)
float64 rgv2_confidence