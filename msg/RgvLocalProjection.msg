## Fields
# Time that this projection is for
time timestamp
# ID of the measured RGV
int8 rgv_id
# Source of the underlying measurement
int8 measurement_source
# Projected ground position of the RGV in the local inertial frame (x,y) [meters]
float64[2] rgv_position_local