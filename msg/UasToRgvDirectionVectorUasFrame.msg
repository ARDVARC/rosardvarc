## Constants
# Possible values of the `rgv_id` field
int8 RGV_1 = 1
int8 RGV_2 = 2
# Possible values of the `measurement_source` field
int8 FROM_CAMERA = 10
int8 FROM_BLUETOOTH = 11

## Fields
# Time that this direction vector is for
time timestamp
# ID of the measured RGV
int8 rgv_id
# Source of this direction vector
int8 measurement_source
# The direction vector
float64[3] direction