## Constants
# Possible values of the `rgv_id` field
int8 RGV_1 = 1
int8 RGV_2 = 2

## Fields
# Time that this estimate is for
time timestamp
# ID of the measured RGV
int8 rgv_id
# Estimated position of the RGV
float64[3] position
# Estimated velocity of the RGV
float64[3] velocity