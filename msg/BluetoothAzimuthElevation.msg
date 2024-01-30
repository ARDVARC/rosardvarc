## Constants
# Possible values of the `rgv_id` field
int8 RGV_1=1
int8 RGV_2=2

## Fields
# Time when measurement was taken, Unix time (UTC)
time timestamp
# ID of the measured RGV
int8 rgv_id
# Azimuth angle, value range TBD, units TBD, also maybe not technically azimuth
float64 azimuth
# Elevation angle, value range TBD, units TBD, also maybe not technically azimuth
float64 elevation
# TBD/TODO: Should this include RSSI? If so, will the RSSI have been processed into a distance by this point?