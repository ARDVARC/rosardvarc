## Constants
# Possible values of the `rgv_id` field
int8 RGV_1=1
int8 RGV_2=2

## Fields
# Time when measurement was taken, Unix time (UTC)
time timestamp
# ID of the measured RGV
int8 rgv_id
# annotated frame (int8 for now while we debug and figure out the type/size of the image)
int8 annotated_image
