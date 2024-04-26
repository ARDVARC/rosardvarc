## --- Topic Names ---
# ANNOTATED_CAMERA_FRAMES = "camera/annotated_frames"
CAMERA_FRAMES = "usb_cam/image_raw"
BAGGED_FRAMES = "camera/frames_to_bag"
UAS_POSES = "mavros/local_position/pose"
RECENT_RGV_SIGHTINGS = "camera/recent_rgv_sightings"
RAW_BLUETOOTH = "bluetooth/az_els"
UAS_TO_RGV_DIRECTION_VECTORS = "estimation/direction_vectors_uas"
RGV_PROJECTIONS = "estimation/rgv_local_projections"
ESTIMATED_RGV_STATES = "estimation/estimated_rgv_states"
MISSION_STATES = "state_machine/mission_states"
FORCED_MISSION_STATE = "state_machine/forced"
STATE_MACHINE_CRITERIA = "state_machine/state_machine_criteria"
#TODO(LF): what is the SETPOINTS for?
SETPOINTS = "pixhawk/setpoints"
REGIONS_OF_INTEREST = "pixhawk/regions_of_interest"

UAS_ARMING_STATE = "mavros/state"
UAS_SETPOINT_LOCAL = "mavros/setpoint_position/local"

# Lyon used this for testing
MAVROS_GPS_POS_FORTESTING = "mavros/global_position/global"

## --- Service Names ---

# clients/services
CLIENT_ARMING = "/mavros/cmd/arming"
CLIENT_SET_MODE = "/mavros/set_mode"

FORCE_MISSION_STATE_SERVICE = "/state_machine/force_mission_state"