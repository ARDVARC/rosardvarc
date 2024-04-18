"""
ARDVARC Flight Software

Author: Lyon Foster
email: lyon.foster@colorado.edu

Description:
    main loop for all fsw

Notes:
    - Some things we NEVER want to do:
     + quit the program
     + do noting unintentionally
     + error out and quit the program
     + get stuck in a loop
"""

# Do rospy first so that the node is ready when other files are imported
import rospy

# Make node for everything else
rospy.init_node("everything_else")

# local imports
from FSW.functional import determine_mission_state, guidance, estimate_rgv_state, generate_state_machine_criteria, process_bluetooth, projector

# third part imports
import numpy as np

# stl imports
import time

# Call all the setups
determine_mission_state.setup()
guidance.setup()
estimate_rgv_state.setup()
generate_state_machine_criteria.setup()
process_bluetooth.setup()
projector.setup()

# Configure MAVROS message intervals
from mavros_msgs.srv import MessageInterval, MessageIntervalRequest, MessageIntervalResponse
rospy.wait_for_service("mavros/set_message_interval")
message_interval_service = rospy.ServiceProxy("mavros/set_message_interval", MessageInterval)
attitude_message_interval_request = MessageIntervalRequest()
attitude_message_interval_request.message_id = 31
attitude_message_interval_request.message_rate = 50
attitude_message_interval_response: MessageIntervalResponse = message_interval_service.call(attitude_message_interval_request)
if not attitude_message_interval_response.success:
    rospy.logerr("Failed to set attitude message interval")
local_position_message_interval_request = MessageIntervalRequest()
local_position_message_interval_request.message_id = 32
local_position_message_interval_request.message_rate = 50
local_position_message_interval_response: MessageIntervalResponse = message_interval_service.call(local_position_message_interval_request)
if not local_position_message_interval_response.success:
    rospy.logerr("Failed to set local position message interval")

# Run the estimation loop until we die
rate = rospy.Rate(2)
now = rospy.Time.now()
while not rospy.is_shutdown():
    estimate_rgv_state.publish_estimated_rgv_state()
    rate.sleep()
