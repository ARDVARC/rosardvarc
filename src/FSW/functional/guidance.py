"""
ARDVARC Flight Software

Author: Lyon Foster
email: lyon.foster@colorado.edu

Description:
    File for guidance functions

Notes:
    - 
"""

import rospy
from rosardvarc.msg import Setpoint, RegionOfInterest, EstimatedRgvState, MissionState
from geometry_msgs.msg import PoseStamped
from genpy import Time
#from ..config.topic_names import SETPOINTS, REGIONS_OF_INTEREST, ESTIMATED_RGV_STATES, MISSION_STATES, UAS_POSES, MAVROS_GPS_POS_FORTESTING 
from FSW.config.topic_names import *
import collections
from typing import Deque
from FSW.config.constants import *
# import ..config.constants as const

# # LYON used this message for testing
# from sensor_msgs.msg import NavSatFix

# Needed for io with mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# current_RGV_state_lla = NavSatFix()

# must be greater than 2 Hz however it ends up getting implemented
guidance_update_rate = rospy.Rate(20)

current_UAS_arming_state = State()

current_UAS_pose = PoseStamped()

last_req = rospy.Time.now()

# this is the setpoint that will be written
toBeWritten_setpoint = PoseStamped()

_setpoint_pub: rospy.Publisher
_estimated_rgv_state_sub: rospy.Subscriber
_mission_state_sub: rospy.Subscriber
_uas_pose_sub: rospy.Subscriber
_uas_arming_state_sub: rospy.Subscriber

_timer: rospy.Timer

offboard_start_time = None

RGV_state = EstimatedRgvState()
mission_state = MissionState()

def _UAS_arming_state_callback(msg: State):
    global current_UAS_arming_state
    current_UAS_arming_state = msg

def _estimated_rgv_state_callback(msg: EstimatedRgvState):
    rospy.logdebug("Guidance saved an estimated RGV state")
    rospy.logdebug(msg)
    global RGV_state
    RGV_state = msg

def _mission_state_callback(msg: MissionState):
    # Do some stuff to prepare calc_orbit_setpoint
    # rgv = RGV_state
    # uas = current_UAS_pose
    # t = msg.timestamp
    global mission_state
    mission_state = msg
    
def _uas_pose_callback(msg: PoseStamped):
    rospy.logdebug("Guidance saved a UAS pose")

    global current_UAS_pose
    current_UAS_pose = msg

def _timer_callback(event=None):
    global offboard_start_time
    if(rospy.is_shutdown()): # then leave bro
        return
    else:
        # bool for if we're offboard or not
        offboard_status = (current_UAS_arming_state.mode == "OFFBOARD")
        if offboard_status and offboard_start_time is None:
            offboard_start_time = rospy.Time.now()
        
        x_set, y_set, z_set = _calc_orbit_setpoint_UpLeftRight(RGV_state, current_UAS_pose, offboard_start_time, offboard_status)
        current_setpoint = PoseStamped()
        # set the fields of 
        current_setpoint.pose.position.x = x_set
        current_setpoint.pose.position.y = y_set
        current_setpoint.pose.position.z = z_set

   
        _setpoint_pub.publish(current_setpoint)
        rospy.logdebug(f"Guidance published an orbit setpoint: {current_setpoint}")

def setup():
    """
    Setup publishers and subscribers for guidance.py
    """
    
    global _setpoint_pub, _estimated_rgv_state_sub, _mission_state_sub, _uas_pose_sub, _uas_arming_state_sub
 
    # make all subs and pubs
    _setpoint_pub = rospy.Publisher(UAS_SETPOINT_LOCAL, PoseStamped, queue_size=64)
    # _estimated_rgv_state_sub = rospy.Subscriber(MAVROS_GPS_POS_FORTESTING, NavSatFix, _estimated_rgv_state_callback)
    _estimated_rgv_state_sub = rospy.Subscriber(ESTIMATED_RGV_STATES, EstimatedRgvState, _estimated_rgv_state_callback)
    _mission_state_sub = rospy.Subscriber(MISSION_STATES, MissionState, _mission_state_callback)
    _uas_pose_sub = rospy.Subscriber(UAS_POSES, PoseStamped, _uas_pose_callback)
    _uas_arming_state_sub = rospy.Subscriber(UAS_ARMING_STATE, State,_UAS_arming_state_callback)

    # Send a few setpoints before starting
    # because can't switch to offboard until after some setpoints have been given
    for i in range(100):
        if(rospy.is_shutdown()):
            break
        #TODO(LF) make this publish the current UAS state for the first however many commands
        dummy_set_point = PoseStamped()
        dummy_set_point.pose.position.x = 0
        dummy_set_point.pose.position.y = 0
        dummy_set_point.pose.position.z = 0

        rate = rospy.Rate(20)
        _setpoint_pub.publish(dummy_set_point)
        rate.sleep()
    
    _timer = rospy.Timer(rospy.Duration(0.05), _timer_callback)

def _calc_orbit_setpoint_UpLeftRight(RGV: EstimatedRgvState, UAS: PoseStamped, start_time: rospy.Time, offboard_status: bool) -> list:
    """ Calculates the orbit set point

    This function takes the RGV & UAS states, as well as time to calculate a 
    setpoint that can be given to the Pixhawk.

    Args:
        RGV: ROS message type containing RGV state information
        UAS: ROS message type containing UAS state information
        t: time elapsed since start of mission time

    Returns: 
        orbit_setpoint: Struct (TBR) containing orbit setpoint data

    Raises:
        None: Raises None at the moment (TBR)
    """
    
    now = rospy.Time.now()

    if offboard_status:
        if now.to_sec() % 20 > 5:
            setpoint = [2,0,10]
        elif now.to_sec() % 20 > 10:
            setpoint = [0,2,10]
        elif now.to_sec() % 20 > 15:
            setpoint = [-2,0,10]
        else:
            setpoint = [0,-2,10]
    else:
        setpoint = CENTER_SETPOINT

    return setpoint
