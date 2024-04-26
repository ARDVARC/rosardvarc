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
import collections
from typing import Deque
from FSW.config.topic_names import *
from FSW.config.constants import *


# Needed for io with mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


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

# timers for making the setpoint (happens slow) and writing the setpoint (happens faster)
_create_setpoint_timer: rospy.Timer
_publish_setpoint_timer: rospy.Timer

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
    global mission_state
    mission_state = msg


def _uas_pose_callback(msg: PoseStamped):
    rospy.logdebug("Guidance saved a UAS pose")

    global current_UAS_pose
    current_UAS_pose = msg


def _create_setpoint_timer_callback(event=None):
    global toBeWritten_setpoint

    case = mission_state.mission_state
    if case == mission_state.FIND_RGV_1:
        x_set, y_set, z_set = _calc_orbit_setpoint_find(
            mission_state, RGV_state, current_UAS_pose
        )

    elif case == mission_state.FIND_RGV_2:
        x_set, y_set, z_set = _calc_orbit_setpoint_find(
            mission_state, RGV_state, current_UAS_pose
        )

    elif case == mission_state.TRACK_RGV_1:
        x_set, y_set, z_set = _calc_orbit_setpoint_track(
            mission_state, RGV_state, current_UAS_pose
        )

    elif case == mission_state.TRACK_RGV_2:
        x_set, y_set, z_set = _calc_orbit_setpoint_track(
            mission_state, RGV_state, current_UAS_pose
        )

    elif case == mission_state.LOCALIZE_RGV_1:
        x_set, y_set, z_set = _calc_orbit_setpoint_localize(
            mission_state, RGV_state, current_UAS_pose
        )

    elif case == mission_state.LOCALIZE_RGV_2:
        x_set, y_set, z_set = _calc_orbit_setpoint_localize(
            mission_state, RGV_state, current_UAS_pose
        )

    elif case == mission_state.JOINT_LOCALIZE:
        x_set, y_set, z_set = _calc_orbit_setpoint_joint(
            mission_state, RGV_state, current_UAS_pose
        )

    else:
        rospy.logdebug("null setpoint returned")
        x_set, y_set, z_set = CENTER_SETPOINT

    toBeWritten_setpoint = PoseStamped()
    # set the fields of
    toBeWritten_setpoint.pose.position.x = x_set
    toBeWritten_setpoint.pose.position.y = y_set
    toBeWritten_setpoint.pose.position.z = z_set
    
    orientation = Rotation.from_euler("Z", (rospy.Time.now().to_sec() * CONSTANT_YAW_RATE) % 360, True).as_quat()
    toBeWritten_setpoint.pose.orientation.x = orientation[0]
    toBeWritten_setpoint.pose.orientation.y = orientation[1]
    toBeWritten_setpoint.pose.orientation.z = orientation[2]
    toBeWritten_setpoint.pose.orientation.w = orientation[3]



def _publish_setpoint_timer_callback(event=None):
    if rospy.is_shutdown():  # then leave bro
        return
    else:
        _setpoint_pub.publish(toBeWritten_setpoint)
        rospy.logdebug(f"Guidance published an orbit setpoint: {toBeWritten_setpoint}")


def setup():
    """
    Setup publishers and subscribers for guidance.py
    """

    global _setpoint_pub, _estimated_rgv_state_sub, _mission_state_sub, _uas_pose_sub, _uas_arming_state_sub

    # make all subs and pubs
    _setpoint_pub = rospy.Publisher(UAS_SETPOINT_LOCAL, PoseStamped, queue_size=64)
    _estimated_rgv_state_sub = rospy.Subscriber(
        ESTIMATED_RGV_STATES, EstimatedRgvState, _estimated_rgv_state_callback
    )
    _mission_state_sub = rospy.Subscriber(
        MISSION_STATES, MissionState, _mission_state_callback
    )
    _uas_pose_sub = rospy.Subscriber(UAS_POSES, PoseStamped, _uas_pose_callback)
    _uas_arming_state_sub = rospy.Subscriber(
        UAS_ARMING_STATE, State, _UAS_arming_state_callback
    )

    # Send a few setpoints before starting
    # because can't switch to offboard until after some setpoints have been given
    for i in range(100):
        if rospy.is_shutdown():
            break
        # TODO(LF) make this publish the current UAS state for the first however many commands
        dummy_set_point = PoseStamped()
        dummy_set_point.pose.position.x = 0
        dummy_set_point.pose.position.y = 0
        dummy_set_point.pose.position.z = 0

        rate = rospy.Rate(20)
        _setpoint_pub.publish(dummy_set_point)
        rate.sleep()

    _create_setpoint_timer = rospy.Timer(
        rospy.Duration(CREATE_SETPOINT_RATE), _create_setpoint_timer_callback
    )
    _publish_setpoint_timer = rospy.Timer(
        rospy.Duration(PUBLISH_SETPOINT_RATE), _publish_setpoint_timer_callback
    )


def _calc_orbit_setpoint_find(
    mission_state: MissionState, RGV: EstimatedRgvState, UAS: PoseStamped
) -> list:
    setpoint = [CENTER_SETPOINT[0], CENTER_SETPOINT[1], UAS_ALTITUDE_SETPOINT]
    return setpoint


def _calc_orbit_setpoint_joint(
    mission_state: MissionState, RGV: EstimatedRgvState, UAS: PoseStamped
) -> list:

    x_dist = RGV.rgv1_position_local[0] - RGV.rgv2_position_local[0]
    y_dist = RGV.rgv1_position_local[1] - RGV.rgv2_position_local[1]
    z_dist = RGV.rgv1_position_local[2] - RGV.rgv2_position_local[2]
    distance_between = np.linalg.norm([x_dist, y_dist, z_dist])

    half_cone_altitude = (distance_between/2) / np.tan(CAM_MIN_FOV / 2)

    now = rospy.Time.now().to_sec()

    switch_rate  = int((now % JOINT_ORBITAL_PERIOD) // JOINT_TIME_AT_ORBIT_POINT)

    if half_cone_altitude < MAX_ARUCO_RESOLVABLE_DISTANCE:
        mid_x = (RGV.rgv1_position_local[0] + RGV.rgv2_position_local[0]) / 2.0
        mid_y = (RGV.rgv1_position_local[1] + RGV.rgv2_position_local[1]) / 2.0
        
        # this is to keep the uas from going too low
        altitude_setpoint = max(half_cone_altitude, UAS_ALTITUDE_SETPOINT_JOINT)
        setpoint = [mid_x, mid_y, altitude_setpoint]

    else:
        if switch_rate == 0:
            setpoint = [RGV.rgv1_position_local[0], RGV.rgv1_position_local[1], UAS_ALTITUDE_SETPOINT_JOINT]
        elif switch_rate == 1:
            setpoint = [RGV.rgv2_position_local[0], RGV.rgv2_position_local[1], UAS_ALTITUDE_SETPOINT_JOINT]

    return setpoint


def _calc_orbit_setpoint_track(
    mission_state: MissionState, RGV: EstimatedRgvState, UAS: PoseStamped
) -> list:

    if mission_state.mission_state == mission_state.TRACK_RGV_1:
        orbit_center = RGV.rgv1_position_local
    elif mission_state.mission_state == mission_state.TRACK_RGV_2:
        orbit_center = RGV.rgv2_position_local
    else:
        orbit_center = RGV.rgv1_position_local

    x_c = orbit_center[0]
    y_c = orbit_center[1]

    setpoint = [x_c, y_c, UAS_ALTITUDE_SETPOINT]

    return setpoint


def _calc_orbit_setpoint_localize(
    mission_state: MissionState, RGV: EstimatedRgvState, UAS: PoseStamped
) -> list:

    if mission_state.mission_state == mission_state.LOCALIZE_RGV_1:
        orbit_center = RGV.rgv1_position_local
    elif mission_state.mission_state == mission_state.LOCALIZE_RGV_2:
        orbit_center = RGV.rgv2_position_local
    else:
        orbit_center = RGV.rgv1_position_local

    x_c = orbit_center[0]
    y_c = orbit_center[1]

    now = rospy.Time.now().to_sec()

    cardinal = int((now % ORBITAL_PERIOD) // TIME_AT_ORBIT_POINT)

    if cardinal == 0:  # positive East
        setpoint = [x_c + ORBITAL_RADIUS_SINGLE, y_c, UAS_ALTITUDE_SETPOINT]
    elif cardinal == 1:  # positive North
        setpoint = [x_c, y_c + ORBITAL_RADIUS_SINGLE, UAS_ALTITUDE_SETPOINT]
    elif cardinal == 2:  # West
        setpoint = [x_c - ORBITAL_RADIUS_SINGLE, y_c, UAS_ALTITUDE_SETPOINT]
    elif cardinal == 3:  # South
        setpoint = [x_c, y_c - ORBITAL_RADIUS_SINGLE, UAS_ALTITUDE_SETPOINT]
    else:
        rospy.logdebug("null setpoint returned")
        setpoint = CENTER_SETPOINT

    return setpoint
