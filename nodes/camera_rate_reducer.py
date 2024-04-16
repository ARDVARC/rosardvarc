import rospy
from FSW.config.topic_names import CAMERA_FRAMES, BAGGED_FRAMES
from sensor_msgs.msg import Image


_previous_frame_time: float = rospy.Time.now().to_sec()
_desired_rate: float = 5 # Hz
_DESIRED_FRAME_GAP = 1/_desired_rate

def frame_callback(msg: Image):
    global _previous_frame_time
    if msg.header.stamp.to_sec() - _DESIRED_FRAME_GAP > _previous_frame_time:
        slow_frame_pub.publish(msg)
        _previous_frame_time += _DESIRED_FRAME_GAP


rospy.init_node("camera_rate_reducer")
slow_frame_pub = rospy.Publisher(BAGGED_FRAMES, Image, queue_size=64)
fast_frame_sub = rospy.Subscriber(CAMERA_FRAMES, Image, frame_callback)

rospy.spin()