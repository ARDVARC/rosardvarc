import rospy
from FSW.config.topic_names import CAMERA_FRAMES
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

rospy.init_node("camera_rate_reducer")
frame_pub = rospy.Publisher(CAMERA_FRAMES, Image, queue_size=64)

bridge = CvBridge()
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print(":'(")
    exit()

while not rospy.is_shutdown():
    ret, frame = cap.read()
    
    if not ret:
        print("No frame?")
    
    msg = bridge.cv2_to_imgmsg(frame)
    
    frame_pub.publish(msg)