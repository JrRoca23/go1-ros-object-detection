#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    cv2.imshow("Camera", cv_image)
    cv2.waitKey(1)

rospy.init_node('camera_viewer')
rospy.Subscriber('/camera/color/image_raw', Image, callback)
rospy.spin()
