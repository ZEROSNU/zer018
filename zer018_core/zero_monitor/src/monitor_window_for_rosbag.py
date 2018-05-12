#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback(msg):
    img = bridge.imgmsg_to_cv2(msg, "rgb8")
    cv2.imshow('monitor', img)
    cv2.waitKey(1)

rospy.init_node('monitor_window_for_rosbag', anonymous=True)
sub = rospy.Subscriber('/monitor_rosbag', Image, callback, queue_size=1, buff_size=2**24)
rospy.spin()
