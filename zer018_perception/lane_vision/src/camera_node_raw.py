#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

# Node to obtain call camera data. Separate I/O pipeline
rospy.loginfo('Init Cameras...')
cam_front = cv2.VideoCapture(1)
cam_left = cv2.VideoCapture(2)
cam_right = cv2.VideoCapture(3)
cam_front.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_front.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_front.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
cam_left.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_left.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
cam_right.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_right.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))


def imagePublisher():
    front_pub = rospy.Publisher('center_cam', Image, queue_size=1)
    left_pub = rospy.Publisher('left_cam', Image, queue_size =1)
    right_pub = rospy.Publisher('right_cam', Image, queue_size = 1)
    rospy.init_node('camera_node', anonymous=True)
    #rate=rospy.Rate(30)#10hz
    bridge = CvBridge()

    while not rospy.is_shutdown():
        _, front_img = cam_front.read()
        _, left_img = cam_left.read()
        _, right_img = cam_right.read()

        cv2.imshow('image', front_img)        # for debugging purposes
        cv2.imshow('left', left_img)
        cv2.imshow('right', right_img)
        front_img = bridge.cv2_to_imgmsg(front_img, "bgr8")
        left_img = bridge.cv2_to_imgmsg(left_img, "bgr8")  #change to grayscale?
        # right_img = bridge.cv2_to_imgmsg(left_img, "bgr8")
        rospy.loginfo("images sent")
        # for debugging purposes, remove if cv2.imshow('imgae', img) is deleted
        k = cv2.waitKey(1) & 0xFF
        if k ==27:
            break

        front_pub.publish(front_img)
        left_pub.publish(left_img)
        # right_pub.publish(right_img)
    cv2.destroyAllWindows()
    cam_front.release()
    cam_left.release()
    # cam_right.release()



if __name__ == '__main__':
    try:
        imagePublisher()
	#print('sending image')
    except rospy.ROSInterruptException:
        pass
