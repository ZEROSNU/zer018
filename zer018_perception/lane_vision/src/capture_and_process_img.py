#!/usr/bin/env python
import rospy
import cv2
import numpy as np
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
import time
from std_msgs.msg import String

def findcolor(raw_img):
    hsv_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0,0,100],np.uint8)
    upper_white = np.array([180,100,255],np.uint8)
    lower_yellow = np.array([15,50,100],np.uint8)
    upper_yellow = np.array([40,255,255],np.uint8)

    maskyellow = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
    return maskyellow

def main():
    rospy.loginfo("Initializing Camera and Publisher")
    rospy.init_node('center_cam_node')
    imgPub = rospy.Publisher('center_cam_info', String, queue_size=1)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    rospy.loginfo("Camera initialization complete!")

    while True: 
        _, frame = cap.read()
        cv2.imshow('original', frame)
        yellow_mask = findcolor(frame)
        cv2.imshow('mask', yellow_mask)
        imgPub.publish("img processed!")
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows
    cap.release

if __name__ == '__main__':
    try:
        main()
	#print('sending image')
    except rospy.ROSInterruptException:
        pass