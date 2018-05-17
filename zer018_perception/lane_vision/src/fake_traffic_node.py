#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import sys, select, termios, tty        # for manually moving images

'''
CAMERA NODE RUNNING AT FULL SPEED
NO IMAGE RECORDING
'''


# Node to obtain call camera data. Separate I/O pipeline
rospy.loginfo('Init Node...')


def imagePublisher():
    count = 0

    warp_pub = rospy.Publisher('traffic_image', Image, queue_size=1)
    rospy.init_node('fake_traffic_node', anonymous=True)
    rate=rospy.Rate(5)#10hz
    bridge = CvBridge()

    while not rospy.is_shutdown():
        im_count = int(count)
        path = '/home/snuzero/day2_traffic2/' + str(im_count) + '.jpg'
        traffic_img = cv2.imread(path)
        cv2.imshow('traffic_img', traffic_img)
        traffic_img = bridge.cv2_to_imgmsg(traffic_img, "bgr8")

        count +=1
        if(count==2875):
            count=0
        warp_pub.publish(traffic_img)

        k = cv2.waitKey(1) & 0xFF
        print(count)
        rate.sleep()
        if k ==27:
            break


    cv2.destroyAllWindows()



if __name__ == '__main__':
    try:
        # settings = termios.tcgetattr(sys.stdin)
        imagePublisher()
	#print('sending image')
    except rospy.ROSInterruptException:
        pass
