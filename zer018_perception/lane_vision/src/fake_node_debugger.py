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

MYCOMPUTER = True

def imagePublisher():
    count = 0

    warp_pub = rospy.Publisher('warped_image', Image, queue_size=1)
    rospy.init_node('camera_node_warp', anonymous=True)
    rate=rospy.Rate(20)#10hz
    bridge = CvBridge()

    while not rospy.is_shutdown():
        if MYCOMPUTER:
            path = '/home/snuzero/day3_uturn2/' + str(count) + '.jpg'
        else: 
            path = '/home/snuzero/day2_park1/' + str(count) + '.jpg'
        summed_image = cv2.imread(path)
        cv2.imshow('summed_image', summed_image)
        summed_image = bridge.cv2_to_imgmsg(summed_image, "bgr8")
        
        key = getKey()
        if key == 'a':
            count -=1
            rospy.loginfo("File number: " + str(count))
            warp_pub.publish(summed_image)
        elif key == 'd':
            count +=1
            rospy.loginfo("File number: " + str(count))
            warp_pub.publish(summed_image)

        # count +=1
        if(count==1707):
            count=0

        k = cv2.waitKey(1) & 0xFF
        rate.sleep()
        if k ==27:
            break

    cv2.destroyAllWindows()

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
    try:
        settings = termios.tcgetattr(sys.stdin)
        imagePublisher()
	#print('sending image')
    except rospy.ROSInterruptException:
        pass
