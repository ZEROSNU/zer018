#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import os

'''
CAMERA NODE RUNNING AT 10HZ
RECORD ALL IMAGES
'''

# Node to obtain call camera data. Separate I/O pipeline
rospy.loginfo('Init Cameras...')
cam_front = cv2.VideoCapture(1)

cam_front.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
cam_front.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_front.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))


current_time = str(time.time())
path = "/home/snuzero/sign"+current_time + "/"
os.mkdir(path)
# path_left = "../collected_images/"+current_time+"/left"
# path_right = "../collected_images/"+current_time+"/right"
# path_center = "../collected_images/"+current_time+"/center"
# os.mkdir(path_left)
# os.mkdir(path_right)
# os.mkdir(path_center)

def imagePublisher():
    warp_pub = rospy.Publisher('warped_image', Image, queue_size=1)
    rospy.init_node('camera_node_warp', anonymous=True)
    rate=rospy.Rate(10)#10hz
    bridge = CvBridge()

    count = 0
    while not rospy.is_shutdown():
        _, front_img = cam_front.read()

        pathname_warp = path + "/" + str(count) + ".jpg"
        cv2.imwrite(pathname_warp, front_img)
        # cv2.imwrite(pathname_left, left_img)
        # cv2.imwrite(pathname_right, right_img)
        # cv2.imwrite(pathname_front, front_img)
        # summed_image = bridge.cv2_to_imgmsg(summed_image, "bgr8")
        rospy.loginfo("images sent")
        cv2.imshow('image', front_img)
        count = count+1
        k = cv2.waitKey(1) & 0xFF
        if k ==27:
            break
        # warp_pub.publish(summed_image)
        rate.sleep()
    cv2.destroyAllWindows()
    cam_front.release()



if __name__ == '__main__':
    try:
        imagePublisher()
	#print('sending image')
    except rospy.ROSInterruptException:
        pass
