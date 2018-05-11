#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import os
'''
CAMERA NODE RUNNING AT FULL SPEED
NO IMAGE RECORDING
'''

# define values boundaries for color
lower_yellow = np.array([15,50,100],np.uint8)
upper_yellow = np.array([40,255,255],np.uint8)
# lower_white_hsv = np.array([0, 0, 150], np.uint8)
lower_white_hsv = np.array([0,0,120], np.uint8)
upper_white_hsv = np.array([255,30,255], np.uint8)

lower_white_rgb = np.array([190,190,190], np.uint8)
upper_white_rgb = np.array([255,255,255], np.uint8)


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


homography_front = np.array([[1.20578316901691e-05, 0.000329407217584187, -0.654631511346573],
        [0.000956007807820993, -0.00231196866646363, -0.755942976367266],
        [4.59523223437821e-08, -7.58289486150618e-06, -0.00119706768797383]])
homography_left = np.array([[0.000718031591780952, -0.000165635757963769, 0.0810412365295545],
        [-0.000457696149644549, 0.00197605152199676, 0.996708002646204],
        [3.29970074027985e-07, 7.21616117600935e-06, 0.000904500215204792]])
homography_right = np.array([[0.000915698708180926, 0.000160507016324320, -0.989937462884797],
        [0.000940331110058012, -0.00341128707549284, -0.141453165043701],
        [2.16613893241818e-07, -1.07504356915543e-05, -0.00119841227368443]])



def warp_image(image, homography):
    im_out = cv2.warpPerspective(image, homography, (600,600))
    return im_out

def find_mask(image):
    black_range1 = np.array([0,0,0])
    im_mask = (cv2.inRange(image, black_range1, black_range1)).astype('bool')
    im_mask_inv = (1-im_mask).astype('bool')
    im_mask_inv = np.dstack((im_mask_inv, im_mask_inv, im_mask_inv))
    im_mask= np.dstack((im_mask, im_mask, im_mask))
    return im_mask_inv, im_mask


def imagePublisher():
    current_time = str(time.time())
    path = "/home/snuzero/"+current_time
    # os.mkdir(path)
    warp_pub = rospy.Publisher('warped_image', Image, queue_size=1)
    rospy.init_node('camera_node_warp', anonymous=True)
    #rate=rospy.Rate(30)#10hz
    bridge = CvBridge()

    while not rospy.is_shutdown():
        _, front_img = cam_front.read()
        _, left_img = cam_left.read()
        _, right_img = cam_right.read()

        init_time = time.time()
        im_front = warp_image(front_img, homography_front).astype('uint8')
        im_left = warp_image(left_img, homography_left).astype('uint8')
        im_right = warp_image(right_img, homography_right).astype('uint8')

        im_mask_inv, im_mask = find_mask(im_front)
        front_masked = np.multiply(im_front, im_mask_inv).astype('uint8')
        left_masked = np.multiply(im_left, im_mask).astype('uint8')
        right_masked = np.multiply(im_right, im_mask).astype('uint8')
        summed_image = front_masked + left_masked+right_masked

        cv2.imshow('warped', summed_image)
        pathname_warp = path + "/" + str(time.time()) + ".jpg"
        # cv2.imwrite(pathname_warp, summed_image)
        summed_image = bridge.cv2_to_imgmsg(summed_image, "bgr8")
        rospy.loginfo("images sent")
        print("Time taken: ", time.time() -init_time)
        k = cv2.waitKey(1) & 0xFF
        if k ==27:
            break

        warp_pub.publish(summed_image)
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
