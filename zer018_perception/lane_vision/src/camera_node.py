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
homography_front = np.array([[-2.70228915e-05, 2.65327952e-04, -8.40851775e-01], 
       [ 1.23350874e-03, -3.09331040e-03, -9.02184303e-01], 
       [-6.07198793e-08, -1.03679613e-05, -1.20307233e-03]]) 

homography_left = np.array([[ 9.52913526e-04, -1.54462293e-04, 1.34050016e-01], 
       [-6.76484134e-04, 2.63203439e-03, 1.30538654e+00], 
       [ 3.00904722e-07, 9.73186701e-06, 1.04804444e-03]]) 

homography_right = np.array([[-8.37207792e-04, -8.42444368e-05, 9.88391547e-01], 
       [-1.03281920e-03, 3.30718896e-03, 2.38351581e-02], 
       [-3.88972127e-07, 1.08066739e-05, 5.85856759e-04]])



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
    warp_pub = rospy.Publisher('warped_image', Image, queue_size=1)
    traffic_pub = rospy.Publisher('traffic_image', Image, queue_size=1)
    rospy.init_node('camera_node_warp', anonymous=True)
    rate=rospy.Rate(20)#10hz
    bridge = CvBridge()

    while not rospy.is_shutdown():
        _, front_img = cam_front.read()
        _, left_img = cam_left.read()
        _, right_img = cam_right.read()
        _, traffic_img = traffic_cam.read()

        im_front = warp_image(front_img, homography_front).astype('uint8')
        im_left = warp_image(left_img, homography_left).astype('uint8')
        im_right = warp_image(right_img, homography_right).astype('uint8')

        im_mask_inv, im_mask = find_mask(im_front)
        front_masked = np.multiply(im_front, im_mask_inv).astype('uint8')
        left_masked = np.multiply(im_left, im_mask).astype('uint8')
        right_masked = np.multiply(im_right, im_mask).astype('uint8')
        summed_image = front_masked + left_masked+right_masked
        summed_image = bridge.cv2_to_imgmsg(summed_image, "bgr8")
        traffic_img = bridge.cv2_to_imgmsg(traffic_img, 'bgr8')
        rospy.loginfo("images sent")

        # PUBLISH
        warp_pub.publish(summed_image)
        traffic_pub.publish(traffic_img)
    cv2.destroyAllWindows()
    cam_front.release()
    cam_left.release()
    cam_right.release()
    traffic_cam.release()

if __name__ == '__main__':
    try:
        # Node to obtain call camera data. Separate I/O pipeline
        rospy.loginfo('Init Cameras...')
        while True:
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
            traffic_cam = cv2.VideoCapture(4)
            traffic_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
            traffic_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            traffic_cam.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

            ret1, front_img = cam_front.read()
            ret2, left_img = cam_left.read()
            ret3, right_img = cam_right.read()
            ret4, traffic_img = traffic_cam.read()

            if (ret1 and ret2 and ret3):
                print("All cameras connected!")
                break
            else:
                print("Connection error! retrying...")
                cam_front.release()
                cam_left.release()
                cam_right.release()
                traffic_cam.release()


        imagePublisher()
	#print('sending image')
    except rospy.ROSInterruptException:
        pass
