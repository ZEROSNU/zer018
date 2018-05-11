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


# homography_front = np.array([[3.32307728204869e-05, 0.000347100641779136, -0.626715469765621],
#         [0.000920258752671037, -0.00352026028546193, -0.779239131081207],
#         [8.93784116716917e-08, -6.96518431402588e-06, -0.000857792488071616]])

# homography_left = np.array([[0.000641768674232369, -0.000116493692310499, 0.0421975278957785],
#         [-0.000493009230079965, 0.00297842233622114, 0.999104231770187],
#         [-8.52350569431273e-08, 6.30630859452040e-06, 0.000750441928321108]])

# homography_right = np.array([[-0.000862512692554985, -0.000177613001757401, 0.949494373350685],
#         [-0.000813905209170013, 0.00539413377532179, 0.313733752493735],
#         [1.60444443430264e-07, 1.03899757941646e-05, 0.00101627664219164]])

current_time = str(time.time())
path = "/home/snuzero/"+current_time
os.mkdir(path)
# path_left = "../collected_images/"+current_time+"/left"
# path_right = "../collected_images/"+current_time+"/right"
# path_center = "../collected_images/"+current_time+"/center"
# os.mkdir(path_left)
# os.mkdir(path_right)
# os.mkdir(path_center)

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
    rospy.init_node('camera_node_warp', anonymous=True)
    rate=rospy.Rate(10)#10hz
    bridge = CvBridge()

    count = 0
    while not rospy.is_shutdown():
        _, front_img = cam_front.read()
        _, left_img = cam_left.read()
        _, right_img = cam_right.read()

        im_front = warp_image(front_img, homography_front).astype('uint8')
        im_left = warp_image(left_img, homography_left).astype('uint8')
        im_right = warp_image(right_img, homography_right).astype('uint8')

        im_mask_inv, im_mask = find_mask(im_front)
        front_masked = np.multiply(im_front, im_mask_inv).astype('uint8')
        left_masked = np.multiply(im_left, im_mask).astype('uint8')
        right_masked = np.multiply(im_right, im_mask).astype('uint8')
        summed_image = front_masked + left_masked+right_masked

        #cv2.imshow('warped', summed_image)
        #record_time = str(time.time())
        # pathname_warp = '../collected_images/warped_images/'+record_time+'.jpg'
        # pathname_left = '../collected_images/left_images/'+record_time+'.jpg'
        # pathname_right = '../collected_images/right_images/'+record_time+'.jpg'
        # pathname_front = '../collected_images/front_images/'+record_time+'.jpg'
        # pathname_left = path_left + "/" +str(count) + ".jpg"
        # pathname_right = path_right + "/" +str(count) + ".jpg"
        # pathname_front = path_center + "/" +str(count) + ".jpg"
        #cv2.imwrite(pathname_warp, summed_image)
        pathname_warp = path + "/" + str(count) + ".jpg"
        cv2.imwrite(pathname_warp, summed_image)
        # cv2.imwrite(pathname_left, left_img)
        # cv2.imwrite(pathname_right, right_img)
        # cv2.imwrite(pathname_front, front_img)
        # summed_image = bridge.cv2_to_imgmsg(summed_image, "bgr8")
        rospy.loginfo("images sent")
        count = count+1
        k = cv2.waitKey(1) & 0xFF
        if k ==27:
            break
        # warp_pub.publish(summed_image)
        rate.sleep()
    cv2.destroyAllWindows()
    cam_front.release()
    cam_left.release()
    cam_right.release()



if __name__ == '__main__':
    try:
        imagePublisher()
	#print('sending image')
    except rospy.ROSInterruptException:
        pass
