#!/usr/bin/env python
# license removed for brevity

import rospy
import cv2
import numpy as np
import time
import math
from std_msgs.msg import String
import rospkg
#from core_msgs.msg import VehicleState
#from core_msgs.msg import Control


#DEBUG MODE : True / False
Z_DEBUG = True

#Minimun number of points required to be detected
DETECT_POINT_THRESHOLD = 7

#number of detected frames in PARKING_BUFFER >= DETECT_THRESHOLD : SIGN DETECTED!
BUFFER_SIZE = 5
DETECT_THRESHOLD = 3
PARKING_BUFFER = np.zeros(BUFFER_SIZE)


rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path('zero_mission')

#Parking Sign Image source
face_parking = cv2.imread(PKG_PATH + '/src/sign/parking.jpg',0)

#Define SURF feature data
surf = cv2.xfeatures2d.SURF_create(600)
surf_parking = cv2.xfeatures2d.SURF_create(200)

#Parking sign image kepoints
kp_parking, des_parking = surf_parking.detectAndCompute(face_parking,None)

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)

# FLANN parameters
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)   # or pass empty dictionary
flann = cv2.FlannBasedMatcher(index_params,search_params)


#reject_outliers(points, standard deviation x, standard deviation y)
def reject_outliers(points,threshold_x, threshold_y):
    one_point = points[0]
    points = np.array(points)

    ########## Initial filter ############
    ### reject totally outlying points ###
    threshold_init = 2.2 #standard deviation
    u1 = np.mean(points[:,0])
    s1 = np.std(points[:,0])
    filtered1 = [point for point in points if (u1 - threshold_init* s1 <= point[0] <= u1 + threshold_init * s1)]
    points = np.array(filtered1)

    if len(points) == 0:
        return [one_point]

    u2 = np.mean(points[:,1])
    s2 = np.std(points[:,1])

    filtered2 = [point for point in points if (u2 - threshold_init * s2 <= point[1] <= u2 + threshold_init * s2)]

    if len(filtered2) == 0:
        return [one_point]
    ######################################

    ############ Main filter #############
    u1 = np.mean(points[:,0])
    s1 = np.std(points[:,0])
    filtered1 = [point for point in points if (u1 - threshold_x * s1 <= point[0] <= u1 + threshold_x * s1)]
    points = np.array(filtered1)

    if len(points) == 0:
        return [one_point]

    u2 = np.mean(points[:,1])
    s2 = np.std(points[:,1])

    filtered2 = [point for point in points if (u2 - threshold_y * s2 <= point[1] <= u2 + threshold_y * s2)]

    if len(filtered2) == 0:
        return [one_point]
    ######################################

    return filtered2

def sign_detect(matches, case, face, kp_case, kp, frame):
    matchesMask = [[0,0] for i in range(len(matches))]
    is_detected = 0
    match_points = []
    for i,(m,n) in enumerate(matches):

        if m.distance < 0.7*n.distance:
            matchesMask[i]=[1,0]

            match_points.append([kp[m.trainIdx].pt[0],kp[m.trainIdx].pt[1]])

            is_detected += 1

    if Z_DEBUG:
        draw_params = dict(matchColor = (0,0,255),
                    singlePointColor = (255,0,0),
                    matchesMask = matchesMask,
                    flags = 0)
        img_show = cv2.drawMatchesKnn(face,kp_case,frame,kp,matches,None,**draw_params)


    #reject repeated points
    if len(match_points) >=2:
        match_points = np.unique(match_points, axis=0)

    #reject outliers
    if len(match_points) >= DETECT_POINT_THRESHOLD:
        sd_x = 1.7
        sd_y = 1.7
        match_points = reject_outliers(match_points,sd_x,sd_y)

    is_detected = len(match_points)

    if Z_DEBUG:
        for [x,y] in match_points:
            cv2.circle(frame,(int(x),int(y)),4,(255,0,0),4)
        print(case + " : " + str(is_detected))
        cv2.imshow(case,img_show)
        cv2.imshow(case + ' ',frame)

    #buffer
    PARKING_BUFFER[0:BUFFER_SIZE-1] = PARKING_BUFFER[1:BUFFER_SIZE]
    PARKING_BUFFER[BUFFER_SIZE-1] = is_detected
    if np.size(np.where(PARKING_BUFFER>=DETECT_POINT_THRESHOLD)) >= DETECT_THRESHOLD:
        print("parking detected!")
        '''
        Do something!
        '''



def init():

    #rospy.Subscriber("control", Control, control_data.callback)
    rospy.init_node('parking_sign', anonymous=True)
    rate = rospy.Rate(50)
    count = 1

    while True:

        while not rospy.is_shutdown():

            while True:
                init_time = time.time()

                #Image input
                path = '/home/ksg/py_ws/sample/sign/parking2/' + str(count) +  '.jpg'
                frame = cv2.imread(path,0)

                kp, des = surf.detectAndCompute(frame,None)


                matches_parking = flann.knnMatch(des_parking,des,k=2)

                sign_detect(matches_parking, "parking", face_parking, kp_parking, kp, frame)

                if Z_DEBUG:
                    print("Time taken: ", (time.time()-init_time))

                    k = cv2.waitKey(50) & 0xFF

                    if k in [27, ord('q')]:
                        rospy.signal_shutdown('Quit')
                count = count + 1
            if Z_DEBUG:
                cv2.destroyAllWindows()


            rate.sleep()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
