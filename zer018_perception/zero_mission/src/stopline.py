#!/usr/bin/env python
# license removed for brevity

import rospy
import cv2
import numpy as np
import time
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from core_msgs.msg import Estop
from cv_bridge import CvBridge, CvBridgeError
import rospkg



#DEBUG MODE 
Z_DEBUG = False
Z_SEND_ESTOP = True
Z_BLUE_MASKING = False
Z_VIEW_TIMETAKEN = False
Z_IGNORE_FIRST_LINE = False
USE_HoughLines = False #HoughLines : True / HoughLinesP : False



lower_white_hsv = np.array([0,0,170], np.uint8)
upper_white_hsv = np.array([255,50,255], np.uint8)
stop_time_stamp = 0
first_line_toggle = False

####BLUE####!!!!!!!!!!!
if Z_BLUE_MASKING:
    lower_white_hsv = np.array([90,50,120], np.uint8)
    upper_white_hsv = np.array([140,255,255], np.uint8)
############!!!!!!!!!!!
ANGLE_THRESHOLD = 20 #angle difference (DEGREE)

MIN_DETECTED_THRESHOLD = 3 #for HoughLines
MIN_DETECTED_THRESHOLD_P = 3  

detect_count = 0
#number of detected frames in STOPLINE_BUFFER >= DETECT_THRESHOLD : STOPLINE DETECTED!  
BUFFER_SIZE = 10
DETECT_THRESHOLD = 6
STOPLINE_BUFFER = np.zeros(BUFFER_SIZE)

bridge = CvBridge()
msg = Estop()
pub = rospy.Publisher('/emergency_stop', Estop, queue_size=10)
###########################################

def init():
    
    #pub = rospy.Publisher('/emergency_stop', Estop, queue_size=10)
    rospy.Subscriber("warped_image", Image, callback)
    rospy.init_node('stopline', anonymous=True)
    #msg = Estop()
    rate = rospy.Rate(20)
 
    while True:
       
        while not rospy.is_shutdown():
            rospy.spin()

def SendEstop(msg):
    msg.estop = 1
    cnt = 1
    while(cnt <= 40):
        if Z_DEBUG:
            print "sending estop sign : " + str(cnt)
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        time.sleep(0.1)
        cnt = cnt+1
    rospy.signal_shutdown("succesfully sended ESTOP sign")

def reject_outliers(data):
    u = np.mean(data)
    s = np.std(data)
    filtered = [e for e in data if (u - 1 * s < e < u + 1 * s)]
    return filtered

def GaussianBlur(img, kernel_size):
    blur_n = 5 # blur_n : 5 or higher
    blurred_img = cv2.GaussianBlur(img,(kernel_size,kernel_size),0)
    return blurred_img

# Masking color
def findColor(hsv_image, lower, upper):
    mask = cv2.inRange(hsv_image, lower, upper)
    return mask


    
def DrawHoughLines(edge, img): #edge: edge input, img : image on which draw lines
    

    lines = cv2.HoughLines(edge,1,np.pi/180,70)  
   
    detect_count = 0
    try:
        for temp in lines:
            
            rho = temp[0][0]
            theta = temp[0][1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
          
            if np.sin(theta) < np.sin(float(ANGLE_THRESHOLD) / float(180) * np.pi):
                detect_count = detect_count + 1
                if Z_DEBUG:
                    
                    cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
                    pass
            else:
                if Z_DEBUG:
                    cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
                    pass


        if Z_DEBUG:
            print detect_count

        if detect_count >= MIN_DETECTED_THRESHOLD:
            detected = True
        else:
            detected = False

        #buffer
        STOPLINE_BUFFER[0:BUFFER_SIZE-1] = STOPLINE_BUFFER[1:BUFFER_SIZE]
        STOPLINE_BUFFER[BUFFER_SIZE-1] = detected

        return img
    except:
        return img

def DrawHoughLinesP(edge, img): #edge: edge input, img : image on which draw lines

    lines = cv2.HoughLinesP(edge, 1, np.pi / 180, 10, minLineLength=50, maxLineGap=20)
    detect_count = 0
    detected_points_x = np.array([])
    
    if np.size(lines)>1:
        for x1, y1, x2, y2 in lines[:, 0]:
            
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            if abs(np.cos(angle*np.pi/float(180))) < np.sin(float(ANGLE_THRESHOLD) / float(180) * np.pi) :
                
                detect_count = detect_count + 1
                detected_points_x = np.append(detected_points_x, [x1, x2])
                if Z_DEBUG:
                    cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
                    pass
            else:
                if Z_DEBUG:
                    cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
                    pass

        if Z_DEBUG:
            print(str(detect_count) + "points detected")
            pass
        
        if detect_count >= MIN_DETECTED_THRESHOLD_P:
            filtered_x_points = reject_outliers(detected_points_x)
            x_mean = int(np.mean(filtered_x_points))
            if(np.size(filtered_x_points) >= 4) and x_mean <= 200:
                cv2.circle(img, (x_mean, int(np.shape(img)[0]/2)),3,(255,0,0),3,1)
                detected = True
            else:
                detected = False
        else:
            detected = False

        #buffer
        STOPLINE_BUFFER[0:BUFFER_SIZE-1] = STOPLINE_BUFFER[1:BUFFER_SIZE]
        STOPLINE_BUFFER[BUFFER_SIZE-1] = detected

        return img
    else:
        return img
###########################################

def callback(data):
    init_time = time.time()
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    img = img[150:450,150:400]

 
    img = GaussianBlur(img, 5)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    white_mask = cv2.inRange(hsv_img, lower_white_hsv,upper_white_hsv)
    edges = cv2.Canny(white_mask, 200, 800)

    #HoughLines vs HoughLinesP
    if USE_HoughLines:
        DrawHoughLines(edges, img)
    else:
        DrawHoughLinesP(edges,img)

    #BUFFER
    if np.size(np.where(STOPLINE_BUFFER == True)) >= DETECT_THRESHOLD:
        if Z_DEBUG:
            print("stopline detected!")

        if Z_SEND_ESTOP:
            if Z_IGNORE_FIRST_LINE:
                global stop_time_stamp
                global first_line_toggle

                if stop_time_stamp == 0:
                    stop_time_stamp = rospy.Time.now().to_sec()
                elif abs(stop_time_stamp - rospy.Time.now().to_sec()) < 0.3:
                    if first_line_toggle == False:
                        stop_time_stamp = rospy.Time.now().to_sec()
                        pass
                    elif first_line_toggle == True:
                        SendEstop(msg)
                else:
                    stop_time_stamp = 0
                    first_line_toggle = True
            else:
                SendEstop(msg)

    if Z_DEBUG:
        cv2.imshow('img', img)
        cv2.imshow('white mask', white_mask)
        k = cv2.waitKey(30) & 0xFF
        if Z_VIEW_TIMETAKEN:
            print("Time taken: ", (time.time()-init_time))


    pass




if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
