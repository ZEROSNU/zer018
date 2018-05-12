#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
from imutils import *
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from core_msgs.msg import CenPoint

'''
BEST WORKING CODE
'''

# define values boundaries for color
lower_yellow = np.array([15,40,150],np.uint8)
upper_yellow = np.array([40,255,255],np.uint8)
# lower_white_hsv = np.array([0, 0, 150], np.uint8)
lower_white_hsv = np.array([0,0,120], np.uint8)
upper_white_hsv = np.array([255,30,255], np.uint8)

lower_white_rgb = np.array([190,190,190], np.uint8)
upper_white_rgb = np.array([255,255,255], np.uint8)

lower_blue = np.array([90,50, 120])
upper_blue = np.array([140,255,255])

# hls_lower = np.array([0, 200, 0], np.uint8)
# hls_upper = np.array([255,255, 150], np.uint8)

coeff_buffer = []
outlier_count = 0
x_waypoint_buffer = []
y_waypoint_buffer = []

def findConfidence(below_200, below_100, nolane, linearfit, small_data, previous_coeff):
    if (not below_200):
        if (previous_coeff):
            conf1 = 5
        else:
            conf1 = 10
    else:
        conf1 = 3
    
    if below_100:
        if below_200:
            conf2 = 3
        else:
            conf2 = 6
    else:
        conf2 = 10
    
    if nolane:
        conf3 = 3
    else:
        conf3 = 10
    
    if linearfit:
        conf4 = 6
    else:
        conf4 = 10
    
    if small_data:
        conf5 = 3
    else:
        conf5 = 10
    confidence = np.array([conf1, conf2, conf3, conf4, conf5])
    print(confidence)
    confidence_val = np.min(confidence)
    return confidence_val

def calculate_rsquared(x, y, f):
    yhat = f(x)
    print(len(x))
    if (len(y) != 0): 
        ybar = np.sum(y)/len(y)
        error = y - yhat
        error[error<10] = 0
        error = np.sum(error)
        ssreg = (error**2)
        sstot = np.sum((y-ybar)**2)
        rsquared = 1-(ssreg/sstot)
    else:
        rsquared = 0
    return rsquared

def imagecallback(msg):
    global coeff_buffer
    global outlier_count
    global x_waypoint_buffer
    global y_waypoint_buffer
    # INIT booleans to determine confidence!
    rsquared_below_200 = False
    rsquared_below_100 = False
    no_lane = False
    linearfit = False
    small_data = False
    use_previous_coeff = False

    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    init_time = time.time()
    img = GaussianBlur(img, 5)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # yellow_mask = findColor(hsv_img, lower_yellow, upper_yellow)
    yellow_mask = findColor(hsv_img, lower_yellow, upper_yellow)
    kernel = np.ones((9,9),np.uint8)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
  
    # DELETE 1/3 OF MASK
    new_mask = yellow_mask
    new_mask[0:200,:] = 0
    # Lane leaving the image at the bottom seems to be a major issue.
    # Correct mask image so that y values are cut if lane leaves from bottom
    lowest_row = new_mask[599,:]
    lowest_row_indexes = np.where(lowest_row==255)[0]
    print(lowest_row_indexes)
    if (len(lowest_row_indexes)!=0):
        new_mask[540:, np.min(lowest_row_indexes):] = 0

    '''
    LINE FITTING
    Use 2nd order polynomial if available (if previous values and new values are similar)
    Use Linear fitting if 2nd order condition is not met
    '''

    points_mask = np.where(new_mask>0)
    x_vals = points_mask[1]
    y_vals = points_mask[0]

    if (x_vals.size!=0):
        # USE LINEAR FITTING IF DATA SIZE IS TOO SMALL
        if(x_vals.size>2000):
            coefficients = np.polyfit(x_vals, y_vals,2)
            poly_order = 2      
        else:
            coefficients = np.polyfit(x_vals, y_vals,1)
            poly_order = 1
            small_data = True
            linearfit = True

    else:
        coefficients = np.array([0,0])
        poly_order = 1
        no_lane = True

    coeff_thresh = 0.002          # Tune value?
    if (abs(coefficients[0])>coeff_thresh):
        coefficients = np.polyfit(x_vals, y_vals,1)
        poly_order = 1
        linearfit = True

    # CALCULATE R-SQUARED FOR CONFIDENCE

    # CREATE FITTING, DRAW ON IMAGE
    polypoints = np.zeros((600,2))
    polypoints_right = np.zeros((600,2))
    t = np.arange(0,600,1)
    f = np.poly1d(coefficients)

    # CALCULATE R-SQUARED FOR CONFIDENCE
    rsquared = calculate_rsquared(x_vals, y_vals, f)

    if(rsquared<-100):
        rsquared_below_100 = True
    if (rsquared<-200):
        rsquared_below_200 = True


    # FILTERING BASED ON PREVIOUS VALUES
    if(len(coeff_buffer)<3):
        coeff_buffer.append(coefficients)
    else:
        y_prev_avg = (coeff_buffer[0][-1] + coeff_buffer[1][-1]+coeff_buffer[2][-1])/3
        if (abs(y_prev_avg - coefficients[-1]) >70) or (rsquared < -200):
            coefficients = coeff_buffer[2]
            outlier_count +=1

            use_previous_coeff = True
                
            if(outlier_count >10):
                outlier_count=0
                coeff_buffer = []
                use_previous_coeff = True
               
        else:
            coeff_buffer[0:-1] = coeff_buffer[1:3]
            coeff_buffer[2]=coefficients
            outlier_count = 0
      
    print("Outlier count: ", outlier_count)
    f = 0
    f = np.poly1d(coefficients)
    rsquared = calculate_rsquared(x_vals, y_vals, f)

    if(rsquared<-200):
        rsquared_below_200 = True

    print("coefficients: ", coefficients)
    print("R squared = ", rsquared)

    if(coefficients.size ==3):      # if 2nd order
        slopes = t*coefficients[0]*2 + coefficients[1]
    else:
        slopes = coefficients[0]

    # Create fake points for right lane
    polypoints[:,0]=t
    polypoints[:,1]=f(t)

    # find gradient at each point and shift by lane width
    lane_width = 280
    theta = np.arctan2((slopes),1.0)
    polypoints_right[:,0] = np.cos(theta-np.pi/2)*lane_width+t
    polypoints_right[:,1] = polypoints[:,1] + np.sin(theta - np.pi/2)*lane_width
    coeff_right = np.polyfit(polypoints_right[:,0], polypoints_right[:,1], poly_order)
    f_right = np.poly1d(coeff_right)
    projection_point = 300


    #array vs float
    if type(theta)==np.float64:
        x_waypoint = np.cos(theta-np.pi/2)*lane_width/2 +projection_point
        y_waypoint = np.sin(theta-np.pi/2) * lane_width/2 + f(projection_point)
    else:
        x_waypoint = np.cos(theta[projection_point]-np.pi/2)*lane_width/2 +projection_point
        y_waypoint = np.sin(theta[projection_point]-np.pi/2)*lane_width/2 +f(projection_point)

    polypoints_right[:,0] = t
    polypoints_right[:,1] = f_right(t)

    cv2.polylines(img, np.int32([polypoints]), False, (255,0,0),2)
    cv2.polylines(img, np.int32([polypoints_right]), False, (0,0,255),2)
    cv2.circle(img, (int(x_waypoint), int(y_waypoint)),2,(255,0,0),5)

    '''
    CREATE NEW MASK FOR PUBLISH, OUTSIDE LANE = WHITE
    For efficient processing, for loop is avoided.
    Create one array with y-values, and one with y=f(x) values
    Mask using numpy masking which is worlds faster than for loop
    '''
    mask = np.arange(0,600,1)       # create x values
    mask1 = np.arange(0,600,1)
    if (len(coefficients)==3):
        mask = coefficients[0] * mask*mask + coefficients[1]*mask + coefficients[2]
        mask = np.zeros((600,600)) + mask           #broadcast into 2d
    
    else:
        mask = coefficients[0]*mask + coefficients[1]
        mask = np.zeros((600,600)) + mask           #broadcast into 2d 600 by 600
        

    if (len(coeff_right)==3):
        mask1 = coeff_right[0]*mask1*mask1 + coeff_right[1]*mask1 + coeff_right[2]
        mask1 = np.zeros((600,600)) + mask1
    else:
        mask1 = coeff_right[0]*mask1+ coeff_right[1]
        mask1 = np.zeros((600,600)) + mask1

    # create 2d array with y values
    y_vals = np.arange(0,600,1)
    y_vals = np.broadcast_to(y_vals, (600,600)).T
    # boolean masking
    masked_img = np.zeros((600,600), dtype='uint8')
    if(not no_lane):
        masked_img[mask<y_vals] = 255
        masked_img[mask1>y_vals]=255

    #resize, rotate, and flip for output
    masked_img = cv2.resize(masked_img,(200,200))
    masked_img = masked_img.T
    M = cv2.getRotationMatrix2D((100,100),180,1)

    masked_img = cv2.warpAffine(masked_img,M,(200,200))
    send_img = bridge.cv2_to_imgmsg(masked_img, "mono8")
    pub.publish(send_img)

    confidence = findConfidence(rsquared_below_200, rsquared_below_100, no_lane,
                                linearfit, small_data, use_previous_coeff)

    laneinfo.x_waypoint = x_waypoint/100.
    laneinfo.y_waypoint = y_waypoint/100. - 3       # subtract by y=3m offset
    laneinfo.confidence = confidence
    pub_waypoint.publish(laneinfo)

    # add to buffer
    print("x point:", x_waypoint)
    print("y point:", y_waypoint)
    print("Confidence: ", confidence)

    print("Time: ", time.time()-init_time)
    cv2.imshow('img', img)
    # cv2.imshow('mask', masked_img)
    cv2.imshow('mask', yellow_mask)
    cv2.waitKey(1)

if __name__=="__main__":
    bridge = CvBridge()
    laneinfo = CenPoint()
    rospy.init_node('lane_detection', anonymous=True)
    pub = rospy.Publisher('/lane_map', Image, queue_size=1)
    pub_waypoint = rospy.Publisher('/waypoints', CenPoint, queue_size=1)
    sub = rospy.Subscriber('/warped_image', Image, imagecallback, queue_size=1, buff_size=2**24)
    rospy.spin()
