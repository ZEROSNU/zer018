#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
from imutils import *
import rospy
from sklearn.cluster import KMeans
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from core_msgs.msg import CenPoint, ParkPoints
from geometry_msgs.msg import Vector3
import copy

'''
MAIN LOGIC FOR LANE DETECTION. 
INPUT: Image of Top-Down stitched image from 3 lane cameras
OUTPUT: 1) Lane information in core_msgs/CenPoint Format, 
        2) Free and occupied space in transformed coordinates for the path planner, in sensor_msgs/Image mono8 image format
        3) (Only when rosparam park_mode is 1) Information regarding the parking spot in core_msgs/ParkPoints format.
'''

Z_DEBUG = False


# define values boundaries for color
lower_yellow = np.array([15,40,150],np.uint8)
upper_yellow = np.array([40,255,255],np.uint8)

#DUBUGGING! GREEN (Used when working at Idea Factory)
if Z_DEBUG:
    lower_white_hsv = np.array([50, 40, 0], np.uint8) 
    upper_white_hsv = np.array([100, 255, 255], np.uint8)
else:
    lower_white_hsv = np.array([0,0,170], np.uint8)
    upper_white_hsv = np.array([255,50,255], np.uint8)

lower_white_rgb = np.array([190,190,190], np.uint8)
upper_white_rgb = np.array([255,255,255], np.uint8)

lower_blue = np.array([90,50, 120])
upper_blue = np.array([140,255,255])

lower_red1 = np.array([0, 100, 100], np.uint8)
upper_red1 = np.array([10, 255,255], np.uint8)
lower_red2 = np.array([160, 100,100], np.uint8)
upper_red2 = np.array([179, 255,255], np.uint8)

# Define some buffers for filtering
coeff_buffer = []
outlier_count = 0
x_waypoint_buffer = []
y_waypoint_buffer = []

# Define a simple logic that selects a confindence (which directly affects vehicle velocity) depending on how 
# certain we are of our lane estimation
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
    confidence_val = np.min(confidence)
    return confidence_val

# Simple method that calculates R-squared value of data
def calculate_rsquared(x, y, f):
    yhat = f(x)
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

# Applies canny edge and returns a cropped image
def parking_detect(img, coefficients):
    edges = CannyEdge(img, 100, 200)
    edges = edges[:400, :550]

    return edges

# Bulk of code. Imagecallback runs when this node receives an input from the camera node
# Processes the image accordingly and extracts useful data
# Also publishes all data
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

    '''
    --------------------------------------------------------------------
    BASIC IMAGE PROCESSING
    --------------------------------------------------------------------
    '''

    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    init_time = time.time()
    img = GaussianBlur(img, 5)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow_mask = findColor(hsv_img, lower_yellow, upper_yellow)
    white_mask = findColor(hsv_img, lower_white_hsv, upper_white_hsv)
    edges = cv2.Canny(white_mask, 100, 200)
    edges = edges[:400, :500]   #cut edges image for cleaner hough lines
    kernel = np.ones((9,9),np.uint8)
    kernel1 = np.ones((5,5), np.uint8)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel1)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel1)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_DILATE, kernel)

    red_mask1 = findColor(hsv_img, lower_red1, upper_red1)
    red_mask2 = findColor(hsv_img, lower_red2, upper_red2)
    red_mask = red_mask1 + red_mask2
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel1)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_DILATE,kernel1)

    # DELETE 1/3 OF MASK
    new_mask = yellow_mask
    new_mask[0:200,:] = 0
    # Lane leaving the image at the bottom seems to be a major issue.
    # Correct mask image so that y values are cut if lane leaves from bottom
    lowest_row = new_mask[599,:]
    lowest_row_indexes = np.where(lowest_row==255)[0]
    if (len(lowest_row_indexes)!=0):
        new_mask[540:, np.min(lowest_row_indexes):] = 0

    '''
    ---------------------------------------------------------------------------
    LINE FITTING
    Use 2nd order polynomial if available (if previous values and new values are similar)
    Use Linear fitting if 2nd order condition is not met
    ---------------------------------------------------------------------------
    '''

    points_mask = np.where(new_mask>0)
    x_vals = points_mask[1]
    y_vals = points_mask[0]

    # CREATE BOTH LINEAR AND POLY FIT. DETERMINE WHICH TO USE BASED ON DATA AND/OR MISSION

    if (x_vals.size!=0):
        coefficients_poly = np.polyfit(x_vals, y_vals,2)
        coefficients_linear = np.polyfit(x_vals, y_vals,1)
    else:
        coefficients_linear = np.array([0,0])
        coefficients_poly = np.array([0,0,0])
        poly_order = 1
        no_lane = True

    # USE LINEAR FITTING IF DATA SIZE IS TOO SMALL
    if(x_vals.size>2000):
        coefficients = coefficients_poly
        poly_order = 2
    else:
        coefficients = coefficients_linear
        poly_order = 1
        small_data = True
        linearfit = True

    coeff_thresh = 0.002          # Tune value?
    if (abs(coefficients[0])>coeff_thresh) and (poly_order == 2):
        coefficients = coefficients_linear
        poly_order = 1
        linearfit = True

    # FORCE LINEAR FITTING FOR MISSION MODES
    if rospy.get_param('park_mode') == 1:
        coefficients = coefficients_linear
        poly_order = 1

    '''
    ---------------------------------------------------------------------------
    CALCULATE R-SQUARED FOR CONFIDENCE
    ---------------------------------------------------------------------------
    '''

    # CREATE FITTING, DRAW ON IMAGE
    polypoints = np.zeros((600,2))
    polypoints_right = np.zeros((600,2))
    polypoints_left = np.zeros((600,2))
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

    # print("Outlier count: ", outlier_count)
    f = 0
    f = np.poly1d(coefficients)
    rsquared = calculate_rsquared(x_vals, y_vals, f)

    if(rsquared<-200):
        rsquared_below_200 = True

    # print("R squared = ", rsquared)


    # FORCE 3 COEFFICIENTS
    if(coefficients.size ==3):      # if 2nd order
        slopes = t*coefficients[0]*2 + coefficients[1]
    else:
        slopes = coefficients[0]
        coefficients = np.insert(coefficients,0,0)

    # LEFT LANE COEFFICIENTS
    laneinfo.a = coefficients[0]
    laneinfo.b = coefficients[1]
    laneinfo.c = coefficients[2]
    print("coefficients: ", coefficients)

    '''
    ------------------------------------------------------------------------
    CREATE RIGHT LANE (AND LEFT LANE NEW OFFSET)
    ------------------------------------------------------------------------
    '''

    # Create fake points for right lane
    polypoints[:,0]=t
    polypoints[:,1]=f(t)

    # find gradient at each point and shift by lane width
    lane_width = 280
    theta = np.arctan2((slopes),1.0)
    polypoints_right[:,0] = np.cos(theta-np.pi/2)*(lane_width-30)+t
    polypoints_right[:,1] = polypoints[:,1] + np.sin(theta - np.pi/2)*(lane_width-30)
    coeff_right = np.polyfit(polypoints_right[:,0], polypoints_right[:,1], poly_order)
    f_right = np.poly1d(coeff_right)

    # shift left lane by 0.3m inwards - helps with Path planner mode selection
    polypoints_left[:,0] = np.cos(theta-np.pi/2) * (30) + t
    polypoints_left[:,1] = polypoints[:,1] + np.sin(theta - np.pi/2) * (30)
    coeff_left = np.polyfit(polypoints_left[:,0], polypoints_left[:,1], poly_order)
    # f_left = np.poly1d(coeff_left)

    projection_point = 400
    projection_addition = 150

    #array vs float
    if rospy.get_param('park_mode')==1:
        if type(theta)==np.float64:
            x_waypoint = np.cos(theta-np.pi/2)*(lane_width/2.5) +projection_point
            y_waypoint = np.sin(theta-np.pi/2) * lane_width/2.5 + f(projection_point)
        else:
            x_waypoint = np.cos(theta[projection_point]-np.pi/2)*lane_width/4 +projection_point
            y_waypoint = np.sin(theta[projection_point]-np.pi/2)*lane_width/4 +f(projection_point)

    # elif rospy.get_param('uturn_mode')==1:
    #     if type(theta)==np.float64:
    #         x_waypoint = np.cos(theta-np.pi/2)*(lane_width/2) +projection_point
    #         y_waypoint = np.sin(theta-np.pi/2) *lane_width/2 + f(projection_point)
    #     else:
    #         x_waypoint = np.cos(theta[projection_point]-np.pi/2)*lane_width/3 +projection_point
    #         y_waypoint = np.sin(theta[projection_point]-np.pi/2)*lane_width/3 +f(projection_point)
        
    else:
        if type(theta)==np.float64:
            if abs(np.degrees(theta))>=30:
                x_waypoint = np.cos(theta-np.pi/2)*lane_width/2 +projection_point+projection_addition
                y_waypoint = np.sin(theta-np.pi/2) * lane_width/2 + f(projection_point+projection_addition)
            else:
                x_waypoint = np.cos(theta-np.pi/2)*lane_width/2 +projection_point
                y_waypoint = np.sin(theta-np.pi/2) * lane_width/2 + f(projection_point)
        else:
            if abs(np.degrees(theta[projection_point]))>=30:
                x_waypoint = np.cos(theta[projection_point+projection_addition]-np.pi/2)*lane_width/2 +projection_point+projection_addition
                y_waypoint = np.sin(theta[projection_point+projection_addition]-np.pi/2)*lane_width/2 +f(projection_point+projection_addition)
            else:
                x_waypoint = np.cos(theta[projection_point]-np.pi/2)*lane_width/2 +projection_point
                y_waypoint = np.sin(theta[projection_point]-np.pi/2)*lane_width/2 +f(projection_point)


    polypoints_right[:,0] = t
    polypoints_right[:,1] = f_right(t)

    cv2.polylines(img, np.int32([polypoints]), False, (255,0,0),2)
    cv2.polylines(img, np.int32([polypoints_right]), False, (0,0,255),2)
    cv2.circle(img, (int(x_waypoint), int(y_waypoint)),2,(255,0,0),5)

    '''
    ---------------------------------------------------------------------------
    CREATE NEW MASK FOR PUBLISH, OUTSIDE LANE = WHITE
    For efficient processing, for loop is avoided.
    Create one array with y-values, and one with y=f(x) values
    Mask using numpy masking which is worlds faster than for loop
    -----------------------------------------------------------------------------
    '''
    mask = np.arange(0,600,1)       # create x values
    mask1 = np.arange(0,600,1)
    if (len(coeff_left)==3):
        mask = coeff_left[0] * mask*mask + coeff_left[1]*mask + coeff_left[2]
        mask = np.zeros((600,600)) + mask           #broadcast into 2d

    else:
        mask = coeff_left[0]*mask + coeff_left[1]
        mask = np.zeros((600,600)) + mask           #broadcast into 2d 600 by 600


    if (len(coeff_right)==3):
        mask1 = coeff_right[0]*mask1*mask1 + coeff_right[1]*mask1 + coeff_right[2]
        mask1 = np.zeros((600,600)) + mask1
    else:
        mask1 = coeff_right[0]*mask1+ coeff_right[1]
        mask1 = np.zeros((600,600)) + mask1
        coeff_right = np.insert(coeff_right, 0, 0)

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

    '''
    ------------------------------------------------------------------------
    PARKING MODE
    ------------------------------------------------------------------------
    '''
    #PARKING
    if rospy.get_param('park_mode') == 1:
        lines = cv2.HoughLinesP(edges,1,np.pi/180,50, minLineLength=5, maxLineGap = 5)
        intersection_x = []
        intersection_y = []
        lane_angle = np.arctan2(coeff_right[1],1)

        #TODO EXCEPTION IF NO LANE IS DETECTED
        print("coeff right: ", coeff_right)
        print("lane angle: ", np.degrees(lane_angle))
        if lines is not None:
            for x1, y1, x2, y2 in lines[:,0]:
                if (white_mask[y1][x1] == 255 and white_mask[y2][x2]==255):
                    line_angle = np.arctan2((y2-y1), (x2-x1))

                    if ((line_angle > lane_angle + np.radians(-80)) and (line_angle<lane_angle+np.radians(-40))) \
                        or ((line_angle<lane_angle-np.radians(140)) and (line_angle>lane_angle+np.radians(100))):
                        m2 = (y2-y1)/(x2-x1)
                        b2 = y1 - m2 * x1
                        x_intersect = (b2-coeff_right[2])/(coeff_right[1]-m2)
                        y_intersect = m2 * x_intersect + b2
                        intersection_x.append(x_intersect)
                        intersection_y.append(y_intersect)
                        cv2.line(img, (x1, y1), (x2,y2), (0,0,0), 2)
                        cv2.circle(img, (int(x_intersect), int(y_intersect)),2,(0,0,0), 4)
        else:
            print("no lines!")

        if (len(intersection_x)>1):
            kmeans = KMeans(n_clusters = 2)
            cluster_values = np.array(list(zip(np.array(intersection_x), np.array(intersection_y))))
            kmeans = kmeans.fit(cluster_values)
            centroids = kmeans.cluster_centers_

            point1 = np.array(centroids[0,:]).astype(int)
            point2 = np.array(centroids[1,:]).astype(int)

            #always point1 larger value
            if (point1[0] < point2[0]):
                temp = copy.deepcopy(point2)
                point2 = copy.deepcopy(point1)
                point1=temp

            distance_points = np.linalg.norm(point1-point2)
            if (distance_points > 200):     # TUNE THIS VALUE 250
                cv2.circle(img, (point1[0], point1[1]), 2, (0, 255, 0), 4)
                cv2.circle(img, (point2[0], point2[1]), 2, (0, 255, 0), 4)
                point3 = np.array([(3*point2[0]+2*point1[0])/5, (3*point2[1]+2*point1[1])/5]).astype(int)
                cv2.circle(img, (point3[0], point3[1]), 2, (0, 252, 255), 4)
                if Z_DEBUG:
                    target_point_x = point3[0] + 200*np.cos(lane_angle + np.radians(-70))
                    target_point_y = point3[1] + 200*np.sin(lane_angle + np.radians(-70))
                else:
                    target_point_x = point3[0] + 200*np.cos(lane_angle + np.radians(-70))
                    target_point_y = point3[1] + 200*np.sin(lane_angle + np.radians(-70))
                cv2.circle(img, (int(target_point_x), int(target_point_y)), 4, (0,0,255), 8)

                #publish data
                parkinfo.initpoints = [Vector3(point1[0]/100., (point1[1]-300)/100., 0), \
                                         Vector3(point2[0]/100., (point2[1]-300)/100., 0)]
                parkinfo.goal_point = Vector3(target_point_x/100., (target_point_y-300)/100., 0)
                parkinfo.header.stamp = rospy.Time.now() 
                pub_parkpoints.publish(parkinfo)

    if rospy.get_param('/uturn_mode') ==1:
        red_mask_roi = red_mask[:,400:450]
        red_mask_roi = red_mask_roi[red_mask_roi==255]
        if (red_mask_roi.size > 10000):
            rospy.set_param('/uturn_mode', 2)
            print("UTURN MODE")


    '''
    ---------------------------------------------------------------------------
    PUBLISH LANE INFO
    ---------------------------------------------------------------------------
    '''

    if not no_lane:
        laneinfo.x_waypoint = x_waypoint/100.
        laneinfo.y_waypoint = y_waypoint/100. - 3       # subtract by y=3m offset
        laneinfo.confidence = confidence
        laneinfo.header.stamp = rospy.Time.now()
        pub_waypoint.publish(laneinfo)
    else: 
        if rospy.get_param('/uturn_mode')==1 or rospy.get_param('/uturn_mode')==2:
            laneinfo.x_waypoint = 3
            laneinfo.y_waypoint = 0
            laneinfo.confidence = 3
            laneinfo.header.stamp = rospy.Time.now()
            pub_waypoint.publish(laneinfo)

    # parkinfo.initpoints = [Vector3(point1[0]/100., (point1[1]-300)/100., 0), \
    #                                      Vector3(point2[0]/100., (point2[1]-300)/100., 0)]
    # parkinfo.goal_point = Vector3(300/100., -4, 0)
    # parkinfo.header.stamp = rospy.Time.now() 
    # pub_parkpoints.publish(parkinfo)
    # add to buffer
    # print("x point:", x_waypoint)
    # print("y point:", y_waypoint)
    # print("Confidence: ", confidence)

    print("Time: ", time.time()-init_time)
    # cv2.imshow('img', img)
    cv2.imshow('white mask', white_mask)

    # cv2.imshow('mask', masked_img)
    cv2.imshow('mask', yellow_mask)
    # cv2.imshow('red_mask', red_mask)
    cv2.waitKey(1)

if __name__=="__main__":
    bridge = CvBridge()
    laneinfo = CenPoint()
    parkinfo = ParkPoints()
    rospy.init_node('lane_detection', anonymous=True)
    pub = rospy.Publisher('/lane_map', Image, queue_size=1)
    pub_waypoint = rospy.Publisher('/waypoints', CenPoint, queue_size=1)
    sub = rospy.Subscriber('/warped_image', Image, imagecallback, queue_size=1, buff_size=2**24)
    pub_parkpoints = rospy.Publisher('/initial_points_for_park', ParkPoints, queue_size = 1)
    rospy.spin()
