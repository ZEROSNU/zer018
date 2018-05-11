import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
from imutils import *
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

'''
BEST WORKING CODE
'''

# define values boundaries for color
lower_yellow = np.array([15,50,100],np.uint8)
upper_yellow = np.array([40,255,255],np.uint8)
# lower_white_hsv = np.array([0, 0, 150], np.uint8)
lower_white_hsv = np.array([0,0,120], np.uint8)
upper_white_hsv = np.array([255,30,255], np.uint8)

lower_white_rgb = np.array([190,190,190], np.uint8)
upper_white_rgb = np.array([255,255,255], np.uint8)

# hls_lower = np.array([0, 200, 0], np.uint8)
# hls_upper = np.array([255,255, 150], np.uint8)


def process_image():
    count = 1
    coeff_buffer = []
    outlier_count = 0
    while True:

        ######
        # READ IMAGE AND BASIC PROCESSES
        ######

        path = '../collected_images/5/mosaic/' + str(count) + '.jpg'
        # path = '../collected_images/' + str(count) + '.jpg'
        img = cv2.imread(path)
        init_time = time.time()
        img = GaussianBlur(img, 5)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        yellow_mask = findColor(hsv_img, lower_yellow, upper_yellow)
        kernel = np.ones((9,9),np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        # Split into two to obtain max contour
        yellow_mask_1 = yellow_mask[:, 0:300]
        yellow_mask_2 = yellow_mask[:,300:600]
        edges = CannyEdge(img, 100, 200)
        lines = Hough(edges)

        # Get CONTOUR of masked image to filter out small blobs
        
        _,contours1, _ = cv2.findContours(yellow_mask_1, 1,2)
        _,contours2, _ = cv2.findContours(yellow_mask_2, 1,2)
        # create new mask images
        new_mask = np.zeros((600,600), dtype=np.uint8)
        new_mask_temp = np.zeros((600,300),dtype=np.uint8)
        if(len(contours1)!=0):
            max_cont_1 = contours1[0]
            max_area1 = 0
            max_index1=0
            for i in range(0,len(contours1)):
                area = cv2.contourArea(contours1[i])
                if (area > max_area1):
                    max_area1 = area
                    max_index1 = i
            # fill in
            cv2.drawContours(new_mask, contours1, max_index1, (255),cv2.FILLED) 
        if(len(contours2)!=0):
            max_cont_2 = contours2[0]
            max_area2 = 0
            max_index2=0
            for i in range(0,len(contours2)):
                area = cv2.contourArea(contours2[i])
                if (area > max_area2):
                    max_area2 = area  
                    max_index2 = i                 
            cv2.drawContours(new_mask_temp, contours2, max_index2, 255, cv2.FILLED)
        
        new_mask[:, 300:600]=new_mask_temp

        '''
        LINE FITTING
        Use 2nd order polynomial if available (if previous values and new values are similar)
        Use Linear fitting if 2nd order condition is not met
        '''
        points_mask = np.where(new_mask>0)
        x_vals = points_mask[1]
        y_vals = points_mask[0]
        if (x_vals.size!=0):
            coefficients = np.polyfit(x_vals, y_vals,2)
            poly_order = 2
        else:
            coefficients = np.array([0,0])
            poly_order = 1
        coeff_thresh = 0.0001           # Tune value?
        if (abs(coefficients[0])>coeff_thresh ):
            coefficients = np.polyfit(x_vals, y_vals,1)
            poly_order = 1

        # FILTERING BASED ON PREVIOUS VALUES
        if(len(coeff_buffer)<3):
            coeff_buffer.append(coefficients)
        else:
            y_prev_avg = (coeff_buffer[0][-1] + coeff_buffer[1][-1]+coeff_buffer[2][-1])/3
            if(abs(y_prev_avg - coefficients[-1]) >100):
                coefficients = coeff_buffer[2]
                outlier_count +=1
                poly_order = len(coefficients)-1
                if(outlier_count >10):
                    outlier_count=0
                    coeff_buffer = []
                
            else:
                coeff_buffer[0:-1] = coeff_buffer[1:3]
                coeff_buffer[2]=coefficients
                poly_order = len(coefficients)-1
        
        # CREATE FITTING, DRAW ON IMAGE
        polypoints = np.zeros((600,2))
        polypoints_right = np.zeros((600,2))
        t = np.arange(0,600,1)
        f = np.poly1d(coefficients)
        if(coefficients.size ==3):      # if 2nd order
            slopes = t*coefficients[0]*2 + coefficients[1]
        else:
            slopes = coefficients[0]
        
        # Create fake points for right lane
        polypoints[:,0]=t
        polypoints[:,1]=f(t)

        theta = np.arctan2((slopes),1.0)
        polypoints_right[:,0] = np.cos(theta-np.pi/2)*340+t
        polypoints_right[:,1] = polypoints[:,1] + np.sin(theta - np.pi/2)*340
        coeff_right = np.polyfit(polypoints_right[:,0], polypoints_right[:,1], poly_order)
        f_right = np.poly1d(coeff_right)

        # find gradient at each point and shift by lane width
        polypoints_right[:,0] = t
        polypoints_right[:,1] = f_right(t)

        cv2.polylines(img, np.int32([polypoints]), False, (255,0,0),2)
        cv2.polylines(img, np.int32([polypoints_right]), False, (0,0,255),2)
        # cv2.circle(img, (np.int32(polypoints_right[:,0]), np.int32(polypoints_right[:,1])),2,(255,0,0),2)
    
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
            mask1 = coeff_right[0]*mask1*mask1 + coeff_right[1]*mask1 + coeff_right[2]
            mask1 = np.zeros((600,600)) + mask1
        else:
            mask = coefficients[0]*mask + coefficients[1]
            mask = np.zeros((600,600)) + mask           #broadcast into 2d 600 by 600
            mask1 = coeff_right[0]*mask1+ coeff_right[1]
            mask1 = np.zeros((600,600)) + mask1

        # create 2d array with y values
        y_vals = np.arange(0,600,1)
        y_vals = np.broadcast_to(y_vals, (600,600)).T
        # boolean masking
        masked_img = np.zeros((600,600),dtype='uint8')
        masked_img[mask<y_vals] = 255
        masked_img[mask1>y_vals]=255

        
        print("Time: ", time.time()-init_time)

        cv2.imshow('masked lane', masked_img)
        #cv2.imshow('original', img)

        count +=1           # iterate count to go through data
        if cv2.waitKey(10) & 0xFF==ord('q'):
            break

if __name__=="__main__":
    process_image()
