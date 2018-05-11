import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
from imutils import *


# define values boundaries for color
lower_yellow = np.array([15,50,100],np.uint8)
upper_yellow = np.array([40,255,255],np.uint8)
lower_white_hsv = np.array([0, 0, 200], np.uint8)
upper_white_hsv = np.array([255,100,255], np.uint8)

lower_white_rgb = np.array([190,190,190], np.uint8)
upper_white_rgb = np.array([255,255,255], np.uint8)

# hls_lower = np.array([0, 200, 0], np.uint8)
# hls_upper = np.array([255,255, 150], np.uint8)


def process_image():
    count = 100
    x_waypoint = 500
    y_waypoint = 300
    masked_img = np.zeros((600,600),dtype='uint8')
    while True:
        path = '../collected_images/5/mosaic/' + str(count) + '.jpg'
        img = cv2.imread(path)
        
        init_time = time.time()
        
        img = GaussianBlur(img, 5)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      
        print("Blur and color time: ", time.time()-init_time)
        yellow_mask = findColor(hsv_img, lower_yellow, upper_yellow)

        '''     WHITE MASK
        # white_mask_hybrid = findWhite(img, hsv_img, lower_white_rgb, upper_white_rgb, lower_white_hsv, upper_white_hsv)
        # white_mask = findColor(hsv_img, lower_white_hsv, upper_white_hsv)
        # rgb_white_mask = findColor(img, lower_white_rgb, upper_white_rgb)
        # full_white = rgb_white_mask & white_mask
        '''
        full_mask = yellow_mask         #only YELLOW For now
        
        #DILATE
        kernel = np.ones((5,5),np.uint8)
        full_mask = cv2.dilate(full_mask, kernel, iterations =1)

        edges = cv2.Canny(img, 100, 200)
        lines = cv2.HoughLinesP(edges,1,np.pi/180,10,minLineLength=30, maxLineGap=20)
        angles = []
        a =[]
        b = []
        y_0_arr = []
        y_300_arr = []
        angle_0_arr = []
        angle_300_arr = []
        #sort line list by x value
        lines = lines[lines[:,0,0].argsort()]
        
        #yellow only
        prev_y1 = lines[0,0,1]
        prev_y2 = lines[0,0,3]
        for x1,y1,x2,y2 in lines[:,0]:
            if((full_mask[y1,x1]==255) and (full_mask[y2,x2]==255)):
                angle = np.degrees(np.arctan2(y2-y1, x2-x1))
                angles.append(angle)
                prev_y1 = y1
                prev_y2 = y2
                #filter if y1-prev_y is greater than 100
                if (angle<10 and angle>-10 and abs(y1-prev_y1) < 100 and abs(y2-prev_y2)<100):
                    #print(y1-prev_y)
                    # cv2.line(img, (x1,y1), (x2,y2),(0,0,255),2)
                    cv2.circle(img, (x1,y1), 2, (255,0,0),2)
                    #cv2.circle(img, (x2,y2), 2, (255,0,0),2)
                    project_x1 = int(np.cos(np.radians(angle+90)) * 340 + x1)
                    project_y1 = int(np.sin(np.radians(angle+90)) * 340 + y1)
                    project_x2 = int(np.cos(np.radians(angle+90)) * 340 + x2)
                    project_y2 = int(np.sin(np.radians(angle+90)) * 340 + y2)
                    
                    cv2.circle(img, (project_x1,project_y1), 2, (0,0,255),2)
                    cv2.circle(img, (project_x2,project_y2), 2, (0,0,255),2)
                    cv2.line(img, (project_x1, project_y1), (project_x2, project_y2), (255,0,0),2)
                    a.append(x1)
                    a.append(x2)
                    b.append(y1)
                    b.append(y2)
                    if x1<300:
                        y_0 = int(np.tan(angle*np.pi/180)*(0-x1)+y1)
                        y_0_arr.append(y_0)

                        angle_0_arr.append(angle)
                    else:
                        y_300 = int(np.tan(angle*np.pi/180)*(300-x1)+y1)
                        y_300_arr.append(y_300)

                        angle_300_arr.append(angle)
        

        # POLYNOMIAL FITTING 4 POINTS
        if (np.size(y_0_arr) > 2 and np.size(y_300_arr)>2):
            x = [0,300,300,600]
            y = []

            y.append((int(np.median(y_0_arr))+int(np.mean(y_0_arr)))/2)
            y.append((int(np.tan(np.median(angle_0_arr) * np.pi / 180) * (300 - 0) + y[0])+int(np.tan(np.mean(angle_0_arr) * np.pi / 180) * (300 - 0) + y[0]))/2)
            y.append((int(np.median(y_300_arr))+int(np.mean(y_300_arr)))/2)
            y.append((int(np.tan(np.median(angle_300_arr) * np.pi / 180) * (600 - 300) + y[2])+int(np.tan(np.mean(angle_300_arr) * np.pi / 180) * (600 - 300) + y[2]))/2)          
            x_points = x 
            y_points = y
            coefficients = np.polyfit(x_points, y_points,2)
            polypoints = np.zeros((600,2))
            t = np.arange(0,600,1)
            f = np.poly1d(coefficients)

            polypoints[:,0]=t
            polypoints[:,1]=f(t)
            cv2.polylines(img, np.int32([polypoints]), False, (255,0,0),2)
            
            # CALCULATE AND SHOW WAYPOINT
            if(coefficients.size ==3):  #IF 2nd order:
                slope = 2*coefficients[0]
            elif(coefficients.size==2):
                slope = coefficients[0]
            else:
                slope = 0
        
            slope_inv = abs(1/slope)
            slope_inv_theta = np.arctan2(slope_inv*1.0,1.0)
            x_waypoint = 500 + np.cos(slope_inv_theta)*170
            y_waypoint = f(500) + np.sin(slope_inv_theta)*170

            mask = np.arange(0,600,1)
            mask = coefficients[0] * mask*mask + coefficients[1]*mask + coefficients[2] 
            mask = np.zeros((600,600)) + mask           #broadcast into 2d

            y_vals = np.arange(0,600,1)
            y_vals = np.broadcast_to(y_vals, (600,600)).T
            
            masked_img = np.zeros((600,600),dtype='uint8')
            masked_img[mask>y_vals] = 255

        cv2.circle(img, (int(x_waypoint), int(y_waypoint)),4,(0,0,255), 8)
        print("x waypoint:", x_waypoint)
        print("y waypoint:", y_waypoint)


        # MASKING OUTPUT IMAGE
        
        cv2.imshow('masked',masked_img)
        print("Time taken: ", time.time() - init_time)

        #cv2.imshow('yellow', full_mask)
        cv2.imshow('original', img)
        count +=1
        if cv2.waitKey(1) & 0xFF==ord('q'):
            break

if __name__=="__main__":
    process_image()
