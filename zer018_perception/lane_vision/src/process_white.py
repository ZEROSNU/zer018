import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
from imutils import *


# define values boundaries for color
lower_yellow = np.array([15,50,100],np.uint8)
upper_yellow = np.array([40,255,255],np.uint8)
# lower_white_hsv = np.array([0, 0, 150], np.uint8)
lower_white_hsv = np.array([0,0,230], np.uint8)
upper_white_hsv = np.array([255,30,255], np.uint8)

lower_white_rgb = np.array([190,190,190], np.uint8)
upper_white_rgb = np.array([255,255,255], np.uint8)

# hls_lower = np.array([0, 200, 0], np.uint8)
# hls_upper = np.array([255,255, 150], np.uint8)


def process_image():
    count = 100
    while True:
        path = '/home/dongwan/park1/' + str(count) + '.jpg'
        img = cv2.imread(path)
        
        init_time = time.time()
        
        img = GaussianBlur(img, 5)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        white_mask = findWhite(img, hsv_img, lower_white_rgb, upper_white_rgb, lower_white_hsv, upper_white_hsv)
        yellow_mask = findColor(hsv_img, lower_white_hsv, upper_white_hsv)
        kernel = np.ones((5,5),np.uint8)
        
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        points_mask = np.where(yellow_mask>0)
        x_vals = points_mask[1]
        y_vals = points_mask[0]
        coefficients = np.polyfit(x_vals, y_vals,1)
        polypoints = np.zeros((600,2))
        t = np.arange(0,600,1)
        f = np.poly1d(coefficients)

        polypoints[:,0]=t
        polypoints[:,1]=f(t)
        cv2.polylines(img, np.int32([polypoints]), False, (255,0,0),2)
    


        cv2.imshow('white mask', yellow_mask)
        
        edges = CannyEdge(img, 50, 200)
        lines = Hough(edges)
        for x1,y1,x2,y2 in lines[:,0]:
            if (white_mask[y1,x1]==255 and white_mask[y2,x2]==255):
                cv2.line(img, (x1,y1), (x2,y2), (0,0,255), 2)
        #cv2.imshow('edges', edges)
        print("Blur and color time: ", time.time()-init_time)

        cv2.imshow('original', img)


        count +=1
        if cv2.waitKey(10) & 0xFF==ord('q'):
            break

if __name__=="__main__":
    process_image()
