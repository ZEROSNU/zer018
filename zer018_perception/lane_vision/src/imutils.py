import numpy as np
import cv2

'''
UTILITIES SCRIPT for frequenty reused functions
'''

def GaussianBlur(img, kernel_size):
    blur_n = 5 # blur_n : 5 or higher
    blurred_img = cv2.GaussianBlur(img,(kernel_size,kernel_size),0)
    return blurred_img

# Masking color
def findColor(hsv_image, lower, upper):
    mask = cv2.inRange(hsv_image, lower, upper)
    return mask

def findWhite(rgb_image, hsv_image, rgb_lower, rgb_upper, hsv_lower, hsv_upper):
    rgb_mask = cv2.inRange(rgb_image, rgb_lower, rgb_upper)
    hsv_mask = cv2.inRange(hsv_image, hsv_lower, hsv_upper)
    mask = rgb_mask & hsv_mask
    return mask

def CannyEdge(img, low_threshold, high_threshold):
    img = cv2.Canny(img, low_threshold, high_threshold)
    return img

def Hough(img):
    lines = cv2.HoughLinesP(img,1,np.pi/180,10,minLineLength=10, maxLineGap=50)
    return lines