import cv2
import numpy as np
import time

'''
TEST FILE using 1000, 1000 output image.
Actual code will have an output image of 200,200, which also means a different homography
'''

#recalculated homography

# homography_front = np.array([[3.12570133882145e-05, 0.000286172662353515, -0.680179732686621],
#         [0.000967963380750764,-0.00220708598330688,-0.733040431894039],
#         [9.31003590466217e-08,-7.28146482745869e-06,-0.00116847956395974]])

# homography_left = np.array([[-0.000710128671370178, 6.65307627276203e-05, -0.0692689783742822],
#         [0.000516381003921171, -0.00181011134155597, -0.997595526929844],
#         [-2.51074118905076e-08, -6.83854860981181e-06, -0.000959883483255739]])

# homography_right = np.array([[-0.000926831714971124,-7.57332958427531e-05,0.994215703860414],
#         [-0.000923137149283102,0.00327126641381199,0.107337667969103],
#         [-2.77833313194565e-07,1.03110471009649e-05,0.00115801865068319]])


# Original
homography_front = np.array([[4.62227601649053e-05, 0.000243520884225642, -0.678748083960862],
    [0.000969465596108860, -0.00207033488113324, -0.734366621126640],
    [1.58512860546350e-07, -6.83048800828728e-06, -0.00119023476366804]])

homography_left = np.array([[-0.000759672412515488, 2.34075591542924e-05, -0.0699936817773495],
    [0.000483107853918350, -0.00189886717269873, -0.997544805245074],
    [-1.49265515027449e-07, -7.08702713960990e-06, -0.000910631508297557]])

homography_right = np.array([[-0.000908962187561903, -3.67579540055241e-05, 0.994837127281325],
    [-0.000886484342219692, 0.00317263543314027, 0.101420799019439],
    [-1.14460320494404e-07, 9.99234254412552e-06, 0.00111021419224332]])

#LARGER RANGE OF VIEW
translation = np.array([[1, 0, 0],[0,1,100],[0,0,1]])

def warp_image(image, homography):
    im_out = cv2.warpPerspective(image, np.matmul(translation,homography), (600, 800))
    # cv2.imshow('warped', im_out)
    # cv2.waitKey(0)
    #cv2.imshow('image', im_out)
    return im_out

def left_hom(image):
    im_out = cv2.warp

# Create mask of front image. im_mask indicates black pixel area
def find_mask(image):
    black_range1 = np.array([0,0,0])
    im_mask = (cv2.inRange(image, black_range1, black_range1)).astype('bool')
    im_mask_inv = (1-im_mask).astype('bool')
    im_mask_inv = np.dstack((im_mask_inv, im_mask_inv, im_mask_inv))
    im_mask= np.dstack((im_mask, im_mask, im_mask))
    return im_mask_inv, im_mask

if __name__ == "__main__":
    count = 0
    while True:
        img_front = cv2.imread('../collected_images/5/center/'+ str(count)+'.jpg')
        img_left = cv2.imread('../collected_images/5/left/'+ str(count)+'.jpg')
        img_right = cv2.imread('../collected_images/5/right/'+ str(count)+'.jpg')


        im_front = warp_image(img_front, homography_front).astype('uint8')
        im_left = warp_image(img_left, homography_left).astype('uint8')
        im_right = warp_image(img_right, homography_right).astype('uint8')

        init_time = time.time()
        im_side = im_left + im_right
        im_mask_inv, im_mask = find_mask(im_side)
        front_masked = np.multiply(im_front, im_mask).astype('uint8')
        side_masked = np.multiply(im_side, im_mask_inv).astype('uint8')
        print("Masking Time: ", time.time()-init_time)
        summed_image = front_masked + side_masked
        #Gaussian Blurring?
        #summed_image = cv2.GaussianBlur(summed_image, (5,5), 0)
        # cv2.imshow('front', front_masked)
        # cv2.imshow('left', im_left)
        # cv2.imshow('right', im_right)
        # cv2.imshow('front', im_front)
        cv2.imshow('summed', summed_image)

        cv2.imwrite('../collected_images/5/mosaic_full/'+str(count) + '.jpg', summed_image)
        #summed_image_cropped = summed_image[200:800, :500, :]
        print("Time elapsed: ", (time.time() - init_time))
        #cv2.imshow('summed cropped', summed_image_cropped)

        count +=1
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break



