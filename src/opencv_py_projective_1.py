import numpy as np
import cv2

def grid(im_width, im_height, x_space, y_space, rgb_list, line_width):
    cv2.line(img, (x_space,0), (x_space,im_height), rgb_list, line_width )
    cv2.line(img, (0, y_space), (im_width,y_space), rgb_list, line_width)

def rgb_to_bgr_opencv_list(rgb_list):
    return rgb_list[2],rgb_list[1],rgb_list[0]

def rgb_to_bgr_opencv(r,g,b):
    return (b,g,r)


def bgr_opencv_to_rgb(b, g, r):
    return (r,g,b)

# draw geometrics and text

img = np.zeros( (512,512,3) , np.uint8)

cv2.line(img, (0,0), (511,511), (255, 0, 255), 5)


# BGR 
cv2.rectangle(img, (10,10), (50,50), rgb_to_bgr_opencv(255, 0, 0), 3)

cv2.rectangle(img, (101,101), (501,501), rgb_to_bgr_opencv_list(bgr_opencv_to_rgb(255, 0, 0)), 3)

font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1.2    # ratio
font_line_thickness = 2  # int
cv2.putText(img, 'some', (25,400), font, font_scale, (200,200,200), font_line_thickness) #, cv2.LINE_AA)

font_line_thickness = 1  # int
cv2.putText(img, 'text', (125,400), font, font_scale, (200,200,200), font_line_thickness) #, cv2.LINE_AA)


grid(512, 512, 10, 10, (0,200,0), 1)

# display image
#  https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_gui/py_image_display/py_image_display.html#py-display-image

#       Load an color image in grayscale
#       img = cv2.imread('messi5.jpg',0)

cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

