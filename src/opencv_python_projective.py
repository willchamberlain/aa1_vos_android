from webcam import Webcam
import cv2
from datetime import datetime

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)

webcam = Webcam(1)
webcam.start()


def main():
    while True:

        #get the image
        image = webcam.get_current_frame()
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        cv2.imshow('grid', image)
        
        ret, corners = cv2.findChessboardCorners(
            image_gray,
            (9,6),
            flags=cv2.cv.CV_CALIB_CB_ADAPTIVE_THRESH)
        
        if ret == True:
            print "found a chessboard"
            cv2.drawChessboardCorners(image, (9,6), corners, ret )
            cv2.imshow('grid', image)
            
            
            corners2 = cv2.cornerSubPix(image_gray,corners,(11,11),(-1,-1),criteria)
            
            filename = datetime.now().strftime('%Y%m%d_%Hh%Mm%Ss%f') + '.jpg'
            filename = "/mnt/nixbig/tmp/pose/sample_images/" + filename
            print "saving image to file %s"%(filename)
            cv2.imwrite( filename , image)
            
            cv2.drawChessboardCorners(image, (9,6), corners2, ret )
            cv2.imshow('grid_2', image)
            key = cv2.waitKey(1000)
            if key == 27:
                break
            cv2.waitKey(100)
        else:
            print "no chessboard found"
            key = cv2.waitKey(1000)
            if key == 27:
                break
            cv2.waitKey(100) 
    
    

if __name__ == '__main__':
    main()
    


cv2.destroyAllWindows()
cv2.waitKey(1)    


