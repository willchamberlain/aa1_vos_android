import vrep
import time

from PIL import Image
import array

import cv2
import numpy as np


# Traversible area <- Camera FoV <- Camera homography <- VRep marker cube <-> image processing Python script 


def mask_coloured_object(image, lower_, upper_):

    # Blur the image to reduce noise
    #blur = cv2.GaussianBlur(image, (5,5),0)
    # blur = image
    blur = cv2.GaussianBlur(image, (3,3),0)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_, upper_)

    kernel = np.ones((3,3), np.uint8)
    img_dilation = cv2.dilate( mask, kernel, iterations=2)
    img_erosion  = cv2.erode( img_dilation, np.ones((3,3), np.uint8), iterations=3)
    return img_erosion

def doNothing(x):
    pass

def grid_points_3D_():
  grid_points_3D = np.float32([9000,9000]) # push dummies
  for x in np.arange(-1.,8.+0.1,0.1):
    for y in np.arange(1.,-9.+-0.1,-0.1):
  # for x in np.arange(5.,7.+0.1,1.):
  #   for y in np.arange(-6.,-8.+-0.1,-1.):
      grid_points_3D = np.append( grid_points_3D , [x,y] ) 
  grid_points_3D = grid_points_3D[2:len(grid_points_3D)] # pop dummies 
  grid_points_3D = grid_points_3D.reshape(-1,2)        
  return grid_points_3D


def main():
  # Threshold the HSV image for carpet colour
  lower_carpet_1 = np.array([165,  10,  10])
  upper_carpet_1 = np.array([195, 255, 255])
  lower_carpet_2 = np.array([  0,   0,  10])
  upper_carpet_2 = np.array([360,  40, 120])
  lower_carpet_3 = np.array([  0,  50,  10])
  upper_carpet_3 = np.array([360, 140, 170])

  cam_603_homographyMatrix = np.array(
  [[  2.23335127e-03 , -5.80560963e-03 ,  4.48131585e+00],
   [  6.54594554e-04 ,  1.02718160e-02 , -8.74098851e+00],
   [ -4.10661566e-05 , -9.71837108e-04 ,  1.00000000e+00]])
  cam_603_inv_homographyMatrix = np.array(
  [[  3.76123874e+02 ,  3.07016442e+02 ,  9.98097315e+02],
   [ -6.25749739e+01 ,  5.11668820e+02 ,  4.75290949e+03],
   [ -4.53667198e-02 ,  5.09866731e-01 ,  5.66004184e+00]] )
  print cam_603_homographyMatrix
  print cam_603_inv_homographyMatrix    

  # Create a window
  cv2.namedWindow('image',cv2.WINDOW_NORMAL)  
  show_taskbars = False
  if show_taskbars:
    cv2.createTrackbar('Hue lower','image',0,360,doNothing)
    cv2.createTrackbar('Hue upper','image',0,360,doNothing)
    cv2.createTrackbar('Sat lower','image',0,255,doNothing)
    cv2.createTrackbar('Sat upper','image',0,255,doNothing)
    cv2.createTrackbar('Val lower','image',0,255,doNothing)
    cv2.createTrackbar('Val upper','image',0,255,doNothing)

  vrep.simxFinish(-1)

  clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

  if clientID!=-1:
    print 'Connected to remote API server'
    # get vision sensor objects
    res, camSensor     = vrep.simxGetObjectHandle(clientID, 'cam603sensor', vrep.simx_opmode_oneshot_wait)
    print 'cam_603_sensor id=camSensor=%d'%camSensor
    res, overlay_sensor = vrep.simxGetObjectHandle(clientID, 'dummy_sensor_for_opencv_overlay', vrep.simx_opmode_oneshot_wait)
    print 'dummy_sensor_for_opencv_overlay id=overlay_sensor=%d'%overlay_sensor

    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, camSensor, 0, vrep.simx_opmode_streaming)
    time.sleep(1)

    while (vrep.simxGetConnectionId(clientID) != -1):
      # get image from vision sensor 'camSensor'
      err, resolution, image = vrep.simxGetVisionSensorImage(clientID, camSensor, 0, vrep.simx_opmode_buffer)
      if err == vrep.simx_return_ok:
        print 'image received to process: resolution = %dx%d'%(resolution[0],resolution[1])


        image_byte_array = array.array('b', image)
        image_buffer = Image.frombuffer("RGB", (resolution[0],resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
        img2 = np.asarray(image_buffer)

        #img2_rgb = Image.fromarray(np.roll(img2, 1, axis=-1))
        img2_rgb = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)

        print 'img2.shape=%dx%d'%(img2.shape[1],img2.shape[2])
        img3 = np.copy(img2)

        if show_taskbars:
          h_l = cv2.getTrackbarPos('Hue lower','image')
          h_u = cv2.getTrackbarPos('Hue upper','image')
          s_l = cv2.getTrackbarPos('Sat lower','image')
          s_u = cv2.getTrackbarPos('Sat upper','image')
          v_l = cv2.getTrackbarPos('Val lower','image')
          v_u = cv2.getTrackbarPos('Val upper','image')

          lower_carpet = np.array([h_l, s_l, v_l])
          upper_carpet = np.array([h_u, s_u, v_u])

          mask_from_GUI = mask_coloured_object(img2, lower_carpet, upper_carpet)
          # cv2.imshow('image', mask_from_GUI)
          print 'type(mask_from_GUI) ='
          print type(mask_from_GUI)
          print len(mask_from_GUI)
          print mask_from_GUI.shape
          print mask_from_GUI[0:10,0:10]

        mask_1 = mask_coloured_object(img2, lower_carpet_1, upper_carpet_1)
        mask_2 = mask_coloured_object(img2, lower_carpet_2, upper_carpet_2)
        mask_3 = mask_coloured_object(img2, lower_carpet_3, upper_carpet_3)
        mask_total = mask_1 + mask_2 + mask_3
        np.clip(mask_total,0,255,mask_total)
        print 'type(mask_total) ='
        print type(mask_total)
        print len(mask_total)
        print mask_total.shape
        print mask_total[0:10,0:10]
        #cv2.imshow('image', mask_total)
        #cv2.imshow('image', mask_from_GUI)

        img2 = img2.ravel()
        vrep.simxSetVisionSensorImage(clientID, overlay_sensor, img2, 0, vrep.simx_opmode_oneshot)
      
        grid_points_3D = grid_points_3D_()
        print 'grid_points_3D=' ;  print grid_points_3D
        # get the pixel coordinates of each square, check the pixels in that range 
        grid_points_px = cv2.perspectiveTransform(grid_points_3D.reshape(-1,1,2),cam_603_inv_homographyMatrix)  
        grid_points_px_squeezed = grid_points_px.astype(int).squeeze()
        grid_points_px_list = grid_points_px_squeezed.tolist()  
        A = np.array(  [ [436, 577], [391, 460] ]  )
        debug_ = False
        if debug_:
          print 'grid_points_px=' ;  print grid_points_px ;  print 'grid_points_px.squeeze()=' ;  print grid_points_px.squeeze() ;  print 'grid_points_px.astype(int).squeeze()=' ;  print grid_points_px.astype(int).squeeze() ;  print 'mask_total[grid_points_px.astype(int).squeeze()]  = '
          print 'type(grid_points_px_squeezed) =' ;  print type(grid_points_px_squeezed) ;  print 'grid_points_px_squeezed.shape =' ;  print grid_points_px_squeezed.shape
          print 'grid_points_px_list = ' ;  print grid_points_px_list ;  print 'len(grid_points_px_list) = ' ;  print len(grid_points_px_list) ;  print 'len(grid_points_px_list[0]) = ' ;  print len(grid_points_px_list[0])
          # print mask_total[grid_points_px.astype(int).squeeze()]
          print 'before : mask_total.shape=' ;  print mask_total.shape
          # print 'A.shape' ;  print A.shape
          # print mask_total[ [[436, 577], [391, 460], [312, 255], [606, 561], [607, 437], [610, 217], [783, 545], [835, 413], [929, 175]] ]
          # print mask_total[ [ [436, 577], [391, 460] ] ]
          print mask_total[ A.tolist() ] ;  print 'after'
          # print mask_total[grid_points_px_list]

        # start checking whether we can see carpet   
        # start display of carpet texture through floor grid 
        mask_total_1 = 0 - mask_total   # right size and type, all zeros 
        for grid_points_px_idx in range(0,len(grid_points_px)):
          px = grid_points_px[grid_points_px_idx]
          px = px.astype(int)
          px = px.squeeze()
          if px[0]>=0 and px[0]<mask_total.shape[0] and px[1]>=0 and px[1]<mask_total.shape[1]:
            # mask_total_1[ px[1],px[0] ] = 255     # x,y maps to image v,u
            mask_total_1[ px[1],px[0] ] = mask_total[ px[1],px[0] ]     # x,y maps to image v,u
            # print 'px  number  %d  is in the FoV'%grid_points_px_idx
            # print px
          # else:
            # print 'px  number  %d  is outside the FoV:'%grid_points_px_idx
            # print px        
        display_orientation_checks = False    
        if display_orientation_checks:    
          mask_total_1[10:20,:] = 255     # check orientation : top-to-bottom/down as displayed
          mask_total_1[:,100:110] = 255   # check orientation : left-to-right as displayed
                  
        cv2.imshow('image', np.flipud(mask_total_1) )
        # end display
        # end checking whether we can see carpet 

      elif err == vrep.simx_return_novalue_flag:
        print "no image yet"
        pass
      else:
        print err

      # Wait longer to prevent freeze for videos.
      if cv2.waitKey(33 ) & 0xFF == ord('q'):
          break

  else:
    print "Failed to connect to remote API Server"
    vrep.simxFinish(clientID)


if __name__ == '__main__':
  main()    