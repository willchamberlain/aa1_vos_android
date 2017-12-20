import cv2
from threading import Thread

class Webcam:
  
  def __init__(self, cam_num_):
    self.video_capture = cv2.VideoCapture(cam_num_)
    self.current_frame = self.video_capture.read()[1]  # capture one frame to make sure that the camera is active and accessible
    
  # create a thread for capturing images  
  def start(self):
    Thread( target=self._update_frame , args=() ).start()
    
  def _update_frame(self):
    while(True):
        self.current_frame = self.video_capture.read()[1]      
        
  def get_current_frame(self):
    return self.current_frame        
    
    
