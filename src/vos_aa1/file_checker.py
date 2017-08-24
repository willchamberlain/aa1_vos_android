#!/usr/bin/env python

import rospy

import os

from time import sleep
from time import gmtime
from time import strftime

import pandas as pd

class ConfigLoader:
    """Allow persistence in scope"""
    def __init__(self,file_path_and_name_):
        self.file_path_and_name = file_path_and_name_    
        self.file_modification_ts_secs = self.check_file_modification_ts()
    
    def check_file_modification_ts(self):
        return os.path.getctime(self.file_path_and_name)
         

    def check_and_echo_file_modification_ts(self):
        print 'checking %s'%self.file_path_and_name
        file_modificaiton_ts_secs   = self.check_file_modification_ts()
        file_modificaiton_ts_struct = gmtime(file_modificaiton_ts_secs)
        print 'file modification time = %s'%strftime('%Y_%m_%d %H:%M:%S',file_modificaiton_ts_struct)

    def echo_file_modification_ts(self):
        print 'modification time for %s'%self.file_path_and_name
        print 'file modification time = %s'%strftime('%Y_%m_%d %H:%M:%S',gmtime(self.file_modification_ts_secs))
            
    def reload_if_file_changed(self):
        current_file_modificaiton_ts_secs     = self.check_file_modification_ts()        
        if current_file_modificaiton_ts_secs != self.file_modification_ts_secs:
            self.file_modification_ts_secs    = current_file_modificaiton_ts_secs
            print 'file modification time has changed.'
            self.echo_file_modification_ts()
        
    
    
    
if __name__ == "__main__":
    tag_pose_loader = ConfigLoader('/mnt/nixbig/build_workspaces/aa1_vos_android_catkin_ws/src/vos_aa1/src/vos_aa1/fixed_tags_poses.txt')
    tag_pose_data = pd.read_csv(tag_pose_loader.file_path_and_name,sep='|',dtype={'tag_id': str, 'x': float, 'z': float, 'z': float}) # You can also add parameters such as header, sep, etc.
    tag_pose_array = tag_pose_data.values
    print tag_pose_array[0][0]
    print tag_pose_array[0][1]
    print tag_pose_array[0][2]
    print tag_pose_array[0][3]
    print tag_pose_array[0][1]+tag_pose_array[0][2]+tag_pose_array[0][3]
    print tag_pose_array[0][4]+tag_pose_array[0][5]+tag_pose_array[0][6]+tag_pose_array[0][7]
    print tag_pose_array
    tag_pose_data_list = tag_pose_array.tolist()
    print tag_pose_data_list    
    
    
    tag_pose_tuple = ( tag_pose_array[0][0] , { 'x':tag_pose_array[0][1] , 'y':tag_pose_array[0][2] , 'z':tag_pose_array[0][3]  } )
    print tag_pose_tuple
    
    camera_pose_loader = ConfigLoader('/mnt/nixbig/build_workspaces/aa1_vos_android_catkin_ws/src/vos_aa1/src/vos_aa1/camera_poses.txt')
    camera_pose_data = pd.read_csv(camera_pose_loader.file_path_and_name,sep='|',dtype={'camera_id': str, 'x': float, 'z': float, 'z': float}) # You can also add parameters such as header, sep, etc.
    camera_pose_array = camera_pose_data.values
    camera_pose_data_list = camera_pose_array.tolist()
    print camera_pose_data_list    
    
    for x in range(0, 300):
        print 'iteration %d'%x
        # an_instance.check_and_echo_file_modification_ts()
        tag_pose_loader.reload_if_file_changed()
        camera_pose_loader.reload_if_file_changed()
        sleep(1)
        
