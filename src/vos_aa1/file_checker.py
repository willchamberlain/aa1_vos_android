#!/usr/bin/env python

import rospy

import os

from time import sleep
from time import gmtime
from time import strftime

class file_checker_class:
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
    an_instance = file_checker_class('/mnt/nixbig/build_workspaces/aa1_vos_android_catkin_ws/src/vos_aa1/src/vos_aa1/config.txt')
    for x in range(0, 300):
        print 'iteration %d'%x
        # an_instance.check_and_echo_file_modification_ts()
        an_instance.reload_if_file_changed()
        sleep(1)
        
