#!/usr/bin/env python

import sys
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, PoseWithCovarianceStamped  # https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf

from tf import transformations
import numpy as np
from visualization_msgs.msg import Marker
import threading
from pprint import pprint
import std_msgs
import time

class robot_self_model:

    def init(self, config_string_or_filepath):
        return [config_string_or_filepath]  # TODO - better, structured, configuration 
        
    def __init__(self, config_string_or_filepath):  # TODO - better, structured, configuration - string, or filepath
        config = self.init(config_string_or_filepath)
        self.robot_name = config[0]  # TODO - better, structured, configuration 
        rospy.init_node(self.robot_name)  # TODO - what if ambiguous in ROSCORE list? WALLEE, WALLEE_1, WALLEE_2, ....
        self.rate = rospy.Rate(10) # 10hz
            
    def run(self):       
        while not rospy.is_shutdown():            
            self.rate.sleep()     

if __name__ == "__main__":
    robot_instance = robot_self_model('this_robot')
    try:
        robot_instance.run()
    except rospy.ROSInterruptException:
        pass    
    
    
