#!/usr/bin/env python

import sys
import rospy
import threading
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Twist, TwistWithCovariance  # https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
from nav_msgs.msg import Odometry

from vos_aa1.srv import InitialPoseToRobot, InitialPoseToRobotRequest,  InitialPoseToRobotResponse
from vos_aa1.srv import PoseToRobot,        PoseToRobotRequest,         PoseToRobotResponse


service_initialpose = 1
service_base_pose = 1

    
    
def keep_alive_and_sleep():    
    rate = rospy.Rate(5)  # 5hz
    while not rospy.is_shutdown():        
        hello_str = "hello world : keep_alive_and_sleep() : time=%s" % rospy.get_time()
        rospy.loginfo(hello_str)    
        
        call_services()    
        
        rate.sleep()
        
def call_services():
    time_now      = rospy.Time.now()
    poseWCS = PoseWithCovarianceStamped()
    poseWCS.header.stamp = time_now
    poseWCS.header.frame_id = '/map'
    poseWCS.pose.pose = Pose(Point(-1, 4, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    poseWCS.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    

    odom = Odometry()
    odom.header.stamp = time_now
    odom.header.frame_id = '/map'
    poseWC = PoseWithCovariance()
    poseWC.pose = Pose(Point(-1, 4, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    poseWC.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    odom.pose = poseWC
    twistWC = TwistWithCovariance()
    twistWC.twist = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0))
    twistWC.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    odom.twist = twistWC

    rospy.wait_for_service('/Pioneer3_vc_initialpose')      
    try:
        initialpose_proxy = rospy.ServiceProxy('/Pioneer3_vc_initialpose', InitialPoseToRobot)
        resp = initialpose_proxy(poseWCS) 
    except rospy.ServiceException, e:
        print '/Pioneer3_vc_initialpose service call failed: %s'%e    
        
    rospy.wait_for_service('/Pioneer3_vc_base_pose')        
    try:
        base_pose_proxy = rospy.ServiceProxy('/Pioneer3_vc_base_pose', PoseToRobot)
        resp = base_pose_proxy(odom) 
    except rospy.ServiceException, e:
        print '/Pioneer3_vc_base_pose service call failed: %s'%e    
    
            

if __name__ == '__main__':
    threads = []
    try:
        rospy.init_node('dummy_robot_pose_loop_node', anonymous=True)
        keep_alive_and_sleep()
                    
 #                   
 #       thread_node_and_topics = threading.Thread( target =  setup_node_and_topics , args = (system_id_prefix , ))
 #         
 #       print "setup thread_initialpose" 
 #       thread_initialpose = threading.Thread( target =  setup_node_for_server_vc_initialpose , args = (system_id_prefix , ))
 #       
 #       print "setup thread_base_pose"
 #       thread_base_pose   = threading.Thread( target =  setup_node_for_server_vc_base_pose   , args = (system_id_prefix , ))
 #       
 #       print "setup thread_keepalive"
 #       thread_keepalive = threading.Thread( target = keep_alive_and_sleep() )
 #       
 #       print "start thread_keepalive"
 #       thread_keepalive.start()
 #       print "join thread_keepalive"
 #       thread_keepalive.join()        
 #       
 #       print "start thread_base_pose"
 #       thread_base_pose.start()
 #       print "join thread_base_pose"
 #       thread_base_pose.join()
 #       
 #       
 #       print "start thread_initialpose"
 #       thread_initialpose.start()
 #       print "join thread_initialpose"
 #       thread_initialpose.join()
        
    except rospy.ROSInterruptException:
        pass
