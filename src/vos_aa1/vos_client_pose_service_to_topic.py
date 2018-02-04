#!/usr/bin/env python

import sys
import rospy
import threading
import tf
import math
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Twist, TwistWithCovariance  # https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
from nav_msgs.msg import Odometry

from vos_aa1.srv import InitialPoseToRobot, InitialPoseToRobotRequest,  InitialPoseToRobotResponse
from vos_aa1.srv import PoseToRobot,        PoseToRobotRequest,         PoseToRobotResponse
from vos_aa1.srv import PoseStampedToRobot, PoseStampedToRobotRequest,  PoseStampedToRobotResponse



pub_topic_initialpose = 1
pub_topic_base_pose = 1
pub_topic_target_pose = 1

service_initialpose = 1
service_base_pose = 1
service_target_pose = 1

def setup_node_and_topics(system_id_prefix):     
    global pub_topic_initialpose
    global pub_topic_base_pose
    global pub_topic_target_pose
    pub_topic_initialpose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    pub_topic_base_pose   = rospy.Publisher('base_pose_ground_truth', Odometry, queue_size=10)
    pub_topic_target_pose   = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    
    
def keep_alive_and_sleep():    
    rate = rospy.Rate(0.5)  # 20hz
    while not rospy.is_shutdown():        
        hello_str = "hello world : keep_alive_and_sleep() : time=%s" % rospy.get_time()
        rospy.loginfo(hello_str)        
        
        rate.sleep()
        
       
def handle_RobotInitialPose(req):
    print "handle_RobotInitialPose(req)"
    pub_topic_initialpose.publish(req.poseWithCovarianceStamped)
    return InitialPoseToRobotResponse("handle_RobotInitialPose(req) : dummy respose : time=s")

def handle_RobotPose(req):
    print "handle_RobotPose(req)"
    pub_topic_base_pose.publish(req.odometry)
    return PoseToRobotResponse("handle_RobotPose(req) : dummy respose : time=s")

def handle_RobotTargetPose(req):
    quat_ = req.poseStamped.pose.orientation
    q_norm = math.sqrt(quat_.x**2 + quat_.y**2 + quat_.z**2 + quat_.w**2)
    normalised_quat = Quaternion( quat_.x/q_norm, quat_.y/q_norm, quat_.z/q_norm, quat_.w/q_norm )
    
    q_norm = math.sqrt( 0.0 + 0.0 + quat_.z**2 + quat_.w**2)
    normalised_quat = Quaternion( 0.0, 0.0, quat_.z/q_norm, quat_.w/q_norm )
    
    poseStamped_ = PoseStamped()
    poseStamped_.header.stamp    = req.poseStamped.header.stamp
    poseStamped_.header.frame_id = req.poseStamped.header.frame_id
    poseStamped_.pose = Pose(  req.poseStamped.pose.position,  normalised_quat  )
                            
                    
    print "handle_RobotTargetPose(req)"
    pub_topic_target_pose.publish(poseStamped_)
    return PoseStampedToRobotResponse("handle_RobotTargetPose(req) : dummy respose : time=s" )



def setup_node_for_server_vc_initialpose(system_id_prefix):
    global service_initialpose
    service_initialpose = rospy.Service( system_id_prefix+'vc_initialpose' , InitialPoseToRobot, handle_RobotInitialPose)
    print "Ready to "+system_id_prefix+'vc_initialpose'

def setup_node_for_server_vc_base_pose(system_id_prefix):
    global service_base_pose
    service_base_pose = rospy.Service( system_id_prefix+'vc_base_pose' , PoseToRobot, handle_RobotPose)
    print "Ready to "+system_id_prefix+'vc_base_pose'

def setup_node_for_server_vc_target_pose(system_id_prefix):
    global service_target_pose
    service_target_pose = rospy.Service( system_id_prefix+'vc_target_pose' , PoseStampedToRobot, handle_RobotTargetPose)
    print "Ready to "+system_id_prefix+'vc_target_pose'
    
            

if __name__ == '__main__':
    threads = []
    try:
        rospy.init_node('vc_node', anonymous=True)        
        #        rospy.init_node(system_id_prefix+'vc_node', anonymous=True)    
        system_id = rospy.get_param('~system_id', '')
        system_id_prefix="bob"
        if not system_id:
            print 'WARNING ------------ '
            print '-- system_id not specified as param ' +rospy.get_name()+'/system_id'+ ': defaulting to "bob" '
            system_id_prefix="bob"
        else:    
            system_id_prefix=system_id+'_'
            
        print 'system_id_prefix= '+system_id_prefix
            
        
        
        setup_node_and_topics(system_id_prefix)
        setup_node_for_server_vc_initialpose(system_id_prefix)
        setup_node_for_server_vc_base_pose(system_id_prefix)
        setup_node_for_server_vc_target_pose(system_id_prefix)
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
