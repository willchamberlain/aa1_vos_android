#!/usr/bin/env python

import rospy
from   geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from   nav_msgs.msg      import Odometry, Path
from 	 tf 							 import transformations as t
import tf
import time
import sys, signal
import copy
import math
import numpy
from collections import deque


global plan_publisher 
plan_publisher = 1

def signal_handler(signal, frame):
	""" Exit if a signal is received - e.g. a keybpard interrupt. """
	print("\nprogram exiting gracefully")
	sys.exit(0)

def initialise_ROS_node(node_name_, wait_duration_):
  rospy.init_node(node_name_)
  rospy.sleep(wait_duration_)

def repost_path_delayed(Path_data):
	print '!!  repost_path_delayed()  -  does nothing !!'	

def repost_path_inverted(Path_data):
	print 'repost_path start'
	#print 'repost_path Path_data='
	#print Path_data
	print 'len(Path_data.poses) = %d'%len(Path_data.poses)
	inverted_Path_data = Path()
	deque_reversing = deque()
	for poseStamped in Path_data.poses :
		# print 'poseStamped.pose.position.x = %f'%poseStamped.pose.position.x
		## M = quaternion_matrix([0.99810947, 0.06146124, 0, 0]) # https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
		quaternion = [ poseStamped.pose.orientation.x , poseStamped.pose.orientation.y , poseStamped.pose.orientation.z , poseStamped.pose.orientation.w ]	
		quaternion = [ poseStamped.pose.orientation.x , poseStamped.pose.orientation.y , poseStamped.pose.orientation.z , poseStamped.pose.orientation.w ]	
		rotation_matrix = t.quaternion_matrix( quaternion )
		## https://answers.ros.org/question/229329/what-is-the-right-way-to-inverse-a-transform-in-python/
		inversed_transform = t.inverse_matrix(rotation_matrix)
		inversed_quaternion = t.quaternion_from_matrix(inversed_transform)
		## print inversed_quaternion
		inverse_poseStamped = poseStamped
		inverse_poseStamped.pose.orientation.x  =  inversed_quaternion[0]
		inverse_poseStamped.pose.orientation.y  =  inversed_quaternion[1]
		inverse_poseStamped.pose.orientation.z  =  inversed_quaternion[2]
		inverse_poseStamped.pose.orientation.w  =  inversed_quaternion[3]
		#inverse_poseStamped.pose.orientation.z  =  -1.0 * inverse_poseStamped.pose.orientation.z
		#inversed_quaternion = [ inverse_poseStamped.pose.orientation.x , inverse_poseStamped.pose.orientation.y , inverse_poseStamped.pose.orientation.z , inverse_poseStamped.pose.orientation.w ]	
		#print 'quaternion, then inversed_quaternion : '
		#print quaternion
		#print inversed_quaternion
		#print 'inverse_poseStamped.header.frame_id=%s'%inverse_poseStamped.header.frame_id
		#inverted_Path_data.poses.insert(0,inverse_poseStamped)
		deque_reversing.appendleft(inverse_poseStamped)
	inverted_Path_data.header = Path_data.header
	inverted_Path_data.poses = 	list(deque_reversing)
	plan_publisher.publish(inverted_Path_data)
	print 'len(Path_data.poses) = %d'%len(Path_data.poses)	
	print 'len(inverted_Path_data.poses) = %d'%len(inverted_Path_data.poses)	
	print 'inverted_Path_data.header.frame_id=%s'%inverted_Path_data.header.frame_id
	print 'inverted_Path_data first x,y = %f,%f'%(inverted_Path_data.poses[0].pose.position.x,inverted_Path_data.poses[0].pose.position.y)
	print 'inverted_Path_data last x,y = %f,%f'%(inverted_Path_data.poses[len(inverted_Path_data.poses)-1].pose.position.x,inverted_Path_data.poses[len(inverted_Path_data.poses)-1].pose.position.y)	
	print inverted_Path_data.poses[0].pose.orientation
	print inverted_Path_data.poses[len(Path_data.poses)-1].pose.orientation
	print 'Path_data first x,y = %f,%f'%(Path_data.poses[0].pose.position.x,Path_data.poses[0].pose.position.y)
	print 'Path_data last x,y = %f,%f'%(Path_data.poses[len(Path_data.poses)-1].pose.position.x,Path_data.poses[len(Path_data.poses)-1].pose.position.y)	
	print Path_data.poses[0].pose.orientation
	print Path_data.poses[len(Path_data.poses)-1].pose.orientation



  #  rosrun  vos_aa1  vrep__robot__handle_odometry.py  _base_link_object_name_in_VRep:=base_link  _new_child_frame:=laser  _old_parent_frame:=Pioneer_p3dx  _new_parent_frame:=base_link 
  #
  #  rosrun  vos_aa1  vrep__robot__handle_odometry.py  _base_link_object_name_in_VRep:=base_link 
  #
def main():
	signal.signal(signal.SIGINT, signal_handler)
	#node_name     			= rospy.get_param("~node_name","_node")								#	"base_link"
	initialise_ROS_node( "robot_renezvous_path" , 0.5 )

	plan_topic_in_name	= rospy.get_param("~plan_topic_in_name","/robot_0/move_base_node/NavfnROS/plan")
	plan_topic_out_name	= rospy.get_param("~plan_topic_out_name","/robot_1/move_base_node/NavfnROS/plan")


	plan_subscriber 		= rospy.Subscriber( plan_topic_in_name , Path, repost_path_inverted)
	#plan_subscriber 		= rospy.Subscriber( plan_topic_in_name , Path, repost_path_delayed)
	global plan_publisher
	plan_publisher 			= rospy.Publisher( plan_topic_out_name , Path, queue_size=1)


	while  not  rospy.is_shutdown() :  # while we are connected to the server..


		rospy.sleep(0.1)

	# Streaming operation is enabled/disabled individually for each command and
	# object(s) the command applies to. In above case, only the joint position of
	# the joint with handle jointHandle will be streamed.

	# time.sleep(5) # pause needed for server side communication


if __name__ == '__main__':
#	M = t.quaternion_matrix([1, 0, 0, 0])  						# 	[ w , x , y , z ] according to https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
#	print 'numpy.allclose(M, numpy.identity(4)) = '
#	print numpy.allclose(M, numpy.identity(4))	FALSE
#	M = t.quaternion_matrix([0, 0, 0, 1])  						# 	[ x , y , z , w ] according to https://github.com/ros/geometry/blob/indigo-devel/tf/src/tf/transformations.py
#	print 'numpy.allclose(M, numpy.identity(4)) = '	
#	print numpy.allclose(M, numpy.identity(4))  TRUE
	main()    

