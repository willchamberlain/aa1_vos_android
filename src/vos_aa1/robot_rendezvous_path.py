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
global other_robot_goal_publisher
plan_publisher = 1
other_robot_goal_publisher = 1

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
	distance_threshold = 2.0
	#print 'repost_path Path_data='
	#print Path_data
	print 'len(Path_data.poses) = %d'%len(Path_data.poses)
	if len(Path_data.poses) <= 0 :
		print 'No path - returning'
		return
	inverted_Path_data = Path()
	print 'Path_data.poses[len(Path_data.poses)-1].pose.orientation='
	print Path_data.poses[len(Path_data.poses)-1].pose.orientation
	deque_reversing = deque(Path_data.poses)
	deque_reversing.reverse()
	Path_data_reverse_order = list(deque_reversing)
	end_of_path_PoseStamped = Path_data_reverse_order[0]  #  TODO/note - this should be the other robot's pose 
	print 'end_of_path_PoseStamped ='
	print end_of_path_PoseStamped
	other_robot_goal_PoseStamped = end_of_path_PoseStamped
	previous_poseStamped = end_of_path_PoseStamped # dummy value 
	for poseStamped in deque_reversing :  #  reverse order 
		distance = (
		(end_of_path_PoseStamped.pose.position.x-poseStamped.pose.position.x)**2 +
		(end_of_path_PoseStamped.pose.position.y-poseStamped.pose.position.y)**2 ) ** 0.5
		print 'distance = %f, qz = %f'%(distance,poseStamped.pose.orientation.z)
		if distance > distance_threshold :
			other_robot_goal_PoseStamped = poseStamped
			print 'far enough: other_robot_goal_PoseStamped = '
			print other_robot_goal_PoseStamped
			print poseStamped
			y = other_robot_goal_PoseStamped.pose.position.y - previous_poseStamped.pose.position.y
			x = other_robot_goal_PoseStamped.pose.position.x - previous_poseStamped.pose.position.x
			theta = math.atan2(y, x)
			other_robot_goal_PoseStamped.pose.orientation.z = theta / math.pi
			other_robot_goal_PoseStamped.pose.orientation.w = (1 - ((other_robot_goal_PoseStamped.pose.orientation.z)/math.pi)**2)**0.5
			break 
		else: 
			previous_poseStamped = poseStamped				
	print 'other_robot_goal_PoseStamped = '
	print other_robot_goal_PoseStamped
	quaternion = [ other_robot_goal_PoseStamped.pose.orientation.x , other_robot_goal_PoseStamped.pose.orientation.y , other_robot_goal_PoseStamped.pose.orientation.z , other_robot_goal_PoseStamped.pose.orientation.w ]	
	rotation_matrix = t.quaternion_matrix( quaternion )	
	inversed_transform = t.inverse_matrix(rotation_matrix)		## https://answers.ros.org/question/229329/what-is-the-right-way-to-inverse-a-transform-in-python/
	inversed_quaternion = t.quaternion_from_matrix(inversed_transform)
	print quaternion
	print rotation_matrix
	print inversed_transform
	print inversed_quaternion
	other_robot_goal_PoseStamped.pose.orientation.x  =  inversed_quaternion[0]
	other_robot_goal_PoseStamped.pose.orientation.y  =  inversed_quaternion[1]
	other_robot_goal_PoseStamped.pose.orientation.z  =  inversed_quaternion[2]
	other_robot_goal_PoseStamped.pose.orientation.w  =  inversed_quaternion[3]
	print 'other_robot_goal_PoseStamped = '
	print other_robot_goal_PoseStamped
	other_robot_goal_publisher.publish(other_robot_goal_PoseStamped)
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

	print 'end_of_path_PoseStamped x,y = %f,%f'%(end_of_path_PoseStamped.pose.position.x,end_of_path_PoseStamped.pose.position.y)	
	print end_of_path_PoseStamped.pose.orientation


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
	goal_topic_out_name	= rospy.get_param("~plan_topic_out_name","/robot_1/move_base_simple/goal")


	plan_subscriber 		= rospy.Subscriber( plan_topic_in_name , Path, repost_path_inverted)
	#plan_subscriber 		= rospy.Subscriber( plan_topic_in_name , Path, repost_path_delayed)
	global plan_publisher
	plan_publisher 			= rospy.Publisher( plan_topic_out_name , Path, queue_size=1)
	global other_robot_goal_publisher
	other_robot_goal_publisher 			= rospy.Publisher( goal_topic_out_name , PoseStamped, queue_size=1)


	while  not  rospy.is_shutdown() :  # while we are running ...
		rospy.sleep(0.1)

	# Streaming operation is enabled/disabled individually for each command and
	# object(s) the command applies to. In above case, only the joint position of
	# the joint with handle jointHandle will be streamed.

	# time.sleep(5) # pause needed for server side communication


if __name__ == '__main__':
	main()    

