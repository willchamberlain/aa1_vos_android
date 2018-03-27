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


global goal_publisher 
global other_robot_goal_publisher
goal_publisher = 1
other_robot_goal_publisher = 1
global robot0_base_pose_ground_truth
global robot1_base_pose_ground_truth
robot0_base_pose_ground_truth = None
robot1_base_pose_ground_truth = None
global call_number 
call_number = 0

def signal_handler(signal, frame):
	""" Exit if a signal is received - e.g. a keybpard interrupt. """
	print("\nprogram exiting gracefully")
	sys.exit(0)

def initialise_ROS_node(node_name_, wait_duration_):
  rospy.init_node(node_name_)
  rospy.sleep(wait_duration_)

def repost_path_delayed(Path_data):
	print '!!  repost_path_delayed()  -  does nothing !!'	

def robot0_pose_callback(robot0_base_pose_ground_truth_):
	global robot0_base_pose_ground_truth
	robot0_base_pose_ground_truth = robot0_base_pose_ground_truth_

def robot1_pose_callback(robot1_base_pose_ground_truth_):
	global robot1_base_pose_ground_truth
	robot1_base_pose_ground_truth = robot1_base_pose_ground_truth_

def replan(distance_threshold_):	
	global robot0_base_pose_ground_truth
	global robot1_base_pose_ground_truth
	print 'replan()'
	print 'robot0_base_pose_ground_truth='
	print robot0_base_pose_ground_truth
	print 'robot1_base_pose_ground_truth='
	print robot1_base_pose_ground_truth
	global call_number
	call_number = call_number+1
	print 'call_number=%d'%call_number
	if robot0_base_pose_ground_truth is not None and robot1_base_pose_ground_truth is not None :
		print 'robot0_base_pose_ground_truth is not None and robot1_base_pose_ground_truth is not None'
		# check whether close enough
		distance = ((robot0_base_pose_ground_truth.pose.pose.position.x-robot1_base_pose_ground_truth.pose.pose.position.x)**2
		+(robot0_base_pose_ground_truth.pose.pose.position.y-robot1_base_pose_ground_truth.pose.pose.position.y)**2 )**0.5
		if distance<=distance_threshold_:
			print 'robots are close enough: distance = %f'%distance
			return

		# if not close enough, set a goal
		if (call_number % 10) == 0 :
			print '(call_number mod 4) == 0   ;  call_number=%d'%call_number

			# set robot_0 goal to the robot_1 pose
			print 'robot1_base_pose_ground_truth.pose.pose.position='
			print robot1_base_pose_ground_truth.pose.pose.position
			print 'robot1_base_pose_ground_truth.pose.pose.orientation='
			print robot1_base_pose_ground_truth.pose.pose.orientation

			goal_pose = PoseStamped()
			goal_pose.header.frame_id="map"
			goal_pose.header.stamp=rospy.Time.now()

			goal_pose.pose.position=robot1_base_pose_ground_truth.pose.pose.position
			goal_pose.pose.orientation=robot1_base_pose_ground_truth.pose.pose.orientation
			goal_publisher.publish(goal_pose)


  #  every 0.25s 
  #   get robot2 location from VOS 
  #   if robot1 and robot2 are within 2m, exit
  #		if 4th iteration (e.g. every 1s)
  #   	plan path to robot2
  #   	robot_rendezvous_path.py hears this path and sends robot2 a replanning signal 
  #
def main():
	signal.signal(signal.SIGINT, signal_handler)
	#node_name     			= rospy.get_param("~node_name","_node")								#	"base_link"
	initialise_ROS_node( "robot_renezvous_robot_1" , 0.5 )

	robot0_topic_in_name				= rospy.get_param("~robot0_topic_in_name","/robot_0/base_pose_ground_truth")
	robot1_topic_in_name				= rospy.get_param("~robot1_topic_in_name","/robot_1/base_pose_ground_truth")
	robot0_goal_topic_out_name	= rospy.get_param("~robot0_goal_topic_out_name","/robot_0/move_base_simple/goal")
	distance_threshold					= rospy.get_param("~distance_threshold",2)


	robot0_pose_subscriber 			= rospy.Subscriber( robot0_topic_in_name , Odometry, robot0_pose_callback)
	robot1_pose_subscriber 			= rospy.Subscriber( robot1_topic_in_name , Odometry, robot1_pose_callback)
	#plan_subscriber 		= rospy.Subscriber( plan_topic_in_name , Path, repost_path_delayed)
	global goal_publisher
	goal_publisher 							= rospy.Publisher( robot0_goal_topic_out_name , PoseStamped, queue_size=1)


	while  not  rospy.is_shutdown() :  # while we are running ...
		rospy.sleep(0.25)
		replan(distance_threshold)

	# Streaming operation is enabled/disabled individually for each command and
	# object(s) the command applies to. In above case, only the joint position of
	# the joint with handle jointHandle will be streamed.

	# time.sleep(5) # pause needed for server side communication


if __name__ == '__main__':
	main()    

