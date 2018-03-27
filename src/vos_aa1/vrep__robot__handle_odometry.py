#!/usr/bin/env python

import rospy
from   nav_msgs.msg  import Odometry
import tf
import vrep
import time
import sys, signal
import copy
import math

def signal_handler(signal, frame):
	""" Exit if a signal is received - e.g. a keybpard interrupt. """
	print("\nprogram exiting gracefully")
	sys.exit(0)

def initialise_ROS_node(node_name_, wait_duration_):
  rospy.init_node(node_name_)
  rospy.sleep(wait_duration_)


def VRep_close_and_connect():	
  vrep.simxFinish(-1)
  clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
  return clientID

  #  rosrun  vos_aa1  vrep__robot__handle_odometry.py  _base_link_object_name_in_VRep:=base_link  _new_child_frame:=laser  _old_parent_frame:=Pioneer_p3dx  _new_parent_frame:=base_link 
  #
  #  rosrun  vos_aa1  vrep__robot__handle_odometry.py  _base_link_object_name_in_VRep:=base_link 
  #
def main():
	signal.signal(signal.SIGINT, signal_handler)
	#node_name     			= rospy.get_param("~node_name","_node")								#	"base_link"
	initialise_ROS_node( "vrep__robot__handle_odometry_node" , 0.5 )


	topic_out_name='/vrep/base_pose_ground_truth'	
	odometry_publisher = rospy.Publisher( topic_out_name , Odometry, queue_size=10)

	base_link_object_name_in_VRep     = rospy.get_param("~base_link_object_name_in_VRep")								#	"base_link"
	# new_child_frame     = rospy.get_param("~new_child_frame") 				# "base_link"
	# old_parent_frame    = rospy.get_param("~old_parent_frame")						#	"odom"
	# new_parent_name     = rospy.get_param("~new_parent_frame")				#	"odom"

	clientID = -1
	while -1 == clientID:
		print '%s : connecting'%__name__
		clientID = VRep_close_and_connect()
		print '%s : failed to connect; now waiting'%__name__
		time.sleep(0.7)


	# res, kinectHandle = vrep.simxGetObjectHandle(clientID, 'kinect', vrep.simx_opmode_oneshot_wait)
	# res, cupHandle = vrep.simxGetObjectHandle(clientID, 'Cup_visible_transparent', vrep.simx_opmode_oneshot_wait)
	# res, cupOrientation = vrep.simxGetObjectOrientation(clientID, cupHandle, kinectHandle, vrep.simx_opmode_streaming)


	res, base_link_Handle 			= vrep.simxGetObjectHandle(clientID, base_link_object_name_in_VRep, vrep.simx_opmode_oneshot_wait)
	print 'got simxGetObjectHandle simx_opmode_oneshot_wait'	
	res, base_link_Position 		= vrep.simxGetObjectPosition(clientID, base_link_Handle, -1, vrep.simx_opmode_streaming)		  # setup stream # absolute
	print 'got simxGetObjectPosition simx_opmode_streaming'	
	res, base_link_Orientation_euler 	= vrep.simxGetObjectOrientation(clientID, base_link_Handle, -1, vrep.simx_opmode_streaming)		# setup stream # absolute
	print 'got simxGetObjectOrientation simx_opmode_streaming'		
	#res, base_link_Orientation_quat 	= vrep.simxGetObjectQuaternion(clientID, base_link_Handle, -1, vrep.simx_opmode_streaming)		# get from stream # absolute	

	previousLastCmdTime= -1
	lastCmdTime= -1
	previousYaw = -90000
		# The control loop:		-		http://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm#streaming
	while vrep.simxGetConnectionId(clientID) != -1 :  # while we are connected to the server..

		res_2, base_link_Position 		= vrep.simxGetObjectPosition(clientID, base_link_Handle, -1, vrep.simx_opmode_buffer)		    # get from stream # absolute
		print 'got simxGetObjectPosition simx_opmode_buffer'	
		print base_link_Position
		
		#lastCmdTime=vrep.simxGetLastCmdTime(clientID)
		#print 'lastCmdTime=%f'%lastCmdTime
		

		# roll, pitch, yaw in rads 
		res_1, base_link_Orientation_euler 	= vrep.simxGetObjectOrientation(clientID, base_link_Handle, -1, vrep.simx_opmode_buffer)		# get from stream # absolute
		print 'got simxGetObjectOrientation simx_opmode_buffer'	
		print base_link_Orientation_euler

		lastCmdTime=vrep.simxGetLastCmdTime(clientID)
		print 'lastCmdTime=%f'%lastCmdTime
		print 'previousLastCmdTime=%f'%previousLastCmdTime
		if previousLastCmdTime == -1 :
			previousLastCmdTime  =  lastCmdTime

		#res_3, base_link_Orientation_quat 	= vrep.simxGetObjectQuaternion(clientID, base_link_Handle, -1, vrep.simx_opmode_buffer)		# get from stream # absolute
		#print 'got simxGetObjectQuaternion simx_opmode_buffer'	
		#print base_link_Orientation_quat


		# Fetch the newest joint value from the inbox (func. returns immediately (non-blocking)):
		if res_1==vrep.simx_return_ok and res_2==vrep.simx_return_ok and lastCmdTime > 0 and previousLastCmdTime > 0 :  # res_3==vrep.simx_return_ok and 
			if previousYaw <= -90000 :
				print 'setting first previousYaw'
				yaw_now = base_link_Orientation_euler[2]  
				previousYaw = yaw_now
			else: 
				if lastCmdTime - previousLastCmdTime > 0:
					print '----- lastCmdTime - previousLastCmdTime > 0 -----'
					previous_base_link_Position    = copy.deepcopy(base_link_Position)
					previous_base_link_Orientation = copy.deepcopy(base_link_Orientation_euler)
					print ' here we have the newest position!'
					print ' x: %f'%base_link_Position[0]
					print ' y: %f'%base_link_Position[1]
					print ' z: %f'%base_link_Position[2]
					print ' here we have the newest quaternion!'
					print ' alpha: %f'%base_link_Orientation_euler[0]
					print '  beta: %f'%base_link_Orientation_euler[1]
					print ' gamma: %f'%base_link_Orientation_euler[2]
					# here we have the newest joint position in variable jointPosition!    
					yaw_now = base_link_Orientation_euler[2]  
					print '(previousYaw - yaw_now) = (%f - %f) = %f'%(previousYaw, yaw_now, (previousYaw - yaw_now))
					print '(lastCmdTime - previousLastCmdTime) = (%f - %f) = %f'%(lastCmdTime, previousLastCmdTime, (lastCmdTime - previousLastCmdTime))
					angular_vel = ((previousYaw - yaw_now) * 1000.00) /  (lastCmdTime - previousLastCmdTime) 
					#angular_vel = angular_vel * 1000.00
					print ' angular_vel: %f'%angular_vel
					rad2deg=( 360/ math.pi*2 )
					print ' angular_vel degrees: %f'%(angular_vel*rad2deg)

					topic_out_parent_frame_id='/world'
					topic_out_timestamp = rospy.get_rostime()
					topic_out_child_frame_id='/base_link'

					odometry_msg = Odometry()
					odometry_msg.header.stamp    = topic_out_timestamp
					odometry_msg.header.frame_id = topic_out_parent_frame_id
					odometry_msg.child_frame_id  = topic_out_child_frame_id
					odometry_msg.pose.pose.position.x     = base_link_Position[0]
					odometry_msg.pose.pose.position.y     = base_link_Position[1]
					odometry_msg.pose.pose.position.z     = 0.0
					odometry_msg.pose.pose.orientation.x  = 0.0
					odometry_msg.pose.pose.orientation.y  = 0.0
					odometry_msg.pose.pose.orientation.z  = base_link_Orientation_euler[2] / (2*math.pi)
					# odometry_msg.pose.pose.orientation.w  = (  1 - (base_link_Orientation_euler[2]**2)  )**0.5  # disregarding angles off the plane 
					odometry_msg.pose.pose.orientation.w  = (  1 - (  base_link_Orientation_euler[2]/(2*math.pi)) **2  ) **0.5
					odometry_msg.pose.covariance 					= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
					odometry_msg.twist.twist.linear.x 		= (1000.00 * (previous_base_link_Position[0]-base_link_Position[0])) /  (lastCmdTime - previousLastCmdTime) #  TODO - should be relative to previous position? this is relative to current position
					odometry_msg.twist.twist.linear.y 		= (1000.00 * (previous_base_link_Position[1]-base_link_Position[1])) /  (lastCmdTime - previousLastCmdTime) #  TODO - should be relative to previous position? this is relative to current position
					odometry_msg.twist.twist.linear.z     = 0.0
					odometry_msg.twist.twist.angular.x		=	0.0
					odometry_msg.twist.twist.angular.y 		= 0.0
					odometry_msg.twist.twist.angular.z 		= angular_vel
					odometry_msg.twist.covariance 				= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

					# do variable updates _after_	calculations
					previousYaw = yaw_now
					previousLastCmdTime  =  lastCmdTime

					odometry_publisher.publish(odometry_msg)

					# pause for simulation to update
					rospy.sleep(0.4)
				else:
					print 'lastCmdTime-previousLastCmdTime <= 0 : %d'%(lastCmdTime-previousLastCmdTime)
					rospy.sleep(0.4)
		else:
			print 'else - still waiting'
		    # once you have enabled data streaming, it will take a few ms until the first value has arrived. So if
		    # we landed in this code section, this does not always mean we have an error!!!


		    # LATER - see  controlTypeExamples.ttt	  
    		#simDisplayDialog('Error',"Remote Api plugin was not found. ('"..pluginFile.."')&&nSimulation will not run properly",sim_dlgstyle_ok,true,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
		    #'bubbleRobClient',portNb.." "..leftMotor.." "..rightMotor.." "..noseSensor,0

  	rospy.sleep(0.1)

	# Streaming operation is enabled/disabled individually for each command and
	# object(s) the command applies to. In above case, only the joint position of
	# the joint with handle jointHandle will be streamed.

	# time.sleep(5) # pause needed for server side communication


if __name__ == '__main__':
  main()    

