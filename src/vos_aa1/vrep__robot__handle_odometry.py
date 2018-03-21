#!/usr/bin/env python

import rospy
import tf
import vrep
import time
import sys, signal

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
def main():
	signal.signal(signal.SIGINT, signal_handler)
	#node_name     			= rospy.get_param("~node_name","_node")								#	"base_link"
	initialise_ROS_node( "vrep__robot__handle_odometry_node" , 0.5 )

	base_link_object_name_in_VRep     = rospy.get_param("~base_link_object_name_in_VRep")								#	"base_link"
	new_child_frame     = rospy.get_param("~new_child_frame") 				# "base_link"
	old_parent_frame    = rospy.get_param("~old_parent_frame")						#	"odom"
	new_parent_name     = rospy.get_param("~new_parent_frame")				#	"odom"


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
	res, base_link_Orientation 	= vrep.simxGetObjectOrientation(clientID, base_link_Handle, -1, vrep.simx_opmode_streaming)		# setup stream # absolute
	print 'got simxGetObjectOrientation simx_opmode_streaming'		

		# The control loop:		-		http://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm#streaming
	while vrep.simxGetConnectionId(clientID) != -1 :  # while we are connected to the server..

		res_2, base_link_Position 		= vrep.simxGetObjectPosition(clientID, base_link_Handle, -1, vrep.simx_opmode_buffer)		    # get from stream # absolute
		print 'got simxGetObjectPosition simx_opmode_buffer'	
		print base_link_Position
		# roll, pitch, yaw in rads 
		res_1, base_link_Orientation 	= vrep.simxGetObjectOrientation(clientID, base_link_Handle, -1, vrep.simx_opmode_buffer)		# get from stream # absolute
		print 'got simxGetObjectOrientation simx_opmode_buffer'	
		print base_link_Orientation

		# Fetch the newest joint value from the inbox (func. returns immediately (non-blocking)):
		if res_1 and res_2:
			print ' here we have the newest joint position in variable jointPosition!'
		    # here we have the newest joint position in variable jointPosition!    
		else:
			print 'else - still waiting'
		    # once you have enabled data streaming, it will take a few ms until the first value has arrived. So if
		    # we landed in this code section, this does not always mean we have an error!!!

  	rospy.sleep(0.02)

	# Streaming operation is enabled/disabled individually for each command and
	# object(s) the command applies to. In above case, only the joint position of
	# the joint with handle jointHandle will be streamed.

	# time.sleep(5) # pause needed for server side communication


if __name__ == '__main__':
  main()    

