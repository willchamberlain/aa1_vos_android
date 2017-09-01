#!/usr/bin/env python

#
# ---- See http://wiki.ros.org/urdfdom_py ----
# 

# Load the urdf_parser_py manifest, you use your own package
# name on the condition but in this case, you need to depend on
# urdf_parser_py.
import roslib; roslib.load_manifest('urdfdom_py')
import rospy

# Import the module

from urdf_parser_py.urdf import URDF

# 1. Parse a string containing the robot description in URDF.
# Pro: no need to have a roscore running.
# Cons: n/a
# Note: it is rare to receive the robot model as a string.
print "--------------- URDF from XML string: ---------------"
robot = URDF.from_xml_string("<robot name='myrobot'></robot>")
print "-----------------------------------------------------"


print "--------------- Minimal URDF from file: ---------------"
robot = URDF.from_xml_file("/mnt/nixbig/build_workspaces/aa1_vos_android_catkin_ws/src/vos_aa1/scripts/test_urdf_parser_py_minimal_urdf.xml")
print "-----------------------------------------------------"

# - OR -

# 2. Load the module from a file.
# Pro: no need to have a roscore running.
# Cons: using hardcoded file location is not portable.
filepath = "/mnt/nixbig/downloads/ros_p2os/p2os/p2os_urdf/defs/pioneer3dx.urdf"
print "------------- URDF from  http://wiki.ros.org/p2os_urdf  URDF file (%s): ---------------"%(filepath)
robot = URDF.from_xml_file(filepath)
print "-----------------------------------------------------"

filepath = "/mnt/nixbig/downloads/ros_amr-ros-config/amr-ros-config/description/urdf/pioneer3dx.urdf"
print "------------- URDF from Omron Adept MobileRobots Pioneer 3DX URDF file (%s): ---------------"%(filepath)
robot = URDF.from_xml_file(filepath)
print "-----------------------------------------------------"

# - OR -

# 3. Load the module from the parameter server.
# Pro: automatic, no arguments are needed, consistent
#      with other ROS nodes.
# Cons: need roscore to be running and the parameter to
#      to be set beforehand (through a roslaunch file for
#      instance).
print "------------- URDF from parameter server: ---------------"
robot = URDF.from_parameter_server()
print "-----------------------------------------------------"

# Print the robot
print(robot)
