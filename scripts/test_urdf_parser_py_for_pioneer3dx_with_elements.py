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
print "     --------------------------     "
print "-- now exploring the model: "
print "-- robot has %d link elements."%(len(robot.links))
print "-----------------------------------------------------"

# - OR -

# 2. Load the module from a file.
# Pro: no need to have a roscore running.
# Cons: using hardcoded file location is not portable.
filepath = "/mnt/nixbig/downloads/ros_p2os/p2os/p2os_urdf/defs/pioneer3dx.urdf"
print "------------- URDF from  http://wiki.ros.org/p2os_urdf  URDF file (%s): ---------------"%(filepath)
robot = URDF.from_xml_file(filepath)
print "-----------------------------------------------------"
print "     --------------------------     "
print "-- now exploring the model: "
print "-- robot has %d link elements."%(len(robot.links))
print "     --------------------------     "
print "-- now exploring the model: "
print "-- robot link[0] has %d appearance elements."%(len(robot.links[0].appearances))
print "-- robot link[0].appearances[0] has '%s' descriptor."%(robot.links[0].appearances[0].descriptor)
print "-- robot link[0].appearances[1] has '%s' descriptor."%(robot.links[0].appearances[1].descriptor)
print "-----------------------------------------------------"




print "start a service here, listening for VOS requests which can include URDFs"
print "..."
print "..."
print "..."
print "..."
print "..."
print "..."
print "receive a VOS request for localise_me(my_URDF)"

appearance_num_ = 0
for appearance in robot.links[0].appearances:
    print "appearance number %d"%(appearance_num_)
    print "  allocate the task to camera(s) as a LocaliseFromView containing a set of VisualFeatureObservation[]"
    appearance_num_ = appearance_num_ + 1
    


#filepath = "/mnt/nixbig/downloads/ros_amr-ros-config/amr-ros-config/description/urdf/pioneer3dx.urdf"
#print "------------- URDF from Omron Adept MobileRobots Pioneer 3DX URDF file (%s): ---------------"%(filepath)
#robot = URDF.from_xml_file(filepath)
#print "-----------------------------------------------------"

# - OR -

# 3. Load the module from the parameter server.
# Pro: automatic, no arguments are needed, consistent
#      with other ROS nodes.
# Cons: need roscore to be running and the parameter to
#      to be set beforehand (through a roslaunch file for
#      instance).
#print "------------- URDF from parameter server: ---------------"
#robot = URDF.from_parameter_server()
#print "-----------------------------------------------------"

# Print the robot
# print(robot)
