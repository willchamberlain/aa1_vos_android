#!/usr/bin/env python

#
# ---- See http://wiki.ros.org/urdfdom_py ----
# 

# Load the urdf_parser_py manifest, you use your own package
# name on the condition but in this case, you need to depend on
# urdf_parser_py.
import roslib; roslib.load_manifest('urdfdom_py')
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
import tf

# Import the module

from urdf_parser_py.urdf import URDF

# Import VOS implementation

from vos_aa1.msg import WhereIsAsPub
from vos_aa1.srv import localise_by_visual_descriptor
from vos_aa1.srv import localise_by_visual_descriptorRequest
from vos_aa1.srv import localise_by_visual_descriptorResponse


# setup ROS nodes, subscribers, etc 

# visiontask_publisher = rospy.Publisher('/cam_607/vos_task_assignment_subscriber',WhereIsAsPub, queue_size=1)

visiontask_publisher_605 = rospy.Publisher('/cam_605/vos_task_assignment_subscriber',WhereIsAsPub, queue_size=10, latch=True)
visiontask_publisher_606 = rospy.Publisher('/cam_606/vos_task_assignment_subscriber',WhereIsAsPub, queue_size=10, latch=True)
visiontask_publisher_607 = rospy.Publisher('/cam_607/vos_task_assignment_subscriber',WhereIsAsPub, queue_size=10, latch=True)
visiontask_publisher_608 = rospy.Publisher('/cam_608/vos_task_assignment_subscriber',WhereIsAsPub, queue_size=10, latch=True)
visiontask_publisher_609 = rospy.Publisher('/cam_609/vos_task_assignment_subscriber',WhereIsAsPub, queue_size=10, latch=True)
visiontask_publisher_611 = rospy.Publisher('/cam_611/vos_task_assignment_subscriber',WhereIsAsPub, queue_size=10, latch=True)
visiontask_publisher_612 = rospy.Publisher('/cam_612/vos_task_assignment_subscriber',WhereIsAsPub, queue_size=10, latch=True)
visiontask_publisher_list = [visiontask_publisher_605,visiontask_publisher_606,visiontask_publisher_607,visiontask_publisher_608,visiontask_publisher_609,visiontask_publisher_611,visiontask_publisher_612] 


def localise_from_feature_from_visionsources():
    rospy.wait_for_service('/vos_server/localise_by_visual_descriptor')
    request = localise_by_visual_descriptorRequest()
    request.request_id  = "TODO request_id"
    request.algorithm   = "TODO algorithm"
    request.descriptor  = "TODO descriptor"
    request.rate        = 2
    request.return_url  = "TODO return_url"
    request.repetitions = 10000
    try:
        vos_server__srv__localise_by_visual_descriptor = rospy.ServiceProxy('/vos_server/localise_by_visual_descriptor', localise_by_visual_descriptor)
        response = vos_server__srv__localise_by_visual_descriptor(request)        
        print "localise_from_feature_from_visionsources: got a response"
        return response
    except rospy.ServiceException, e:
        print "ERROR: ---- localise_from_feature_from_visionsources: Service call failed: %s"%e


localise_from_feature_from_visionsources()


# 2. Load the module from a file.
# Pro: no need to have a roscore running.
# Cons: using hardcoded file location is not portable.
filepath = "/mnt/nixbig/build_workspaces/aa1_vos_android_catkin_ws/src/vos_aa1/resources/pioneer3dx_pioneer1.urdf"
print "------------- URDF from  http://wiki.ros.org/p2os_urdf  URDF file (%s): ---------------"%(filepath)
robot = URDF.from_xml_file(filepath)
print "-----------------------------------------------------"
print "     --------------------------     "
print "-- now exploring the model: "
print "-- robot has %d link elements."%(len(robot.links))
print "     --------------------------     "
print "-- now exploring the model: "
print "-- robot link[0] has %d appearance elements."%(len(robot.links[0].appearances))
print "-----------------------------------------------------"




print "start a service here, listening for VOS requests which can include URDFs"
print "..."
print "..."
print "..."
print "..."
print "..."
print "..."
print "receive a VOS request for localise_me(my_URDF)"


rospy.init_node("dev_python_URDF_file_to_WhereIsAsPub_messages")


appearance_num_ = 0
for appearance in robot.links[0].appearances:
    print "appearance number %d"%(appearance_num_)
    print "  allocate the task to camera(s) as a LocaliseFromView containing a set of VisualFeatureObservation[]"
    print appearance.origin
    print appearance.origin.rpy
    print appearance.origin.rpy[0]
    print appearance.origin.xyz
    whereis_message = WhereIsAsPub()
    whereis_message.request_id   = "900999991"
    whereis_message.algorithm    = appearance.algorithm #"BoofCV | binary square fiducial | 0.257 "
    whereis_message.descriptor   = appearance.descriptor
    pose_ = Pose()
    pose_.position.x=appearance.origin.xyz[0]
    pose_.position.y=appearance.origin.xyz[1]
    pose_.position.z=appearance.origin.xyz[2]
    quaternion = tf.transformations.quaternion_from_euler(appearance.origin.rpy[0], appearance.origin.rpy[1], appearance.origin.rpy[2])
    print "quaternion = %d %d %d %d"%(quaternion[0],quaternion[1],quaternion[2],quaternion[3]) 
    print quaternion
    pose_.orientation.x=  quaternion[0]
    pose_.orientation.y=  quaternion[1]
    pose_.orientation.z=  quaternion[2]
    pose_.orientation.w=  quaternion[3]
    print pose_.orientation
    whereis_message.relation_to_base = pose_
    whereis_message.rate = 100000
    whereis_message.return_url = 'bob'
    print "publish appearance number %d"%(appearance_num_)
    for visiontask_publisher_ in visiontask_publisher_list:
        visiontask_publisher_.publish(whereis_message)
    appearance_num_ = appearance_num_ + 1
    print "published appearance number %d"%(appearance_num_)
    print "... tick ... having published, now need to give ROSJava time to pick up the messages - don't assume that the subscriber can pick up immediately, e.g. there may be latency due to the smart camera controller or network communications "
    rospy.rostime.wallsleep(0.5)    
    print "... tock ... tick ..."
    rospy.rostime.wallsleep(0.5)    
    print "... tock ... now complete."
# having published, now need to give ROSJava time to pick up the messages - don't assume that the subscriber can pick up immediately, e.g. there may be latency due to the smart camera controller or network communications 
print "... tick ... having published, now need to give ROSJava time to pick up the messages - don't assume that the subscriber can pick up immediately, e.g. there may be latency due to the smart camera controller or network communications "
rospy.rostime.wallsleep(0.5)    
print "... tock ... tick ..."
rospy.rostime.wallsleep(0.5)    
print "... tock ... now complete."

## keep thread alive
#while not rospy.core.is_shutdown():    
#    # do stuff 
#    print "... tick ..."
#    rospy.rostime.wallsleep(0.5)


# end


