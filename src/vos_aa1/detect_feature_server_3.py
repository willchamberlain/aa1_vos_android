#!/usr/bin/env python

import sys
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Twist, TwistWithCovariance  # https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal

from tf import transformations
import numpy as np
from visualization_msgs.msg import Marker
import threading
from pprint import pprint
import std_msgs
import time
from datetime import timedelta, datetime
import threading
from itertools import chain

from threading import Lock

import load_properties


import jsonpickle
from collections import deque


from vos_aa1.srv import DetectedFeature
from vos_aa1.srv import DetectedFeatureRequest
from vos_aa1.srv import DetectedFeatureResponse
from vos_aa1.srv import LocaliseFromAFeature
from vos_aa1.srv import LocaliseFromAFeatureRequest
from vos_aa1.srv import LocaliseFromAFeatureResponse
from vos_aa1.srv import RegisterVisionSource
from vos_aa1.srv import RegisterVisionSourceRequest 
from vos_aa1.srv import RegisterVisionSourceResponse
from vos_aa1.srv import there_is_alg_desc
from vos_aa1.srv import there_is_alg_descRequest
from vos_aa1.srv import there_is_alg_descResponse
from vos_aa1.msg import WhereIsAsPub
from vos_aa1.srv import localise_by_visual_descriptor
from vos_aa1.srv import localise_by_visual_descriptorRequest
from vos_aa1.srv import localise_by_visual_descriptorResponse
from vos_aa1.msg import VisualFeatureInWorld
from vos_aa1.msg import VisualFeatureObservation

from file_checker import *

algorithm_abreviations = {'AprilTags_Kaess_36h11':'t'}


#
fixed_features = []
robotPoseHistory = []
targetPoseHistory = []

# tag_55 = VisualFeatureInWorld()
# tag_55.algorithm = 'AprilTags_Kaess_36h11'
# tag_55.id = '55'
# tag_55.pose = Pose()
# tag_55.pose.position.x=0
# tag_55.pose.position.y=0
# tag_55.pose.position.z=0.8
# tag_55.pose.orientation.x=0
# tag_55.pose.orientation.y=0
# tag_55.pose.orientation.z=0
# tag_55.pose.orientation.w=1
# fixed_features.append(tag_55)

tag_210 = VisualFeatureInWorld()
tag_210.algorithm = 'AprilTags_Kaess_36h11'
tag_210.id = '210'
tag_210.pose = Pose()
tag_210.pose.position.x=3
tag_210.pose.position.y=-3.5
tag_210.pose.position.z=1.25
tag_210.pose.orientation.x=0
tag_210.pose.orientation.y=0
tag_210.pose.orientation.z=-0.619
tag_210.pose.orientation.w=0.785390985 
fixed_features.append(tag_210)

cam_poses_on_cam      = ( 40057, 40157, 40257, 40357, 40457, 40557)
print "cam_poses_on_cam = %s"%(str(cam_poses_on_cam))

robot_features_on_cam = ( 50057, 50157, 50257, 50357, 50457, 50557)
print "robot_features_on_cam=%s"%(str(robot_features_on_cam))

features_present_list_of_tuples = [ cam_poses_on_cam , robot_features_on_cam ]
features_on_cam = tuple(chain.from_iterable(features_present_list_of_tuples))
print "features_on_cam=%s"%(str(features_on_cam))

robot_features_via_tf = ( 80557, 70557, 60557)
features_present_list_of_tuples = [ robot_features_via_tf , features_on_cam ]
robot_features_present = tuple(chain.from_iterable(features_present_list_of_tuples))
print "robot_features_present=%s"%(str(robot_features_present))

other_features_present = ( 90170 , 90250 , 90290 , 90330 , -90000 , 90555 , 90210 , 91610 , 91690 , 91730 , 91650 , 90057 , 90157 , 90257 , 90357 , 90457 , 90557 , 90000+999999 , 70170 , 60170 , 50170 )
print "other_features_present=%s"%(str(other_features_present)) 

features_present_list_of_tuples = [robot_features_present,other_features_present]
print "features_present_list_of_tuples=%s"%(features_present_list_of_tuples)
features_present = tuple(chain.from_iterable(features_present_list_of_tuples))
features_present = tuple(sorted(features_present))
print "features_present=%s"%(str(features_present))
type(features_present[0])
print "features_present(0)=%s of type=%s"%(features_present[0],type(features_present[0]))


tag_210_target_publisher    = 1
tag_210_target_pose = Pose()

vision_sources = []


marker_publisher  = 1
pose_publisher    = 1
initialpose_poseWCS_publisher = 1
fakelocalisation_poseWCS_publisher = 1

tfBroadcaster__ = []
tfListener      = 1

# robot task information demands
localise_by_visual_descriptor_server = 1

# Server-internal - interfaces to other nodes of the VosServer
#list_vision_sources_publisher = 1


# monitoring
management_subscriber = 1
retarget_requested = False
detection_true_monitoring_publisher = 1
detection_false_monitoring_publisher = 1


robot_pose_estimates_list = []
averaging_lock = Lock()


def set_tfBroadcaster(tfBroadcaster_):
    tfBroadcaster__.append(tfBroadcaster_)

def get_tfBroadcaster():
    return tfBroadcaster__[0]



# adapted from https://afni.nimh.nih.gov/pub/dist/src/pkundu/meica.libs/nibabel/quaternions.py
def conjugate(q):
    ''' Conjugate of quaternion

    Parameters
    ----------
    q : 4 element sequence
       i, j, k, w of quaternion

    Returns
    -------
    conjq : array shape (4,)
       i, j, k, w of conjugate of `q`
    '''
    return np.array(q) * np.array([-1, -1, -1, 1.0])

# adapted from https://afni.nimh.nih.gov/pub/dist/src/pkundu/meica.libs/nibabel/quaternions.py
def norm(q):
    ''' Return norm of quaternion

    Parameters
    ----------
    q : 4 element sequence
       i, j, k, w of quaternion

    Returns
    -------
    n : scalar
       quaternion norm
    '''
    return np.dot(q, q)


# adapted from https://afni.nimh.nih.gov/pub/dist/src/pkundu/meica.libs/nibabel/quaternions.py
def inverse(q):
    ''' Return multiplicative inverse of quaternion `q`

    Parameters
    ----------
    q : 4 element sequence
       i, j, k, w of quaternion

    Returns
    -------
    invq : array shape (4,)
       i, j, k, w of quaternion inverse
    '''
    return conjugate(q) / norm(q)

def localise_from_a_feature_callback(req):
    position_   = [0,0,0]
    quaternion_ = [0,0,0,1]
    time_now = rospy.Time.now()
    response = LocaliseFromAFeatureResponse()
    print "----------------------------------------------"
    print "localise_from_a_feature_callback: %d"%(req.visualFeature.id)
    cN = req.visualFeature.pose.header.frame_id
    visual_feature_descriptor = req.visualFeature.id

    #  1) set up the tf names to use as variables
    #  2) use those variables to pull the tfs and respond to the camera self-localisation request
    #    for x in fixed_features:
    #        print '---- fixed_feature exists with id "',x.id,'"'

    if any( x.id == visual_feature_descriptor or str(x.id) == str(visual_feature_descriptor) for x in fixed_features):
        print '---- localise_from_a_feature_callback: fixed feature "',req.visualFeature.id,'" matches descriptor "',visual_feature_descriptor,'"'
        tag_same_fixed = '%s%s'%(algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)
        c2_from_fixed_t55 = '%s_from_fixed_%s'%(cN,tag_same_fixed)
        try:
            tfListener.waitForTransform('map', c2_from_fixed_t55, rospy.Time(), rospy.Duration(4.0))      # from the map origin, to the camera, through a fixed point
            position_, quaternion_ = tfListener.lookupTransform('map', c2_from_fixed_t55,  rospy.Time())
            response.pose.position.x = position_[0]
            response.pose.position.y = position_[1]
            response.pose.position.z = position_[2]
            response.pose.orientation.x = quaternion_[0]
            response.pose.orientation.y = quaternion_[1]
            response.pose.orientation.z = quaternion_[2]
            response.pose.orientation.w = quaternion_[3]
            response.poseFound = True
            print '---- localise_from_a_feature_callback: '
        except:
            print '---- localise_from_a_feature_callback: exception'
            #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            response.poseFound = False
    else:
        print '---- localise_from_a_feature_callback: no fixed features match descriptor "', visual_feature_descriptor,'"'
        # default to not found, then check the cameras
        response.poseFound = False
        print '---- localise_from_a_feature_callback: sending request on to sources for descriptor"', visual_feature_descriptor,'"'
        #return_url = cN;
        response = localise_from_feature_from_visionsources(req)
#        try:
            # NOT going to use a return URL for this: the vision sources register with their base URLs, so we can take their localise_from_feature service URLs relative to that  -- return_url = 'c3/localise_from_feature_return'  or some such - to get back to the camera(s)
            # NOT going to check FoV: the server doesn't know where that feature is in the world   --   fov = True or some such region of interest -
            # NOT for this func tion, but for the detectFreeSpace or whatever later NOTE: should probably go back and see if there is good code to re-use from Aug-Oct 2015
# leave off threading for now            thread = Thread(target = localise_from_feature_from_visionsources(return_url,visual_feature_descriptor,fov))  # https://www.tutorialspoint.com/python/python_multithreading.htm
#            response = localise_from_feature_from_visionsources(req)
#        except:
#           print "ERROR: ---- localise_from_a_feature_callback: unable to start threads: error = ", sys.exc_info()[0]
#           raise
    return response


def localise_from_feature_from_visionsources(LocaliseFromAFeatureRequest_):
    for vision_source in vision_sources:
        localise_service_url = '%s/localise_from_a_feature'%vision_source.base_url
        rospy.wait_for_service(localise_service_url)
        try:
            cam_localise_serviceproxy = rospy.ServiceProxy(localise_service_url, LocaliseFromAFeature)
            response = cam_localise_serviceproxy(LocaliseFromAFeatureRequest_)
            print 'localise_from_feature_from_source: service call succeeded'
            if response.poseFound:
                return response
                # TODO:  average over all responses, or some such, rather than just return the first match found
        except rospy.ServiceException, e:
            print "ERROR: ---- localise_from_feature_from_source: Service call failed: %s"%e
    # no location found for that feature: return poseFound=False, empty pose
    response = LocaliseFromAFeatureResponse()
    response.poseFound = False
    return response


def distribute_to_visionsources(return_url_,visual_feature_descriptor_,fov_):
    for vision_source in vision_sources:
        rospy.wait_for_service(vision_source.base_url)
        try:
            cam_localise_serviceproxy = rospy.ServiceProxy(vision_source.base_url, visual_feature_descriptor_,fov_)
            ack = cam_localise_serviceproxy(x, y)
            print 'localise_from_feature_from_source: service call succeeded'
        except rospy.ServiceException, e:
            print "localise_from_feature_from_source: Service call failed: %s"%e


def localise_by_visual_descriptor_callback(request):
  # print "localise_by_visual_descriptor_callback: algorithm=%s, descriptor=%s, rate=%d"%(request.algorithm,request.descriptor,request.rate)

  for vision_source in vision_sources:
    whereIsAsPub = WhereIsAsPub()
    whereIsAsPub.algorithm  = request.algorithm
    whereIsAsPub.descriptor = request.descriptor
    whereIsAsPub.rate       = request.rate
    vision_source.publisher_to_phone.publish(whereIsAsPub)
    print "localise_by_visual_descriptor_callback: vision_source_id=%s : algorithm=%s, descriptor=%s, rate=%d"%(vision_source.vision_source_id, request.algorithm,request.descriptor,request.rate)

  response = localise_by_visual_descriptorResponse()
  response.acknowledgment = 'OK'
  return response


def detect_feature_callback(req):
    # print "----------------------------------------------"
    # print "detect_feature_callback: "
    # print "  camera frame_id [%s] :"%(req.cameraPose.header.frame_id)
    # print "  camera translation from map [%.4f,%.4f,%.4f] : "%(req.cameraPose.pose.position.x,req.cameraPose.pose.position.y,req.cameraPose.pose.position.z)
    # print "  camera orientation (quaternion) [%.4f,%.4f,%.4f,%.4f] : "%(req.cameraPose.pose.orientation.x,req.cameraPose.pose.orientation.y,req.cameraPose.pose.orientation.z,req.cameraPose.pose.orientation.w)
    # print "  feature algorithm [%s] : "%(req.visualFeature.algorithm)
    # print "  feature id [%d] : "%(req.visualFeature.id)
    # print '  feature frame_id : %s_%s%s' % (req.cameraPose.header.frame_id, algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)
    # print "  feature translation [%.4f,%.4f,%.4f] : "%(req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z)
    # print "  feature orientation (quaternion) [%.4f,%.4f,%.4f,%.4f] "%(req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w)
    global robotPoseHistory
    global targetPoseHistory

    cN = req.cameraPose.header.frame_id

    if req.visualFeature.id not in features_present:
        print "detect_feature_callback: camera frame_id [%s] : feature id [%d] : feature is not present - is a false positive - not listing as a detection."%(req.cameraPose.header.frame_id, req.visualFeature.id)
        detection_false_monitoring_publisher.publish("False detection: camera frame_id [%s] : feature id [%d]")
        response = DetectedFeatureResponse()
        response.acknowledgement="feature not present"
        return response
    else:
        print "detect_feature_callback: camera frame_id [%s] : feature id [%d] : feature is present - is a true positive - listing as a detection."%(req.cameraPose.header.frame_id, req.visualFeature.id)

    detection_true_monitoring_publisher.publish("True detection: camera frame_id [%s] : feature id [%d]")

    # # TODO - something about this conversion is not right: the translation and quaternion match those found by AprilTags_Kaess and sent by DetectedFeatureClient, but the RPY are different
    # euler = tf.transformations.euler_from_quaternion([req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w])
    # print "  feature x_y_z_w roll=%.4f pitch=%.4f yaw=%.4f"%(euler[0], euler[1], euler[2])

    tfBroadcaster = tf.TransformBroadcaster()
    time_now      = rospy.Time.now()


    ### fixed feature / fixed tag / fixed marker 
    for fixed_feature in fixed_features:
        tfBroadcaster.sendTransform(
            (fixed_feature.pose.position.x, fixed_feature.pose.position.y, fixed_feature.pose.position.z ),
            (fixed_feature.pose.orientation.x, fixed_feature.pose.orientation.y, fixed_feature.pose.orientation.z, fixed_feature.pose.orientation.w),
            time_now,
            '%s%s' % (algorithm_abreviations[fixed_feature.algorithm], fixed_feature.id),
            'map')
        tfBroadcaster.sendTransform(
            (fixed_feature.pose.position.x, fixed_feature.pose.position.y, fixed_feature.pose.position.z ),
            (fixed_feature.pose.orientation.x, fixed_feature.pose.orientation.y, fixed_feature.pose.orientation.z, fixed_feature.pose.orientation.w),
            time_now,
            'fixed_%s%s' % (algorithm_abreviations[fixed_feature.algorithm], fixed_feature.id),
            'map')
            
            


# working through BoofCV transforms and orientations: BoofCV uses yet another frame for tags
    c2 = req.cameraPose.header.frame_id                          # c2, c11, ... , cN     : DOES include the 'c'
    t = algorithm_abreviations[req.visualFeature.algorithm]     # 't'
    t_id = req.visualFeature.id                                 # 0, 2, 170, 210,       : does NOT include any 't'
    marker_tag_id = t_id
    t55 = "%s%s"%(t,t_id)
    
    
    print "-------------"
    print "-------------"
    print "-------------"
    print "-------------"
    print "-------------"
    print " c2='%s' , t55='%s' "%(c2, t55)
    print "-------------"
    print "-------------"
    print "-------------"
    print "-------------"
    print "-------------"
    # camera reporting robot pose rather than feature - TODO - move this to another service than DetectedFeature
    if  marker_tag_id in features_on_cam: # ['t50557', 't40557']:    #  t40557 is the world-to-camera  , t50557 is the world-to-marker , t60557 is the camera-to-marker , t70557 is the camera-to-robot-base 
        dum_c2_map_to_robot_via_t55_trans = "dum_%s_map_to_robot_via_%s_trans"%(c2,t55)
        print " t55 in ['50557', 't40557'] : t55='%s' : publishing tf from /map to %s "%(t55 , dum_c2_map_to_robot_via_t55_trans)
        tfBroadcaster.sendTransform(
            (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
            (0,0,0,1),                          # apply the translation first ...
            time_now,
            dum_c2_map_to_robot_via_t55_trans,  # to
            'map')                              # from    

        dum_c2_map_to_robot_via_t55_trans_rot = "dum_%s_map_to_robot_via_%s_trans_rot"%(c2,t55)
        print " t55 in ['50557', 't40557'] : t55='%s' : publishing tf from %s to %s "%(t55 , dum_c2_map_to_robot_via_t55_trans , dum_c2_map_to_robot_via_t55_trans_rot)
        tfBroadcaster.sendTransform(
            (0,0,0),                            # ... then apply the rotation - because the Python does things _differently_. 
            (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
            time_now,
            dum_c2_map_to_robot_via_t55_trans_rot,    # to
            dum_c2_map_to_robot_via_t55_trans)        # from
            
        if  marker_tag_id in robot_features_on_cam:     
            tfListener.waitForTransform('map', dum_c2_map_to_robot_via_t55_trans_rot, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', dum_c2_map_to_robot_via_t55_trans_rot, rospy.Time(0))            
            
            position_now_ = (pos_[0], pos_[1], pos_[2])
            global robot_pose_estimates_list
            global averaging_lock
            datetime_0 = datetime.utcnow()
            robot_pose_estimates_list.append((datetime_0, pos_[0], pos_[1], pos_[2]))
            
            
            
            if len(robot_pose_estimates_list) > 5 :            
                print "averaging before lock: len(robot_pose_estimates_list)=%d"%(len(robot_pose_estimates_list))
                averaging_lock.acquire()
                print "averaging inside lock"
                try:
                    i_ = 0
                    average_pos = [0.0,0.0,0.0]
                    average_pos_ = [0.0,0.0,0.0]
                    num_averaged = 0.0
                    while i_ < len(robot_pose_estimates_list) :   
                        position_then_ = robot_pose_estimates_list[i_]   
                        i_ += 1
                        timedelta_0_then = datetime_0 - position_then_[0]
                        if  timedelta_0_then.seconds < 1.0 :  # less than x seconds old                       
                            num_averaged += 1.0
                            average_pos[0] += position_then_[1]
                            average_pos[1] += position_then_[2]
                            average_pos[2] += position_then_[3]      
                            print position_then_
                    print "averaged %d poses"%(num_averaged)        
                    average_pos_[0] = average_pos[0]/num_averaged
                    average_pos_[1] = average_pos[1]/num_averaged
                    average_pos_[2] = average_pos[2]/num_averaged        
                    print average_pos_
                    publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', average_pos_[0], average_pos_[1], average_pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942])
                    publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', average_pos_[0], average_pos_[1], average_pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])                    
                finally:
                    averaging_lock.release() # release lock, no matter what                    
            else :
                print "not averaging: len(robot_pose_estimates_list)=%d"%(len(robot_pose_estimates_list))
                publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942])
                publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            
            
#        response = DetectedFeatureResponse()
#        response.acknowledgement="bob"
#        return response
    # back to the other tags         
        

# the camera's self-reported pose in the world after localisation
    # map_to_c2 = "map_to_%s"%(c2)
    # tfBroadcaster.sendTransform(
    #     (req.cameraPose.pose.position.x,req.cameraPose.pose.position.y,req.cameraPose.pose.position.z),
    #     (req.cameraPose.pose.orientation.x,req.cameraPose.pose.orientation.y,req.cameraPose.pose.orientation.z,req.cameraPose.pose.orientation.w),
    #     time_now,
    #     map_to_c2,              # to
    #     'map')                  # from


# Fixed camera poses - square plus a diagonal
    # catch-all
    dum_c2 = "dum_%s"%(c2)

# fixed camera pose
#   dummy frame for the camera: 1m up away from origin, same orientation - facing toward the lobby
#   TODO : use load_properties, or jsonpickle, to read - and reload - from file
#   fixed camera
    if  c2 in ['c10', 'c15']:
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            (1.9, -5.75, 0.57),
            (0,0,0.707106781,0.707106781),
            time_now,
            dum_c2,                         # to
            'map')                          # from

# fixed camera pose
#   dummy frame for the camera: 1m up away from origin, facing right - toward S1165
#   TODO : use load_properties, or jsonpickle, to read - and reload - from file
#   fixed camera
    if c2 in ['c11', 'c12']:
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            (1.9, -5.75, 0.57),
            (0,0,0,1),
            time_now,
            dum_c2,                         # to
            'map')                          # from

# fixed camera pose
#   dummy frame for the camera: 1m up away from origin, facing 45 degrees right - toward corner of Michael's office
#   TODO : use load_properties, or jsonpickle, to read - and reload - from file
#   fixed camera
    if c2 in ['c20', 'c21']:
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            (1.85, -5.95, 0.74),
            (0, 0, 0.383, 0.92374834235304581),
            time_now,
            dum_c2,                         # to
            'map')                          # from

# end Fixed camera poses - square plus a diagonal

# Fixed camera poses Configuration B - two at 120 degrees: tripods back-to-back with feet touching

# toward Michael's office/S1125
    if c2 in ['c60']:
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            (1.85, -5.95, 0.74),
            (0, 0, 0.131, 0.991382368),
            time_now,
            dum_c2,                         # to
            'map')                          # from
# toward lobby/paper wall
    if c2 in ['c70']:
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            (1.85, -5.95, 0.74),
            (0, 0, 0.609, 0.793170221),
            time_now,
            dum_c2,                         # to
            'map')                          # from

    if c2 in ['c35']:
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            ( 1.28985595703, 2.43800625801,  0.979),
            (0.033861 , 0.152731 , -0.213782 , 0.964274),
            time_now,
            dum_c2,                         # to
            'map')                          # from
    if c2 in ['c45']:
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            ( 1.28985595703, 2.33800625801,  0.979),
            (-0.089342 , 0.215681 , 0.372121 , 0.898346),
            time_now,
            dum_c2,                         # to
            'map')                          # from
    if c2 in ['c30']:
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            (1.28985595703, 2.33800625801-0.689+0.1,  0.56),
            (0,     0,      -0.382392469,   0.924),
            time_now,
            dum_c2,                         # to
            'map')                          # from
    if c2 in ['c40']:
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            (1.28985595703, 2.33800625801-0.689,  0.56),
            (0,     0,      -0.707106781,   0.707106781),
            time_now,
            dum_c2,                         # to
            'map')                          # from
            
    if c2 in ['c100']:                      # VOS check Nexus 6 accuracy - Nexus 6
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            (2.74980449677, -9.07514190674,  0.88 ),
            (0,     0,      0.707106781,   0.707106781),
            time_now,
            dum_c2,                         # to
            'map')                          # from
    if c2 in ['c110']:                      # VOS check Nexus 6 accuracy - Samsung Galaxy 3
        cam_110_translation = (2.84980449677, -9.07514190674,  0.84 )
        cam_110_rotation    = (0,     0,      0.707106781,   0.707106781)
        cam_110_parent_frame = 'map'
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            cam_110_translation, #(2.84980449677, -9.07514190674,  0.84 ),
            cam_110_rotation,    #(0,     0,      0.707106781,   0.707106781),
            time_now,
            dum_c2,                         # to
            cam_110_parent_frame)                          # from

    if c2 in ['c110','c210','c310','c410','c510']:                      # End of S1130 by Sarah Allen desk. Facing toward PC office.    x: -7.20502614975  y: -8.07002162933
        cam_translation = (-7.20502614975, -8.07002162933,  0.84 )
        cam_rotation    = (0,     0,      0,   1)
        cam_parent_frame = 'map'
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            cam_translation, #(2.84980449677, -9.07514190674,  0.84 ),
            cam_rotation,    #(0,     0,      0.707106781,   0.707106781),
            time_now,
            dum_c2,                         # to
            cam_parent_frame)                          # from
    if c2 in ['c220']:                      # End of S1130 by Sarah Allen desk. Facing toward PC office.      x: 3.11342740059 y: -8.03106594086
        cam_translation = (3.11342740059, -8.03106594086,  0.84 )
        cam_rotation    = (0,     0,      0,   1)
        cam_parent_frame = 'map'
        dum_c2 = "dum_%s"%(c2)
        tfBroadcaster.sendTransform(
            cam_translation, #(2.84980449677, -9.07514190674,  0.84 ),
            cam_rotation,    #(0,     0,      0.707106781,   0.707106781),
            time_now,
            dum_c2,                         # to
            cam_parent_frame)                          # from
                        
            

# end Fixed camera poses - two at 120 degrees: tripods back-to-back with feet touching


    # just as the visual feature is reported from the camera/robot
    t55_trans_from_dum_c2 = "dum_%s_trans_to_%s"%(c2,t55)
    tfBroadcaster.sendTransform(
        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        # (req.visualFeature.pose.pose.position.z, req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y),
        (0,0,0,1),
        time_now,
        t55_trans_from_dum_c2,                          # to   tag-from-camera-body t55_from_c2
        dum_c2)                                         # from camera-body c2

#    t55_trans_from_dum_c2__drop = "z_dum_%s_trans_to_%s__drop"%(c2,t55)
#    tfBroadcaster.sendTransform(
#        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y,0),
#        (0,0,0,1),
#        time_now,
#        t55_trans_from_dum_c2__drop,                          # to   tag-from-camera-body t55_from_c2
#        '/map')                                         # from camera-body c2
#    axisMarker(req.visualFeature.id,t55_trans_from_dum_c2__drop)


    t55_transrot_from_dum_c2 = "dum_%s_trans_rot_to_%s"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        t55_transrot_from_dum_c2,
        t55_trans_from_dum_c2)

    t55_transrot_from_dum_c2_180x = "dum_%s_trans_rot_to_%s_180x"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (1,0,0,0),
        time_now,
        t55_transrot_from_dum_c2_180x,
        t55_transrot_from_dum_c2)

#    t55_transrot_from_dum_c2_b = "dum_%s_trans_rot_to_%s_b"%(c2,t55)
#    tfBroadcaster.sendTransform(
#        (0,0,0),
#        (req.visualFeature.pose.pose.orientation.x, -req.visualFeature.pose.pose.orientation.y, -req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
#        time_now,
#        t55_transrot_from_dum_c2_b,
#        t55_transrot_from_dum_c2 )

    t55_transrot_from_dum_c2_pre90y = "dum_%s_trans_rot_to_%s_pre90y"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (0, 0.707106781, 0, 0.707106781),
        time_now,
        t55_transrot_from_dum_c2_pre90y,
        t55_trans_from_dum_c2)

    t55_transrot_from_dum_c2_pre90y90z = "dum_%s_trans_rot_to_%s_pre90y90z"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (0, 0, 0.707106781, 0.707106781),
        time_now,
        t55_transrot_from_dum_c2_pre90y90z,
        t55_transrot_from_dum_c2_pre90y)

    t55_transrot_from_dum_c2_pre90y180z = "dum_%s_trans_rot_to_%s_pre90y180z"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (0, 0, 1.0, 0),
        time_now,
        t55_transrot_from_dum_c2_pre90y180z,
        t55_transrot_from_dum_c2_pre90y)

    # t55_transrot_from_dum_c2_post90y = "dum_%s_trans_rot_to_%s_post90y"%(c2,t55)
    # tfBroadcaster.sendTransform(
    #     (0,0,0),
    #     (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
    #     time_now,
    #     t55_transrot_from_dum_c2_post90y,
    #     t55_transrot_from_dum_c2_pre90y)

    t55_transrot_from_dum_c2_post90y90z = "dum_%s_trans_rot_to_%s_post90y90z"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        t55_transrot_from_dum_c2_post90y90z,
        t55_transrot_from_dum_c2_pre90y90z)

    t55_transrot_from_dum_c2_post90y180z = "dum_%s_trans_rot_to_%s_post90y180z"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        t55_transrot_from_dum_c2_post90y180z,
        t55_transrot_from_dum_c2_pre90y180z)

    t55_transrot_from_dum_c2_post90y180zneg90z = "dum_%s_trans_rot_to_%s_post90y180zneg90z"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (0, 0, -0.707106781, 0.707106781),
        time_now,
        t55_transrot_from_dum_c2_post90y180zneg90z,
        t55_transrot_from_dum_c2_post90y180z)



    #  TODO - load from file with load_properties or jsonpickle, and also move into the robot's request
    #  ROBOT VISUAL MODEL / robot model if 't9250'==t55:
    if 't90171'==t55: # 't90170'==t55:
        t55_transrot_from_dum_c2_robot_pose_170 = "dum_%s_trans_rot_to_%s_robot_pose_170"%(c2,t55)
        tfBroadcaster.sendTransform(
            # pioneer - the tall one ( -0.12, 0, -0.76),                             # before rot, step back --> forward
            ( 0.12-0.18, 0, -0.61),                     # forward of centre of tag box, tag box centre is 18cm rear of base_link (Pioneer2)
            (0, 0, 0, 1),                               # 170 is on the front
            time_now,
            t55_transrot_from_dum_c2_robot_pose_170,
            t55_transrot_from_dum_c2_post90y180zneg90z)
#        tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_250, rospy.Time(), rospy.Duration(0))      # from the map origin, to the robot
        try:  #  TODO - produce a unified pose estimate from this set of observations of this robot model, on the phone side - in this case average the quaternions, see https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
            print "------------------- start publish initialpose 170 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_170, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_170,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])

            print "------------------- published initialpose 170 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 170: {0}".format(err)
    elif 't90250'==t55:
        t55_transrot_from_dum_c2_robot_pose_250 = "dum_%s_trans_rot_to_%s_robot_pose_250"%(c2,t55)
        tfBroadcaster.sendTransform(
            # pioneer - the tall one ( -0.12, 0, -0.76),                             # before rot, step back --> forward
            ( -0.12-0.18, 0, -0.64),                    # before rot, step back --> forward , tag box centre is 18cm rear of base_link (Pioneer2)
            (0, 0, 1, 0),                               # 250 is on the back, so turn 180 to face forward
            time_now,
            t55_transrot_from_dum_c2_robot_pose_250,
            t55_transrot_from_dum_c2_post90y180zneg90z)
#        tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_250, rospy.Time(), rospy.Duration(0))      # from the map origin, to the robot
        try:
            print "------------------- start publish initialpose 250 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_250, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_250,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose 250 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 250: {0}".format(err)
    elif 't90290'==t55:
        t55_transrot_from_dum_c2_robot_pose_290 = "dum_%s_trans_rot_to_%s_robot_pose_290"%(c2,t55)
        tfBroadcaster.sendTransform(
            ( -0.10, 0-(-0.18), -0.64),                 # before left rot, step back --> left , tag box centre is 18cm rear of base_link (Pioneer2)
            (0, 0, 0.707106781, 0.707106781),                     # 290 is on the right side, so turn left to face forward
            time_now,
            t55_transrot_from_dum_c2_robot_pose_290,
            t55_transrot_from_dum_c2_post90y180zneg90z)
#        tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_290, rospy.Time(), rospy.Duration(0))      # from the map origin, to the robot
        try:
            print "------------------- start publish initialpose 290 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_290, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_290,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose 290 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 290: {0}".format(err)
    elif 't90330'==t55:
        t55_transrot_from_dum_c2_robot_pose_330 = "dum_%s_trans_rot_to_%s_robot_pose_330"%(c2,t55)
        tfBroadcaster.sendTransform(                     # robot pose, as estimated from the inverse of the base_link-to-tag-330 transform
            ( -0.10, 0-0.18, -0.64),                              # before right rot, step back --> right , tag box centre is 18cm rear of base_link (Pioneer2)
            (0, 0, -0.707106781, 0.707106781),                     # 330 is on the left side, so turn right to face forward
            time_now,
            t55_transrot_from_dum_c2_robot_pose_330,
            t55_transrot_from_dum_c2_post90y180zneg90z)
        try:
            print "------------------- start publish initialpose 330 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_330, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_330,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose 330 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 330: {0}".format(err)
    elif 't91650'==t55:
        t55_transrot_from_dum_c2_robot_pose_1650 = "dum_%s_trans_rot_to_%s_robot_pose_1650"%(c2,t55)
        tfBroadcaster.sendTransform(
            ( 0.1, 0.0, 0.0),
            (0, 0, 0, 1),                     
            time_now,
            t55_transrot_from_dum_c2_robot_pose_1650,
            t55_transrot_from_dum_c2_post90y180zneg90z)
        try:
            print "------------------- start publish initialpose _dummy_ 1650 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_1650, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_1650,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose _dummy_ 1650 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 1650: {0}".format(err)
            
    print '------------ ???? ---------------------------------- ??? 90557 for robot tag ??? --------------------------------'
    print ' if \'90557\' == t55 :  t55 == %s'%(t55)        
            
    if False and 't90557' == t55: # front, lower
        print '---------------------------------------------- 90557 for robot tag --------------------------------'
        t55_transrot_from_dum_c2_robot_pose_57 = "dum_%s_trans_rot_to_%s_robot_pose_57"%(c2,t55)
        tfBroadcaster.sendTransform(
            ( 0, 0.0, 0.42),                # tag centre 42cm above robot base, but is inverted, so going up here is going down after the rotation
            ( 0, 1, 0, 0),                  # inverted 
            time_now,
            t55_transrot_from_dum_c2_robot_pose_57,
            t55_transrot_from_dum_c2_post90y180zneg90z)
        try:
            print "------------------- start publish initialpose _dummy_ 57 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_57, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_57,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose _dummy_ 57 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 57: {0}".format(err)
    if False and 't90457' == t55: # front, upper
        t55_transrot_from_dum_c2_robot_pose_57 = "dum_%s_trans_rot_to_%s_robot_pose_57"%(c2,t55)
        tfBroadcaster.sendTransform(
            ( 0, 0.0, -0.79),                # tag centre 79cm above robot base
            (0, 0, 0, 1),                   # 
            time_now,
            t55_transrot_from_dum_c2_robot_pose_57,
            t55_transrot_from_dum_c2_post90y180zneg90z)
        try:
            print "------------------- start publish initialpose _dummy_ 57 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_57, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_57,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose _dummy_ 57 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 57: {0}".format(err)
            
    if False and 't90357' == t55: # left, lower
        t55_transrot_from_dum_c2_robot_pose_57 = "dum_%s_trans_rot_to_%s_robot_pose_57"%(c2,t55)
        tfBroadcaster.sendTransform(
            ( 0, 0.0, -0.42),                # tag centre 42cm above robot base
            ( 0,   0, -0.866043, 0.499969),  # rotate right to go from left side to centreline 
            time_now,
            t55_transrot_from_dum_c2_robot_pose_57,
            t55_transrot_from_dum_c2_post90y180zneg90z)
        try:
            print "------------------- start publish initialpose _dummy_ 57 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_57, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_57,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose _dummy_ 57 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 57: {0}".format(err)
    if False and 't90257' == t55: # left, upper
        t55_transrot_from_dum_c2_robot_pose_57 = "dum_%s_trans_rot_to_%s_robot_pose_57"%(c2,t55)
        tfBroadcaster.sendTransform(
            ( 0, 0.0, -0.79),                # tag centre 78cm above robot base
            (0, 0, 0.866043, 0.499969),      # rotate right to go from left side to centreline 
            time_now,
            t55_transrot_from_dum_c2_robot_pose_57,
            t55_transrot_from_dum_c2_post90y180zneg90z)
        try:
            print "------------------- start publish initialpose _dummy_ 57 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_57, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_57,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose _dummy_ 57 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 57: {0}".format(err)
            
    if False and 't90057' == t55: # right, upper
        t55_transrot_from_dum_c2_robot_pose_57 = "dum_%s_trans_rot_to_%s_robot_pose_57"%(c2,t55)
        tfBroadcaster.sendTransform(
            ( 0, 0.0, -0.79),                # tag centre 79cm above robot base
            ( 0,   0, 0.866043, 0.499969),     # rotate left to go from right side to centreline 
            time_now,
            t55_transrot_from_dum_c2_robot_pose_57,
            t55_transrot_from_dum_c2_post90y180zneg90z)
        try:
            print "------------------- start publish initialpose _dummy_ 57 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_57, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_57,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose _dummy_ 57 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 57: {0}".format(err)
    if False and 't90157' == t55: # right, lower
        t55_transrot_from_dum_c2_robot_pose_57 = "dum_%s_trans_rot_to_%s_robot_pose_57"%(c2,t55)
        tfBroadcaster.sendTransform(
            ( 0, 0.0, -0.42),                # tag centre 42cm above robot base
            ( 0,   0,  0.866043, 0.499969),  # rotate left to go from right side to centreline
            time_now,
            t55_transrot_from_dum_c2_robot_pose_57,
            t55_transrot_from_dum_c2_post90y180zneg90z)
        try:
            print "------------------- start publish initialpose _dummy_ 57 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_57, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_57,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose _dummy_ 57 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 57: {0}".format(err)
            
            
    ### _target_ pose publisher ##################################################################################        
    #elif 't90210'==t55:                                  # tag 210 is the target tag : if it moves more than 20cm from last posn, publish an updated target
    if t55 in ",".join ( ['t50957','t90210','t9099999999210'] ) :
        try:
            print "------------------- start check 210 as target ----------------------"
            tfListener.waitForTransform('map',              t55_transrot_from_dum_c2_post90y180zneg90z,  rospy.Time(),  rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_post90y180zneg90z,  rospy.Time(0)  )
            global retarget_requested
            if retarget_requested or abs(tag_210_target_pose.position.x - pos_[0]) > 0.2  or  abs(tag_210_target_pose.position.y - pos_[1]) > 0.2 :
                retarget_requested = False                # reset the flag
                tag_210_target_pose.position.x = pos_[0]
                tag_210_target_pose.position.y = pos_[1]
                publish_pose_xyz_xyzw(tag_210_target_publisher,time_now,  'map', pos_[0], pos_[1], 0.0, 0.0, 0.0, quat_[2], quat_[3])  # NOTE: z is zero for ground robots, and it likes zero roll and pitch  :  move_base.cpp "ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.")"
                targetPoseHistory.append([pos_[0], pos_[1]])
                print "------------------- 210 re-published as target ----------------------"
            else :
                print "------------------- 210 not changed enough to re-publish as target ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 210: {0}".format(err)
            
            
    ### load fixed tags from file ##########################################################################################################
    # ...
    # ...
    # ...
    # ...
    
            
    ### load robot model tags from file ##########################################################################################################
    robot_tag_poses       = [ ('t90999999999157', {'x':0.16,'y':0.0,'z':0.2, 'qx':0, 'qy':0, 'qz':0, 'qw':1}) , ('t909999999999257', {'x':0.05,'y':0.0,'z':0.2, 'qx':0, 'qy':0, 'qz':1, 'qw':0}) ] # 157 faces forward, 257 faces backward # developing - see https://stackoverflow.com/questions/16021571/iterating-quickly-through-list-of-tuples
    robot_tag_poses_dict  = dict(robot_tag_poses)
    print 'load robot_tag_poses from file for t55=%s'%(t55)
    # robot_tag_tuple       = robot_tag_poses_dict.get(t55)
    # print robot_tag_tuple
    # robot_tag_as_dict     = robot_tag_tuple[1]
    if t55 in robot_tag_poses_dict:
        robot_tag_as_dict       = robot_tag_poses_dict.get(t55)
        print robot_tag_as_dict
        
        t55_transrot_from_dum_c2_robot_pose_fixed_robot_tag = "dum_%s_trans_rot_to_%s_robot_pose_57"%(c2,t55)
        tfBroadcaster.sendTransform(
            ( robot_tag_as_dict['x'], robot_tag_as_dict['y'], robot_tag_as_dict['z'] ),
            ( robot_tag_as_dict['qx'], robot_tag_as_dict['qy'], robot_tag_as_dict['qz'], robot_tag_as_dict['qw']),                     
            time_now,
            t55_transrot_from_dum_c2_robot_pose_fixed_robot_tag,
            t55_transrot_from_dum_c2_post90y180zneg90z)
        try:
            print "------------------- start publish initialpose _dummy_ fixed_robot_tag ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_fixed_robot_tag, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_fixed_robot_tag,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose _dummy_ fixed_robot_tag ----------------------"
        except tf.Exception as err:
            print "some tf exception happened fixed_robot_tag: {0}".format(err)
            
            
    ### load fixed camera poses from file ##########################################################################################################
# 612
# point - from map - first ARCAA sofa, facing down Mobile Lab toward S1130 - has angle down open edge of Mobile Lab, and to separator between Jason's office (S1110) Frederik's office (S1111) and: 
#   x: 1.1440243721
#   y: 6.53061819077
#   z: 0.000180721282959

#    camera_poses = [('c',{'x':,'y':,'z':,'qx':,'qy':,'qz':,'qw': })]
#    camera_poses       = [] # ('c110',{'x':1.5,'y':-7.0,'z':1.1}) ] # , ('c111',1.5,-7.5,1.15) , ('c112',1.55,-8.0,1.1) ] # developing - see https://stackoverflow.com/questions/16021571/iterating-quickly-through-list-of-tuples
    camera_poses = [('c999393939',{'x':1.1440243721,'y':6.53061819077,'z':0.000180721282959,'qx':0,'qy':0,'qz':0,'qw':1 })]
    camera_poses_dict  = dict(camera_poses)
    if c2 in camera_poses_dict:
        camera_as_dict             = camera_poses_dict.get(c2)
        camera_frame_label = "dum_%s"%(c2)
        
        t55_transrot_from_dum_c2_robot_pose_fixed_camera = "dum_%s_trans_rot_to_%s_robot_pose_57"%(c2,t55)
        tfBroadcaster.sendTransform(
            ( camera_as_dict['x'], camera_as_dict['y'], camera_as_dict['z'] ),
            ( camera_as_dict['qx'], camera_as_dict['qy'], camera_as_dict['qz'], camera_as_dict['qw']), 
            time_now,            
            camera_frame_label,     # to camera
            '/map')                  # from map
        try:
            print "------------------- start publish initialpose _dummy_ fixed_camera ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_fixed_camera, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_fixed_camera,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            robotPoseHistory.append([pos_[0], pos_[1]])
            print "------------------- published initialpose _dummy_ fixed_camera ----------------------"
        except tf.Exception as err:
            print "some tf exception happened fixed_camera: {0}".format(err)

    #############################################################################################################
    ori = req.visualFeature.pose.pose.orientation
    pos = req.visualFeature.pose.pose.position

# removed tf transforms code is at the bottom of this file

    if 'c3'==cN:
        try:
            # LATER; 'c1_from_fixed_t32'
            listener.waitForTransform('map', 't50_from_c1_180x', rospy.Time(), rospy.Duration(4.0))      # from the map origin, to the camera, through a fixed point
        except:
            print 'ERROR ------ : no transform from %s to %s '%('map', 't50_from_c1_180x')

    response = DetectedFeatureResponse()
    response.acknowledgement="bob"
    return response


    ###  end detect_feature_callback(req)




def axisMarker(marker_id_,parent_frame_id_):
    marker = Marker()
    marker.ns = parent_frame_id_
    marker.id = marker_id_
    marker.header.frame_id = parent_frame_id_
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 0.8
    marker.color.g = 0.8
    marker.color.b = 0.8
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker_publisher.publish(marker)
    print 'axisMarker done'
    #    print marker

##########################################################################################################

default_resolution = (640.0, 480.0)

class VisionSource:
        
    def __init__(self, *args, **kwargs):
        self.hasFov  = False
        self.hasPose = False
        self.vision_source_id = args[0]
        self.base_url         = args[1]
        self.publisher_to_phone  = args[2]
        if len(args) > 3:
            self.hasFov=True
            self.min_x = args[3]
            self.min_y = args[4]
            self.max_x = args[5]
            self.max_y = args[6]
            if len(args) > 7:
                self.hasPose = True
                self.pose = args[7]
            if len(args) > 9:
                self.resolution_x = args[8]
                self.resolution_y = args[9]                
                self.relative_resolution = ((self.resolution_x**2 + self.resolution_y**2)**0.5) / ((default_resolution[0]**2 + default_resolution[1]**2)**0.5)
            else:  
                global default_resolution  
                self.resolution_x = default_resolution[0]
                self.resolution_y = default_resolution[1]
                self.relative_resolution = 1.0 
            
##########################################################################################################


allocatedVisionSources = []

class VisionSourceAllocation:
    def __init__(self,job_, vision_source_id_):
        self.job = job_
        self.vision_source_id = vision_source_id_
        
    def __str__(self):
        return "VisionSourceAllocation[job:%s, vision_source_id:%s]"%(self.job, self.vision_source_id)   


def allocateVisionSourcesWithFieldOfView(job_, x, y):
    isAllocated = False
    for visionSource in vision_sources:
        if visionSource.hasFov:
            if visionSource.min_x <= x and visionSource.min_y <= y and visionSource.max_x >= x and visionSource.max_y >= y :
                visionSourceAllocation = VisionSourceAllocation(job_, visionSource.vision_source_id)
                allocatedVisionSources.append(visionSourceAllocation)
                print "allocateVisionSourcesWithFieldOfView(job_,x:%d,y:%d): allocated: %s"%(x,y,visionSourceAllocation)
                isAllocated = True
    if not isAllocated:
        print "allocateVisionSourcesWithFieldOfView(job_,x:%d,y:%d): NOT allocated"%(x,y)           



def allocateVisionSourcesWithFieldOfViewAndPose(job_, x, y):
    isAllocated = False
    closest_distance_to_location = 10**6
    visionSource_to_allocate = -1
    can_allocate_a_vision_source = False
    for visionSource in vision_sources:
        if visionSource.hasFov:
            if visionSource.min_x <= x and visionSource.min_y <= y and visionSource.max_x >= x and visionSource.max_y >= y :
                if visionSource.hasPose:
                    distance = (visionSource.pose.position.x - x)**2 + (visionSource.pose.position.y - y)**2
                    distance = distance / visionSource.relative_resolution
                    if distance < closest_distance_to_location:                    
                        closest_distance_to_location = distance
                        visionSource_to_allocate = visionSource    
                        can_allocate_a_vision_source = True                        
    if can_allocate_a_vision_source:
        visionSourceAllocation = VisionSourceAllocation(job_, visionSource_to_allocate.vision_source_id)
        allocatedVisionSources.append(visionSourceAllocation)
        print "allocateVisionSourcesWithFieldOfViewAndPose(job_,x:%d,y:%d): allocated: %s"%(x,y,visionSourceAllocation)
    else:
        print "allocateVisionSourcesWithFieldOfViewAndPose(job_,x:%d,y:%d): NOT allocated"%(x,y)           



def deallocateVisionSource(vision_source_id_):
    for visionSourceAllocation in allocatedVisionSources:
        if visionSourceAllocation.vision_source_id == vision_source_id_:
            allocatedVisionSources.remove(visionSourceAllocation)

def deallocateJob(job_id_):     
    for visionSourceAllocation in allocatedVisionSources:
        if visionSourceAllocation.job.job_id == job_id_:
            allocatedVisionSources.remove(visionSourceAllocation)
            
def deallocateVosClient(vos_client_id_):     
    for visionSourceAllocation in allocatedVisionSources:
        if visionSourceAllocation.job.vosClient.vos_client_id == vos_client_id_:
            allocatedVisionSources.remove(visionSourceAllocation)
            
##########################################################################################################

vos_clients = []
vos_client_id_seq = 1

class VosClient:
    vos_client_id = -1
    url=""
    
    def __init__(self, url_, vos_client_id_): 
       self.url=url_
       self.vos_client_id = vos_client_id_
       
    def __str__(self):
        return "VosClient[id:%s, url:%s]"%(self.vos_client_id, self.url)   
       
def next_vos_client_id_seq():
    global vos_client_id_seq 
    vos_client_id_seq = vos_client_id_seq + 1
    return vos_client_id_seq

def new_vos_client(url_):
    vosClient = VosClient(url_, next_vos_client_id_seq())
    return vosClient
    
def register_vos_client(vosClient_):
    vos_clients.append(vosClient_)
    
def deregister_vos_client(vosClient_):
    vos_clients.remove(vosClient_)

##########################################################################################################

jobs = []
job_id_seq = 1

class Job:
    job_id = -1
    vosClient = -1
    
    def __init__(self, job_id_, vosClient_):
        self.job_id    = job_id_
        self.vosClient = vosClient_        

    def __str__(self):
        return "Job[job_id:%s, vosClient:%s]"%(self.job_id, self.vosClient)   

def next_job_id_seq():
    global job_id_seq
    job_id_seq = job_id_seq+1
    return job_id_seq


def new_job(vosClient_):
    job = Job(next_job_id_seq(),vosClient_)
    return job
    
def start_job(job_):
    jobs.append(job_)
    print 'Added job %s: now have %d jobs.'%(job_,len(jobs))
    
def cease_job(job_id_):
    print 'cease_job(%s)'%(job_id_)
    for job_ in jobs:
        if job_.job_id == job_id_:
           deallocateJob(job_id_)   
           print 'Deallocated job %s: now have %d jobs.'%(job_,len(jobs))
           jobs.remove(job_)              
           print 'Removed job %s: now have %d jobs.'%(job_,len(jobs))
           
##########################################################################################################


def unique(sequence):
    """Generate unique items from sequence in the order of first occurrence."""
    seen = set()
    for value in sequence:
        if value in seen:
            continue
        seen.add(value)
        yield value


##########################################################################################################


def phone_whereis_setupPublisherForPhoneCamera(phone_camera_id_string_):  # problem with Android phones being able to serve ROS services: set up a pulisher from this 'Server', and set up a subscriber on the phone
    topic_name = "/phone_whereis/" + phone_camera_id_string_
    new_publisher = rospy.Publisher(topic_name, WhereIsAsPub, queue_size=2, latch=True)
    print "set up publisher for " + topic_name + " : problem with Android phones being able to serve ROS services: set up a pulisher from this 'Server', and set up a subscriber on the phone"
    return new_publisher


def register_vision_source_callback(req_registerVisionSource):
    print "---------------------------------------------------------------------"
    print "start register_vision_source_callback(req_registerVisionSource)"
    print "start register_vision_source_callback("+req_registerVisionSource.vision_source_base_url+")"
    print "start register_vision_source_callback("+str(req_registerVisionSource.vision_source_base_url)+")"
    print "registering vision source '%s'"%(req_registerVisionSource.vision_source_id)
    print "---------------------------------------------------------------------"
    print req_registerVisionSource.vision_source_base_url
    publisher_to_phone = phone_whereis_setupPublisherForPhoneCamera(req_registerVisionSource.vision_source_base_url)
    new_vision_source = VisionSource(req_registerVisionSource.vision_source_id, req_registerVisionSource.vision_source_base_url, publisher_to_phone)
    vision_sources.append(new_vision_source)
    response = RegisterVisionSourceResponse()
    if  new_vision_source.vision_source_id in ['c10', 'c15']:
        #   dummy frame for the camera: 1m up away from origin, same orientation - facing toward the lobby
        #   fixed camera pose
        response.acknowledgement = 'registered vos_id=%s x=1.9 y=-5.75 z=0.57 qx=0 qy=0 qz=0.70710678118654757 qw=0.70710678118654757'%(new_vision_source.vision_source_id)
    elif new_vision_source.vision_source_id in ['c11', 'c12']:
        #   dummy frame for the camera: 1m up away from origin, facing right - toward S1165
        #   fixed camera pose
        response.acknowledgement = 'registered vos_id=%s x=1.9 y=-5.75 z=0.57 qx=0 qy=0 qz=0 qw=1'%(new_vision_source.vision_source_id)
    elif new_vision_source.vision_source_id in ['c20', 'c21','c606']:
        #   dummy frame for the camera: 1m up away from origin, facing 45 degrees right - toward corner of Michael's office
        #   fixed camera pose
        response.acknowledgement = 'registered vos_id=%s x=1.9 y=-5.75 z=0.57 qx=0 qy=0 qz=0.383 qw=0.92374834235304581'%(new_vision_source.vision_source_id)
    else:
        response.acknowledgement = 'registered'
    rospy.loginfo('register_vision_source_callback: registered vision source %s with base URL %s',new_vision_source.vision_source_id, new_vision_source.base_url)
    print vision_sources
    return response


def management_input(req_management):
  if 're-target' in req_management.data:
    print "management_input(%s): re-target"%(req_management)
    global retarget_requested
    retarget_requested = True   # set the flag



def display_status_callback(string_msg_):
    print "-------------------------------------"
    print "current status: "
    print "  Registered vision sources:"
    for vision_source_ in vision_sources:
        print '    vision source: id = "%s", base URL = "%s"'%(vision_source_.vision_source_id, vision_source_.base_url)
    print "-------------------------------------"


def fixed_tag_pose_callback(string_msg_):
    msg = string_msg_.data
    parts = msg.split("|")
    tag_number = parts[0]
#    matches = (fixed_feature for fixed_feature in fixed_features if fixed_feature.algorithm == 'AprilTags_Kaess_36h11'  and fixed_feature.id == tag_number )
    for fixed_feature in fixed_features:
        if fixed_feature.algorithm == 'AprilTags_Kaess_36h11'  and fixed_feature.id == tag_number:
            print 'fixed_tag_pose_callback: updating tag %s'%(tag_number)
            fixed_feature.pose.position.x=float(parts[1])
            fixed_feature.pose.position.y=float(parts[2])
            fixed_feature.pose.position.z=float(parts[3])
            fixed_feature.pose.orientation.x=float(parts[4])
            fixed_feature.pose.orientation.y=float(parts[5])
            fixed_feature.pose.orientation.z=float(parts[6])
            fixed_feature.pose.orientation.w=float(parts[7])
    time_now = rospy.Time.now()
    tfBroadcaster_ = get_tfBroadcaster()
    publish_fixed_tags_tf(tfBroadcaster_, time_now)




fixed_tags_filename = "/tmp/fixed_tags"

def write_fixed_tags_to_file(fixed_tags_filename_):
    print "write_fixed_tags_to_file(): start"
    print "write_fixed_tags_to_file(): writing to file %s" % ( fixed_tags_filename )
    f = open(fixed_tags_filename_, 'w')
    for fixed_feature in fixed_features:
        json_obj = jsonpickle.encode(fixed_feature)
        f.write(json_obj)
        f.flush()
#    json_obj = jsonpickle.encode(fixed_features)
#    f.write(json_obj)
#    f.flush()
    f.close()
    print "write_fixed_tags_to_file(): end"

def load_fixed_tags_from_file(fixed_tags_filename_):
    print "load_fixed_tags_from_file(): start"
    print "load_fixed_tags_from_file(): reading fixed_tags from file %s" % ( fixed_tags_filename )
    f = open(fixed_tags_filename_, 'r')
    single_line = f.read()
    all_lines = single_line.split("}{")

    fixed_features = []

    i_ = 0
    for a_line in all_lines:
        i_ = i_ + 1
        json_str = a_line+"}"
        if i_ > 1:
            json_str = "{" + a_line
        print json_str
        obj = jsonpickle.decode(json_str)
        print "loading fixed tag %s = %s" % (type(obj),obj)
        fixed_features.append(obj)
#    json_str = f.read()
#    obj = jsonpickle.decode(json_str)
#    print obj

    print "load_fixed_tags_from_file(): end"


vos_base_frame_name = "vos_base_frame"
map_to_vos_base_frame_tf = Pose()
map_to_vos_base_frame_tf.position.x=0
map_to_vos_base_frame_tf.position.y=0
map_to_vos_base_frame_tf.position.z=0
map_to_vos_base_frame_tf.orientation.x=0
map_to_vos_base_frame_tf.orientation.y=0
map_to_vos_base_frame_tf.orientation.z=0
map_to_vos_base_frame_tf.orientation.w=1

deque_to_base_frame_publisher_thread = deque()



def publish_map_to_vos_base_frame_tf(tfBroadcaster,time_now):
    pos = map_to_vos_base_frame_tf.position
    ori = map_to_vos_base_frame_tf.orientation
    tfBroadcaster.sendTransform(
        (pos.x, pos.y, pos.z ),
        (ori.x, ori.y, ori.z, ori.w),
        time_now,
        vos_base_frame_name,        # to   'vos_base_frame'
        'map')                      # from 'map'


def publish_map_to_vos_base_frame_tf_loop_forever(tfBroadcaster,time_now,deque_to_base_frame_publisher_thread):
    print "publish_map_to_vos_base_frame_tf_loop_forever: start"
    while 2 > 1:
        print "publish_map_to_vos_base_frame_tf_loop_forever: iteration start"
        try:
            something = deque_to_base_frame_publisher_thread.popleft()
            print "publish_map_to_vos_base_frame_tf_loop_forever: something on deque"
        except IndexError as e:
            print "publish_map_to_vos_base_frame_tf_loop_forever: nothing on deque: running publish_map_to_vos_base_frame_tf(tfBroadcaster,time_now)"
            publish_map_to_vos_base_frame_tf(tfBroadcaster,time_now)
        time.sleep(0.1)


def publish_map_to_vos_base_frame_tf_threaded(tfBroadcaster,time_now):
    map_to_vos_base_frame_tf_thread = threading.Thread(target=publish_map_to_vos_base_frame_tf_loop_forever, args=(tfBroadcaster,time_now,deque_to_base_frame_publisher_thread))
    map_to_vos_base_frame_tf_thread.start()


def publish_map_coordinate_frame_origin_tf(tfBroadcaster,time_now):
    pos = map_to_vos_base_frame_tf.position
    ori = map_to_vos_base_frame_tf.orientation
    tfBroadcaster.sendTransform(
        (pos.x, pos.y, pos.z ),
        (ori.x, ori.y, ori.z, ori.w),
        time_now,
        vos_base_frame_name,        # to   'vos_base_frame'
        'map')                      # from 'map'

def publish_fixed_tags_tf(tfBroadcaster,time_now):
    for fixed_feature in fixed_features:
        tfBroadcaster.sendTransform(
            (fixed_feature.pose.position.x, fixed_feature.pose.position.y, fixed_feature.pose.position.z ),
            (fixed_feature.pose.orientation.x, fixed_feature.pose.orientation.y, fixed_feature.pose.orientation.z, fixed_feature.pose.orientation.w),
            time_now,
            '%s%s' % (algorithm_abreviations[fixed_feature.algorithm], fixed_feature.id),           # /map -> t55
            vos_base_frame_name)
        tfBroadcaster.sendTransform(
            (fixed_feature.pose.position.x, fixed_feature.pose.position.y, fixed_feature.pose.position.z ),
            (fixed_feature.pose.orientation.x, fixed_feature.pose.orientation.y, fixed_feature.pose.orientation.z, fixed_feature.pose.orientation.w),
            time_now,
            'fixed_%s%s' % (algorithm_abreviations[fixed_feature.algorithm], fixed_feature.id),     # /map -> fixed_t55
            vos_base_frame_name)

# id_ is the frame that this pose is relative to: e.g. if this pose is relative to the map, use "map"
def publish_pose_xyz_rpy(pose_publisher,time_now, id_, x, y, z, r, p, yaw):
    # next, we'll publish the pose message over ROS
    print "publish_pose_xyz_rpy(pose_publisher,time_now, id_=%s"%(id_)
    pose_quat = tf.transformations.quaternion_from_euler(r,p,yaw)
    pose = PoseStamped()
    pose.header.stamp = time_now
    pose.header.frame_id = id_
    pose.pose.pose = Pose(Point(x, y, z), Quaternion(*pose_quat))  # NOTE: '*' means turn into a list of arguments or some such
    pose_publisher.publish(pose)


# id_ is the frame that this pose is relative to: e.g. if this pose is relative to the map, use "map"
def publish_pose_xyz_xyzw(pose_publisher,time_now, id_, x, y, z, qx, qy, qz, qw):
    # next, we'll publish the pose message over ROS
    print "publish_pose_xyz_xyzw(pose_publisher,time_now, id_=%s"%(id_)
    pose = PoseStamped()
    pose.header.stamp = time_now
    pose.header.frame_id = id_
    pose.pose = Pose(Point(x, y, z), Quaternion(qx, qy, qz, qw))
    pose_publisher.publish(pose)

# id_ is the frame that this pose is relative to: e.g. if this pose is relative to the map, use "map"
def publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, id_, x, y, z, qx, qy, qz, qw, covariance_):
    # next, we'll publish the pose message over ROS
    print "publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, id_=%s"%(id_)
    poseWCS = PoseWithCovarianceStamped()
    poseWCS.header.stamp = time_now
    poseWCS.header.frame_id = id_
    poseWCS.pose.pose = Pose(Point(x, y, z), Quaternion(qx, qy, qz, qw))
    poseWCS.pose.covariance = covariance_
    initialpose_poseWCS_publisher.publish(poseWCS)

    poseWC = PoseWithCovariance()
    poseWC.pose = Pose(Point(x, y, z), Quaternion(qx, qy, qz, qw))
    poseWC.covariance = covariance_
    twistWC = TwistWithCovariance()
    twistWC.twist = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0))
    twistWC.covariance = covariance_
    odom = Odometry()
    odom.header.stamp = time_now
    odom.header.frame_id = id_
    odom.pose = poseWC
    odom.twist = twistWC
    fakelocalisation_poseWCS_publisher.publish(odom)



def detect_feature_server1():
    print "---------- detect_feature_server1(): start ----------"
    rospy.init_node('detect_feature_server')
    print "---------- detect_feature_server1(): end ----------"

def detect_feature_server2():
    print "---------- detect_feature_server2(): start ----------"

#######    write_fixed_tags_to_file("/tmp/fixed_tags")
#######    load_fixed_tags_from_file("/tmp/fixed_tags")
#######    write_fixed_tags_to_file("/tmp/fixed_tags2")
#######    load_fixed_tags_from_file("/tmp/fixed_tags2")

    global localise_by_visual_descriptor_server
    localise_by_visual_descriptor_server = rospy.Service('/vos_server/localise_by_visual_descriptor', localise_by_visual_descriptor, localise_by_visual_descriptor_callback)



    detect_feature_server = rospy.Service('/androidvosopencvros/detected_feature', DetectedFeature, detect_feature_callback)
    print "Ready to receive detected features."
    rospy.loginfo("Ready to receive detected features.")
    localise_from_a_feature_server = rospy.Service('/androidvosopencvros/localise_from_a_feature', LocaliseFromAFeature, localise_from_a_feature_callback)
    print "Ready to receive localise from individual features."
    rospy.loginfo("Ready to receive localise from individual features.")
    global marker_publisher
    marker_publisher = rospy.Publisher('axis_markers', Marker, queue_size = 100)
    print "Ready to publish markers"
    rospy.loginfo("Ready to publish markers")
    print marker_publisher
    vision_source_registration_server = rospy.Service('/androidvosopencvros/register_vision_source',RegisterVisionSource,register_vision_source_callback)
    print "Ready to register vision sources"
    rospy.loginfo("Ready to register vision sources")

    global management_subscriber
    global retarget_requested
    management_subscriber = rospy.Subscriber('/management',std_msgs.msg.String, management_input)
    retarget_requested = False
    print "Ready to accept management input"

    display_status_subscriber = rospy.Subscriber('/androidvosopencvros/display_status',std_msgs.msg.String, display_status_callback)
    print "Ready to display current status to console"
    rospy.loginfo("Ready to display current status to console")
    fixed_tag_pose_subscriber = rospy.Subscriber('/androidvosopencvros/fixed_tag_pose',std_msgs.msg.String, fixed_tag_pose_callback)
    print "Ready to update fixed tags"
    rospy.loginfo("Ready to update fixed tags")

    global pose_publisher
    pose_publisher = rospy.Publisher("poses_from_requests", PoseStamped, queue_size=1)
    print "Ready to publish poses"
    rospy.loginfo("Ready to publish poses")

    global tag_210_target_publisher
    tag_210_target_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    #pose_publisher = rospy.Publisher("/move_base/goal", move_base_msgs.MoveBaseActionGoal, queue_size=1)
    print "Ready to publish tag 210 as target poses"
    rospy.loginfo("Ready to publish tag 210 as target poses")

    global initialpose_poseWCS_publisher
    initialpose_poseWCS_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=50, latch=True)  # latch to make sure that AMCL has an intitial pose to use
    print "Ready to publish poses with covariance"
    rospy.loginfo("Ready to publish poses with covariance")

    global fakelocalisation_poseWCS_publisher  # fake_localisation / fake_localization
    fakelocalisation_poseWCS_publisher = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=50, latch=True)  # latch to make sure that AMCL has an intitial pose to use
    print "Ready to publish /base_pose_ground_truth for fake_localization"
    rospy.loginfo( "Ready to publish /base_pose_ground_truth for fake_localization" )


    tfBroadcaster = tf.TransformBroadcaster()
    set_tfBroadcaster(tfBroadcaster)
    time_now = rospy.Time.now()
    global tfListener
    tfListener = tf.TransformListener()

#    publish_map_coordinate_frame_origin_tf(tfBroadcaster,time_now)
#    publish_map_to_vos_base_frame_tf_threaded(tfBroadcaster,time_now)
    publish_map_to_vos_base_frame_tf(tfBroadcaster,time_now)
    print "Done: publish_map_to_vos_base_frame_tf"
    publish_fixed_tags_tf(tfBroadcaster, time_now)
    print "Done: publish_fixed_tags_tf"


    # VosServer internal interfaces to other nodes of VosServer
#    global list_vision_sources_publisher
#    list_vision_sources_publisher = rospy.Publisher();

    # monitoring
    global detection_true_monitoring_publisher
    detection_true_monitoring_publisher = rospy.Publisher("/monitoring/detections_true", std_msgs.msg.String, queue_size=2, latch=True)
    print "Ready to publish /monitoring/detections_true for monitoring true detections"

    global detection_false_monitoring_publisher
    detection_false_monitoring_publisher = rospy.Publisher("/monitoring/detections_false", std_msgs.msg.String, queue_size=2, latch=True)
    print "Ready to publish /monitoring/detections_false for monitoring false detections"

    print "---------- detect_feature_server2(): before rospy.spin() ----------"


    # rospy.spin()  # spin() simply keeps python from exiting until this node is stopped  -  https://svn.jderobot.org/users/mmoya/tfm/trunk/jderobot_ros/scripts/camera_dumper  -  http://answers.ros.org/question/252545/interrupting-rospyspin-or-writing-a-custom-loop-that-does-the-equivalent/


    print "---------- detect_feature_server2(): after rospy.spin() ----------"


RETURN_URL_OF_ROBOT_AS_SERVICE_NAME = "robot/vos_whereis/return"

def detect_feature_server3():
    print "---------- start detect_feature_server3() ----------"

    connected_ = False
    VOS_SERVER__WHERE_IS__SERVICE_NAME = "whereis"
    C60__WHERE_IS__SERVICE_NAME = "/cam_60/where_is"      # TODO - from the registered cameras as necessary to service robot's requests

    while not rospy.core.is_shutdown():
        #  problem with ROSJava and setting up service server on the phones
#        if not connected_:
#            try:
#                rospy.wait_for_service(C60__WHERE_IS__SERVICE_NAME, 1.0)
#                try:
#                    pose_of_feature_from_base = Pose()
#                    pose_of_feature_from_base = Pose(Point(0.10, 0.18, 0.64), Quaternion(0, 0, 0.707106781, 0.707106781))  # NOTE: '*' means turn into a list of arguments or some such
                    #        ( -0.10, 0-0.18, -0.64),                              # before right rot, step back --> right , tag box centre is 18cm rear of base_link (Pioneer2)
                    #        (0, 0, -0.707106781, 0.707106781),                     # 330 is on the left side, so turn right to face forward
#                    request_id = "9000"
#                    whereis = rospy.ServiceProxy(C60__WHERE_IS__SERVICE_NAME, localise_by_visual_descriptor)
#                    resp1 = whereis("BoofCV Binary", "width=14.0|id=330", request_id,
#                        pose_of_feature_from_base, RETURN_URL_OF_ROBOT_AS_SERVICE_NAME, rospy.Time.now(), 0, 0, 0)
#                    connected_ = True
#                    return resp1.result
#                except rospy.exceptions.ROSException, e:
#                    print "FAILED:  rospy.wait_for_service(VOS_SERVER__WHERE_IS__SERVICE_NAME, 1.0) failed: %s"%e
#                except rospy.ServiceException, e:
#                    print "FAILED:  Service call failed: %s"%e
#            except rospy.exceptions.ROSException, e:
#                print "FAILED:  rospy.wait_for_service(VOS_SERVER__WHERE_IS__SERVICE_NAME, 1.0) failed: %s"%e
#            except rospy.ServiceException, e:
#                print "FAILED:  rospy.wait_for_service(VOS_SERVER__WHERE_IS__SERVICE_NAME, 1.0) failed: %s"%e

        rospy.rostime.wallsleep(0.5)

    print "---------- end detect_feature_server3() ----------"


def keep_loop_open():       # http://answers.ros.org/question/252545/interrupting-rospyspin-or-writing-a-custom-loop-that-does-the-equivalent/
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)


if __name__ == "__main__":
    global job_id_seq
    global vos_client_id_seq
    vosClient_1 = new_vos_client("URL 1")
    vosClient_2 = new_vos_client("URL 2")
    
    start_job(new_job(vosClient_1))
    start_job(new_job(vosClient_2))
    start_job(new_job(vosClient_1))
    start_job(new_job(vosClient_2))
    start_job(new_job(vosClient_1))
    job_6 = new_job(vosClient_2)
    start_job(job_6)
    cease_job(3)
    
    #VisionSource(vision_source_id_, base_url_, publisher_to_phone_, min_x, min_y, max_x, max_y)
    visionSource_607_pose = Pose()
    visionSource_607_pose.position.x = 1
    visionSource_607_pose.position.y = 1
    visionSource_607_pose.position.z = 1
    visionSource_607 = VisionSource("cam_607", "/cam_607/url/", "publisher_to_phone_607", 10, 10, 100, 100, visionSource_607_pose)
    visionSource_608_pose = Pose()
    visionSource_608_pose.position.x = 88
    visionSource_608_pose.position.y = 88
    visionSource_608_pose.position.z = 88
    visionSource_608 = VisionSource("cam_608", "/cam_608/url/", "publisher_to_phone_608", 20, 20, 100, 100, visionSource_608_pose)   # default 640x480
    visionSource_609_pose =  Pose()
    visionSource_609_pose.position.x = 99
    visionSource_609_pose.position.y = 99
    visionSource_609 = VisionSource("cam_609", "/cam_609/url/", "publisher_to_phone_609", 20, 20, 200, 200, visionSource_609_pose, 420, 360) # same FoV, same camera pose, worse resolution 
    visionSource_610_pose =  Pose()
    visionSource_610_pose.position.x = 150
    visionSource_610_pose.position.y = 150
    visionSource_610 = VisionSource("cam_610", "/cam_610/url/", "publisher_to_phone_609", 20, 20, 200, 200, visionSource_610_pose, 400, 300) # same FoV, same camera pose, better resolution 
    
    vision_sources.append(visionSource_607)
    vision_sources.append(visionSource_608)
    
    allocateVisionSourcesWithFieldOfView(job_6, 9,10)
    allocateVisionSourcesWithFieldOfView(job_6, 10,9)
    allocateVisionSourcesWithFieldOfView(job_6, 10,10)
    allocateVisionSourcesWithFieldOfView(job_6, 100,100)
    allocateVisionSourcesWithFieldOfView(job_6, 101,100)
    allocateVisionSourcesWithFieldOfView(job_6, 100,101)
    allocateVisionSourcesWithFieldOfView(job_6, 20,20)
    
    vision_sources.append(visionSource_609)
    vision_sources.append(visionSource_610)
    allocateVisionSourcesWithFieldOfViewAndPose(job_6, 20,20)
    allocateVisionSourcesWithFieldOfViewAndPose(job_6, 44,45)
    allocateVisionSourcesWithFieldOfViewAndPose(job_6, 45,44)
    allocateVisionSourcesWithFieldOfViewAndPose(job_6, 45,45)
    allocateVisionSourcesWithFieldOfViewAndPose(job_6, 100,100)
    allocateVisionSourcesWithFieldOfViewAndPose(job_6, 125,125)
    allocateVisionSourcesWithFieldOfViewAndPose(job_6, 126,126)
    allocateVisionSourcesWithFieldOfViewAndPose(job_6, 180,180)


    an_instance = ConfigLoader('/mnt/nixbig/build_workspaces/aa1_vos_android_catkin_ws/src/vos_aa1/src/vos_aa1/config.txt')
    detect_feature_server1()
    thread1 = threading.Thread(target = detect_feature_server2(), args=[])
    thread2 = threading.Thread(target = detect_feature_server3(), args=[])
    thread2.start()
    print "---------- after thread2.start() ----------"
    thread2.join()
    print "---------- after thread2.join() ----------"

    thread1.start()
    print "---------- after thread1.start() ----------"

    thread1.join()
    print "---------- after thread1.join() ----------"

#    thread = Thread(target = threaded_function, args = (10, ))





################################################################################


