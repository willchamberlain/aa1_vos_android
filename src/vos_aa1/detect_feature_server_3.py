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
import threading

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
from vos_aa1.srv import where_is_alg_desc
from vos_aa1.srv import where_is_alg_descRequest
from vos_aa1.srv import where_is_alg_descResponse
from vos_aa1.msg import VisualFeatureInWorld
from vos_aa1.msg import VisualFeatureObservation

algorithm_abreviations = {'AprilTags_Kaess_36h11':'t'}


#
fixed_features = []

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
# tag_32 = VisualFeatureInWorld()
# tag_32.algorithm = 'AprilTags_Kaess_36h11'
# tag_32.id = '32'
# tag_32.pose = Pose()
# tag_32.pose.position.x=0
# tag_32.pose.position.y=0
# tag_32.pose.position.z=1.18
# tag_32.pose.orientation.x=0
# tag_32.pose.orientation.y=0
# tag_32.pose.orientation.z=0
# tag_32.pose.orientation.w=1
# fixed_features.append(tag_32)

# "2|-0.5|-3|1.15|0|0|-0.70710678118654757|0.70710678118654757"
tag_2 = VisualFeatureInWorld()
tag_2.algorithm = 'AprilTags_Kaess_36h11'
tag_2.id = '2'
tag_2.pose = Pose()
tag_2.pose.position.x=-0.5
tag_2.pose.position.y=-3
tag_2.pose.position.z=1.15
tag_2.pose.orientation.x=0
tag_2.pose.orientation.y=0
tag_2.pose.orientation.z=-0.707106781
tag_2.pose.orientation.w=0.707106781
fixed_features.append(tag_2)

# tag_19 = VisualFeatureInWorld()
# tag_19.algorithm = 'AprilTags_Kaess_36h11'
# tag_19.id = '19'
# tag_19.pose = Pose()
# tag_19.pose.position.x=0
# tag_19.pose.position.y=0
# tag_19.pose.position.z=1.18
# tag_19.pose.orientation.x=0
# tag_19.pose.orientation.y=0
# tag_19.pose.orientation.z=0
# tag_19.pose.orientation.w=1
# fixed_features.append(tag_19)

# 0|2.5|-1.5|1.25|0|0|-0.707106781|0.707106781
tag_0 = VisualFeatureInWorld()
tag_0.algorithm = 'AprilTags_Kaess_36h11'
tag_0.id = '0'
tag_0.pose = Pose()
tag_0.pose.position.x=2.5
tag_0.pose.position.y=-1.5
tag_0.pose.position.z=1.25
tag_0.pose.orientation.x=0
tag_0.pose.orientation.y=0
tag_0.pose.orientation.z=-0.707106781
tag_0.pose.orientation.w=0.707106781
fixed_features.append(tag_0)

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

# features_present = (0,2,3,9)
# features_present = (170, 210, 250, 290, 330, 370, 410, 450, 490, 530, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59)
# features_present = (210, 1210, 2210, 3210, 4210, 5210, 6210, 7210, 8210, 9210, 10210,   22210)
features_present = ( 9170 , 9250 , 9290 , 9330 , -9000 , 9555 , 9210  )

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
where_is_server = 1

# Server-internal - interfaces to other nodes of the VosServer
#list_vision_sources_publisher = 1


# monitoring
detection_true_monitoring_publisher = 1
detection_false_monitoring_publisher = 1


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
    print "localise_from_a_feature_callback: "
    cN = req.visualFeature.pose.header.frame_id
    visual_feature_descriptor = req.visualFeature.id

    #  1) set up the tf names to use as variables
    #  2) use those variables to pull the tfs and respond to the camera self-localisation request
    for x in fixed_features:
        print '---- fixed_feature exists with id "',x.id,'"'

    if any( x.id == visual_feature_descriptor or str(x.id) == str(visual_feature_descriptor) for x in fixed_features):
        print '---- localise_from_a_feature_callback: fixed feature "',x,'" matches descriptor "',visual_feature_descriptor,'"'
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


def where_is_callback(request):
  print "where_is_callback: algorithm=%s, descriptor=%s, rate=%d"%(request.algorithm,request.descriptor,request.rate)

  for vision_source in vision_sources:
    whereIsAsPub = WhereIsAsPub()
    whereIsAsPub.algorithm  = request.algorithm
    whereIsAsPub.descriptor = request.descriptor
    whereIsAsPub.rate       = request.rate
    vision_source.publisher_to_phone.publish(whereIsAsPub)
    print "where_is_callback: vision_source_id=%s : algorithm=%s, descriptor=%s, rate=%d"%(vision_source.vision_source_id, request.algorithm,request.descriptor,request.rate)

  response = where_is_alg_descResponse()
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

    cN = req.cameraPose.header.frame_id

    if req.visualFeature.id not in features_present:
        print "detect_feature_callback: camera frame_id [%s] : feature id [%d] : feature is not present - is a false positive - not listing as a detection."%(req.cameraPose.header.frame_id, req.visualFeature.id)        
        detection_false_monitoring_publisher.publish("False detection: camera frame_id [%s] : feature id [%d]")
        response = DetectedFeatureResponse()
        response.acknowledgement="feature not present"
        return response        

    detection_true_monitoring_publisher.publish("True detection: camera frame_id [%s] : feature id [%d]")

    # # TODO - something about this conversion is not right: the translation and quaternion match those found by AprilTags_Kaess and sent by DetectedFeatureClient, but the RPY are different
    # euler = tf.transformations.euler_from_quaternion([req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w])
    # print "  feature x_y_z_w roll=%.4f pitch=%.4f yaw=%.4f"%(euler[0], euler[1], euler[2])

    tfBroadcaster = tf.TransformBroadcaster()
    time_now      = rospy.Time.now()



# working through BoofCV transforms and orientations: BoofCV uses yet another frame for tags
    c2 = req.cameraPose.header.frame_id                          # c2, c11, ... , cN     : DOES include the 'c'
    t = algorithm_abreviations[req.visualFeature.algorithm]     # 't'
    t_id = req.visualFeature.id                                 # 0, 2, 170, 210,       : does NOT include any 't'
    t55 = "%s%s"%(t,t_id)

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

    t55_trans_from_dum_c2__drop = "z_dum_%s_trans_to_%s__drop"%(c2,t55)
    tfBroadcaster.sendTransform(
        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y,0),
        (0,0,0,1),
        time_now,
        t55_trans_from_dum_c2__drop,                          # to   tag-from-camera-body t55_from_c2
        '/map')                                         # from camera-body c2
    axisMarker(req.visualFeature.id,t55_trans_from_dum_c2__drop)


    t55_transrot_from_dum_c2 = "dum_%s_trans_rot_to_%s"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        t55_transrot_from_dum_c2,
        t55_trans_from_dum_c2)

    t55_transrot_from_dum_c2_b = "dum_%s_trans_rot_to_%s_b"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (req.visualFeature.pose.pose.orientation.x, -req.visualFeature.pose.pose.orientation.y, -req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        t55_transrot_from_dum_c2_b,
        t55_transrot_from_dum_c2 )

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
    if 't9170'==t55:
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
            print "------------------- start publish initialpose 250 ----------------------"
            tfListener.waitForTransform('map', t55_transrot_from_dum_c2_robot_pose_170, rospy.Time(), rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_robot_pose_170,  rospy.Time(0))
            publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, 'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3], [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942])
            publish_pose_xyz_xyzw(pose_publisher,time_now,  'map', pos_[0], pos_[1], pos_[2], quat_[0], quat_[1], quat_[2], quat_[3])
            
            print "------------------- published initialpose 250 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 250: {0}".format(err)
    elif 't9250'==t55:
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
            print "------------------- published initialpose 250 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 250: {0}".format(err)
    elif 't9290'==t55:
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
            print "------------------- published initialpose 290 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 290: {0}".format(err)
    elif 't9330'==t55:
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
            print "------------------- published initialpose 330 ----------------------"
        except tf.Exception as err:
            print "some tf exception happened 330: {0}".format(err)
    elif 't9210'==t55:                                  # tag 210 is the target tag : if it moves more than 20cm from last posn, publish an updated target 
        try:
            print "------------------- start check 210 as target ----------------------"
            tfListener.waitForTransform('map',              t55_transrot_from_dum_c2_post90y180zneg90z,  rospy.Time(),  rospy.Duration(1))
            pos_, quat_ = tfListener.lookupTransform('map', t55_transrot_from_dum_c2_post90y180zneg90z,  rospy.Time(0)  )
            if abs(tag_210_target_pose.pose.position.x - pos_[0]) > 0.2  or  abs(tag_210_target_pose.pose.position.y - pos_[1]) > 0.2 : 
                tag_210_target_pose.position.x = pos_[0]
                tag_210_target_pose.position.y = pos_[1]    
                publish_pose_xyz_xyzw(tag_210_target_publisher,time_now,  'map', pos_[0], pos_[1], 0.0, quat_[0], quat_[1], 0-quat_[2], quat_[3])  # NOTE: z is zero for ground robots, qz is negated to make it run up against the tag
                print "------------------- 210 re-published as target ----------------------"                
            else :
                print "------------------- 210 not changed enough to re-publish as target ----------------------"                
        except tf.Exception as err:
            print "some tf exception happened 210: {0}".format(err)        

    ori = req.visualFeature.pose.pose.orientation
    pos = req.visualFeature.pose.pose.position

# removed tf transforms code is at the bottom of this file

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


class VisionSource:
    def __init__(self, vision_source_id_, base_url_, publisher_to_phone_):
        self.vision_source_id = vision_source_id_
        self.base_url         = base_url_
        self.publisher_to_phone  = publisher_to_phone_


def phone_whereis_setupPublisherForPhoneCamera(phone_camera_id_string_):  # problem with Android phones being able to serve ROS services: set up a pulisher from this 'Server', and set up a subscriber on the phone
    topic_name = "/phone_whereis/" + phone_camera_id_string_
    new_publisher = rospy.Publisher(topic_name, WhereIsAsPub, queue_size=2, latch=True)
    print "set up publisher for " + topic_name + " : problem with Android phones being able to serve ROS services: set up a pulisher from this 'Server', and set up a subscriber on the phone"
    return new_publisher
    

def register_vision_source_callback(req_registerVisionSource):
    print "start register_vision_source_callback(req_registerVisionSource)"
    print "start register_vision_source_callback("+req_registerVisionSource.vision_source_base_url+")"
    print "start register_vision_source_callback("+str(req_registerVisionSource.vision_source_base_url)+")"
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
    elif new_vision_source.vision_source_id in ['c20', 'c21']:
        #   dummy frame for the camera: 1m up away from origin, facing 45 degrees right - toward corner of Michael's office
        #   fixed camera pose
        response.acknowledgement = 'registered vos_id=%s x=1.9 y=-5.75 z=0.57 qx=0 qy=0 qz=0.383 qw=0.92374834235304581'%(new_vision_source.vision_source_id)
    else:
        response.acknowledgement = 'registered'
    rospy.loginfo('register_vision_source_callback: registered vision source %s with base URL %s',new_vision_source.vision_source_id, new_vision_source.base_url)
    print vision_sources
    return response




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
    pose_quat = tf.transformations.quaternion_from_euler(r,p,yaw)
    pose = PoseStamped()
    pose.header.stamp = time_now
    pose.header.frame_id = id_
    pose.pose.pose = Pose(Point(x, y, z), Quaternion(*pose_quat))  # NOTE: '*' means turn into a list of arguments or some such
    pose_publisher.publish(pose)


# id_ is the frame that this pose is relative to: e.g. if this pose is relative to the map, use "map"
def publish_pose_xyz_xyzw(pose_publisher,time_now, id_, x, y, z, qx, qy, qz, qw):
    # next, we'll publish the pose message over ROS
    pose = PoseStamped()
    pose.header.stamp = time_now
    pose.header.frame_id = id_
    pose.pose = Pose(Point(x, y, z), Quaternion(qx, qy, qz, qw))
    pose_publisher.publish(pose)

# id_ is the frame that this pose is relative to: e.g. if this pose is relative to the map, use "map"
def publish_pose_xyz_xyzw_covar(initialpose_poseWCS_publisher, fakelocalisation_poseWCS_publisher, time_now, id_, x, y, z, qx, qy, qz, qw, covariance_):
    # next, we'll publish the pose message over ROS
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

    global where_is_server
    where_is_server = rospy.Service('/vos_server/where_is', where_is_alg_desc, where_is_callback)



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
    display_status_subscriber = rospy.Subscriber('/androidvosopencvros/display_status',std_msgs.msg.String, display_status_callback)
    print "Ready to display current status to console"
    rospy.loginfo("Ready to display current status to console")
    fixed_tag_pose_subscriber = rospy.Subscriber('/androidvosopencvros/fixed_tag_pose',std_msgs.msg.String, fixed_tag_pose_callback)
    print "Ready to update fixed tags"
    rospy.loginfo("Ready to update fixed tags")

    global pose_publisher
    pose_publisher = rospy.Publisher("poses_from_requests", PoseStamped, queue_size=50)
    print "Ready to publish poses"
    rospy.loginfo("Ready to publish poses")
    
    global tag_210_target_publisher
    pose_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    #pose_publisher = rospy.Publisher("/move_base/goal", move_base_msgs.MoveBaseActionGoal, queue_size=1)
    print "Ready to publish tag 210 as target poses"
    rospy.loginfo("Ready to publish tag 210 as target poses")
    
    global initialpose_poseWCS_publisher
    initialpose_poseWCS_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=50, latch=True)  # latch to make sure that AMCL has an intitial pose to use
    print "Ready to publish poses with covariance"
    rospy.loginfo("Ready to publish poses with covariance")
    
    global fakelocalisation_poseWCS_publisher
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
#                    whereis = rospy.ServiceProxy(C60__WHERE_IS__SERVICE_NAME, where_is_alg_desc)
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



#     publish_pose_xyz_xyzw(pose_publisher, time_now, t55_transrot_from_dum_c2, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
#
#     #
#     # t55_transrot_from_dum_c2 = "dum_%s_rospy_to_%s"%(c2,t55)
#     # tfBroadcaster.sendTransform(
#     #     (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
#     #     (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
#     #     time_now,
#     #     t55_transrot_from_dum_c2,
#     #     t55_trans_from_dum_c2)
#     # #
#     # create the robot-convention camera body frame, step 1 : -90Z
#     # t55_from_dum_c2_negz
#     tfBroadcaster.sendTransform(
#         (0.0, 0.0, 0.0),
#         ( 0.0, 0.0, -0.70710678118654757, 0.70710678118654757 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
#         time_now,
#         t55_transrot_from_dum_c2 + 'negz' ,  # to
#         t55_transrot_from_dum_c2 )        # from
#
#     # create the robot-convention camera body frame, step 2 : -90Y
#     # t55_from_dum_c2_negznegy
#     tfBroadcaster.sendTransform(
#         (0.0, 0.0, 0.0),
#         ( 0.0, -0.70710678118654757, 0.0, 0.70710678118654757 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
#         time_now,
#         t55_transrot_from_dum_c2 + 'negznegy' ,  # to
#         t55_transrot_from_dum_c2 + 'negz' )        # from
#
#     # create the robot-convention camera body frame, step 2 : +90x
#     # t55_from_dum_c2_negzposx
#     tfBroadcaster.sendTransform(
#         (0.0, 0.0, 0.0),
#         ( 1.0, 0.0, 0.0, 0.0 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
#         time_now,
#         t55_transrot_from_dum_c2 + 'negz180x' ,  # to
#         t55_transrot_from_dum_c2 + 'negz' )        # from
#
#     # create the robot-convention camera body frame, step 2 : +90x
#     # t55_from_dum_c2_negzposx
#     tfBroadcaster.sendTransform(
#         (0.0, 0.0, 0.0),
#         ( 0.70710678118654757, 0.0, 0.0, 0.70710678118654757 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
#         time_now,
#         t55_transrot_from_dum_c2 + 'negzposx' ,  # to
#         t55_transrot_from_dum_c2 + 'negz' )        # from
#     axisMarker(req.visualFeature.id, t55_transrot_from_dum_c2 + 'negzposx')
#
#     mirror_in_xy_origin_point     = np.zeros(4)
#     mirror_in_xy_origin_point[3]  = 1.0                   # homogeneous
#     mirror_in_xy_normal_is_z_axis = np.zeros(3)
#     mirror_in_xy_normal_is_z_axis[2] = 1.0
#     mirror_in_xy_matrix  = tf.transformations.reflection_matrix(mirror_in_xy_origin_point, mirror_in_xy_normal_is_z_axis)  # point is homogeneous 4-vec, normal is 3-vec
#     mirror_in_xy_quat    = tf.transformations.quaternion_from_matrix(mirror_in_xy_matrix)
#     tfBroadcaster.sendTransform(
#         (0.0, 0.0, 0.0),
#         ( mirror_in_xy_quat[0], mirror_in_xy_quat[1], mirror_in_xy_quat[2], mirror_in_xy_quat[3] ), # https://github.com/ros/geometry/blob/indigo-devel/tf/src/tf/transformations.py
#         time_now,
#         t55_transrot_from_dum_c2 + 'negzposxmirrorxy' ,  # to
#         t55_transrot_from_dum_c2 + 'negzposx' )        # from
#
# ##  DOESNT MIRROR - JUST ROTATES
#     # tfBroadcaster.sendTransform(
#     #     (0.0, 0.0, 0.0),
#     #     ( 0.0, 0.0, -1.0, -1.0 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
#     #     time_now,
#     #     t55_transrot_from_dum_c2 + 'negzposxmirrorxy' ,  # to
#     #     t55_transrot_from_dum_c2 + 'negzposx' )        # from
#
# # now need to negate the Z i.e. mirror through the XY plane
#
#
#     tfBroadcaster.sendTransform(
#         (0.0, 0.0, 0.0),
#         ( 0.0, 0, 1, 0 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
#         time_now,
#         t55_transrot_from_dum_c2 + '180z' ,  # to
#         t55_transrot_from_dum_c2 )        # from
#
# # END working through BoofCV transforms and orientations: BoofCV uses yet another frame for tags
#
#     #
#     tfBroadcaster.sendTransform(
#         (req.cameraPose.pose.position.x,req.cameraPose.pose.position.y,req.cameraPose.pose.position.z),
#         (req.cameraPose.pose.orientation.x,req.cameraPose.pose.orientation.y,req.cameraPose.pose.orientation.z,req.cameraPose.pose.orientation.w),
#         time_now,
#         req.cameraPose.header.frame_id,            # to      '/cam/%s/pose' % req.cameraPose.header.frame_id e.g. "/cam/c_1/pose"    # TODO - remove hardcoding to base namespace
#         'map')                                                      # from frame
#
#
#     # publish the map-to-camera_pose tf
#     tfBroadcaster.sendTransform(
#         (req.cameraPose.pose.position.x,req.cameraPose.pose.position.y,req.cameraPose.pose.position.z),
#         (req.cameraPose.pose.orientation.x,req.cameraPose.pose.orientation.y,req.cameraPose.pose.orientation.z,req.cameraPose.pose.orientation.w),
#         time_now,
#         req.cameraPose.header.frame_id + '_in_detect_feature_req_header',            # to      '/cam/%s/pose' % req.cameraPose.header.frame_id e.g. "/cam/c_1/pose"    # TODO - remove hardcoding to base namespace
#         'map')                                                      # from frame
#
#
#
#
#
#
#     # create the robot-convention camera body frame, step 1 : +90Y
#     tfBroadcaster.sendTransform(
#         (0.0, 0.0, 0.0),
#         ( 0.0, 0.70710678118654757, 0.0, 0.70710678118654757 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
#         time_now,
#         req.cameraPose.header.frame_id + 'y' ,  # to
#         req.cameraPose.header.frame_id )        # from
#
#     # create the robot-convention camera body frame, step 2 : +90Z
#     tfBroadcaster.sendTransform(
#         (0.0, 0.0, 0.0),
#         ( 0.0, 0.0, 0.70710678118654757, 0.70710678118654757 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
#         time_now,
#         req.cameraPose.header.frame_id + 'yz' ,  # to
#         req.cameraPose.header.frame_id + 'y' )        # from
#
#     camera_tag_frame_id = '%s_%s%s' % (req.cameraPose.header.frame_id, algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)
#
#     t55 = '%s%s'%(algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)
#
#
#         # THIS IS THE GOOD TAG POSITION, AND GOOD TAG ORIENTATION BUT THE TAG FACES AWAY FROM THE CAMERA
#         # INTIIALLY GOOD AND ALIGNED WITH c2_from_fixed_t55 BUT THEN ROTATES STRANGELY IF THE CAMERA ROTATES
#     t55_from_c2 = '%s%s_from_%s'% (algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id, req.cameraPose.header.frame_id)
#     tfBroadcaster.sendTransform(
#         (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
#         #(req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
#         (req.visualFeature.pose.pose.orientation.z, -req.visualFeature.pose.pose.orientation.x, -req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.w),
#         time_now,
#         t55_from_c2,                          # to   tag-from-camera-body t55_from_c2
#         req.cameraPose.header.frame_id)                     # from camera-body c2
#
#         # THIS IS THE GOOD TAG POSITION, AND GOOD TAG ORIENTATION BUT THE TAG FACES AWAY FROM THE CAMERA
#         # INTIIALLY GOOD AND ALIGNED WITH c2_from_fixed_t55 BUT THEN ROTATES STRANGELY IF THE CAMERA ROTATES
#     t55_from_c2_boof = '%s%s_from_%s_boof'% (algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id, req.cameraPose.header.frame_id)
#     tfBroadcaster.sendTransform(
#         (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
#         #(req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
#         (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
#         time_now,
#         t55_from_c2_boof,                          # to   tag-from-camera-body t55_from_c2
#         req.cameraPose.header.frame_id)                     # from camera-body c2
#
#
#
#
#
#
#
#     # GOOD:  t55_from_c2_180x
#     # GOOD with translationRotationWithAxisChange --> tagDetection.getRelativeTranslationRotation
#     # Problem is the once-off localisation against the fixed tag
#         #  THIS IS THE GOOD TAG POSE, with the visible tag facing toward the camera
#         #  INTIIALLY GOOD  BUT THEN ROTATES STRANGELY WITH t55_from_c2 IF THE CAMERA ROTATES
#     t55_from_c2_180x_boof = t55_from_c2_boof + '_180x'
#     tfBroadcaster.sendTransform(
#         (0.0, 0.0, 0.0),
#         (1.0, 0.0, 0.0, 0.0),
#         time_now,
#         t55_from_c2_180x_boof,                   # to
#         t55_from_c2_boof)                        # from
#
#
#     # GOOD:  t55_from_c2_180x
#     # GOOD with translationRotationWithAxisChange --> tagDetection.getRelativeTranslationRotation
#     # Problem is the once-off localisation against the fixed tag
#         #  THIS IS THE GOOD TAG POSE, with the visible tag facing toward the camera
#         #  INTIIALLY GOOD  BUT THEN ROTATES STRANGELY WITH t55_from_c2 IF THE CAMERA ROTATES
#     t55_from_c2_180x = t55_from_c2 + '_180x'
#     tfBroadcaster.sendTransform(
#         (0.0, 0.0, 0.0),
#         (1.0, 0.0, 0.0, 0.0),
#         time_now,
#         t55_from_c2_180x,                   # to
#         t55_from_c2)                        # from
#
#     # simplify the below (c2_from_t55_from_c2_180x is GOOD, but maybe simplify)
#     quat_t55_from_c2_plain = [req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w]
#     quat_c2_from_t55_plain = inverse(quat_t55_from_c2_plain)
#     fixed_t55 = 'fixed_%s'%(t55)
#
#
#     # GOOD:  c2_from_t55_from_c2_180x
#     # GOOD with translationRotationWithAxisChange --> tagDetection.getRelativeTranslationRotation
#     # Problem is the once-off localisation against the fixed tag
#         # NOTE 1:  t55 from c2  is  [ +z -x -y +w ]
#         # NOTE 2:  c2 from t55  is  inverse( [ +z -x -y +w ] )
#     quat_t55_from_c2 = [req.visualFeature.pose.pose.orientation.z, -req.visualFeature.pose.pose.orientation.x, -req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.w]
#     quat_c2_from_t55 = inverse(quat_t55_from_c2)
#     c2_from_t55_from_c2_180x = '%s_from_%s'%(cN,t55_from_c2_180x)
#     c2_from_t55_from_c2_180x_tmp = c2_from_t55_from_c2_180x+'_tmp'
#     tfBroadcaster.sendTransform(
#         #(req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, -req.visualFeature.pose.pose.position.z),
#         (0.0,0.0,0.0),
#         quat_c2_from_t55,
#         time_now,
#         c2_from_t55_from_c2_180x_tmp,           # to
#         t55_from_c2)                   # from
#     tfBroadcaster.sendTransform(
#         (-req.visualFeature.pose.pose.position.x, -req.visualFeature.pose.pose.position.y, -req.visualFeature.pose.pose.position.z),
#         (0.0,0.0,0.0,1.0),
#         time_now,
#         c2_from_t55_from_c2_180x,           # to
#         c2_from_t55_from_c2_180x_tmp)                   # from
#
#     #  from fixed tag to camera
#     t55_mirrored      = t55+'_mirrored'           # tag is rot+-180Z from robot pov: AprilTags gives tag pose as away from camera i.e. tag x-axis is into the tag / robot-convention frame aligned with back of tag, I treat tag x-axis as out of the tag / robot-convention axes aligned with face of tag
#     #   t55 = '%s%s'%(algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)
#     c2_from_fixed_t55       = '%s_from_fixed_%s'%(cN,t55)
#     c2_from_fixed_t55_tmp   = c2_from_fixed_t55+'_tmp'
#     c2_from_fixed_t55_tmp180x   = c2_from_fixed_t55+'_tmp180x'
#
#     tfBroadcaster.sendTransform(
#         (0.0, 0.0, 0.0),                # same position: _t55_tmp180x is the same position as t55 / t55 ...
#         (1.0, 0.0, 0.0, 0.0),           # 180 around x:  ... but rotated 180 around x
#         time_now,                       # simultaneous 'now'
#         c2_from_fixed_t55_tmp180x,      # to
#         t55)                 # from
#     tfBroadcaster.sendTransform(
#         #(req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, -req.visualFeature.pose.pose.position.z),
#         (0.0,0.0,0.0),                  # _tmp is same position as the fixed tag ...
#         quat_c2_from_t55,               # ... but inverse quaternion to point back toward the camera
#         time_now,                       # simultaneous 'now'
#         c2_from_fixed_t55_tmp,          # to
#         c2_from_fixed_t55_tmp180x)      # from
#     tfBroadcaster.sendTransform(
#         (-req.visualFeature.pose.pose.position.x, -req.visualFeature.pose.pose.position.y, -req.visualFeature.pose.pose.position.z),    # inverse/reversed translation from fixed tag ...
#         (0.0,0.0,0.0,1.0),              # ... same orientation as the _tmp
#         time_now,                       # simultaneous 'now'
#         c2_from_fixed_t55,              # to
#         c2_from_fixed_t55_tmp)          # from
#     axisMarker(req.visualFeature.id,c2_from_fixed_t55)
#
#
#
#     tfBroadcaster.sendTransform(
#         (0,0,0),
#         (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
#         time_now,
#         camera_tag_frame_id,
#         camera_tag_frame_id + 't')              # from      '/cam/%s/pose' % req.cameraPose.header.frame_id e.g. "/cam/c_1/pose"    # TODO - remove hardcoding to base namespace
#
#
#
#     tag_label = '%s%s'%(algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)
#
#
#
#     print camera_tag_frame_id + '_mirrored_translation', ': from camera to tag: x=' , req.visualFeature.pose.pose.position.z, ', y=', -req.visualFeature.pose.pose.position.x, ', z=', -req.visualFeature.pose.pose.position.y
#     print camera_tag_frame_id + '_mirrored_translation', ': from tag to camera: x=' , req.visualFeature.pose.pose.position.z, ', y=', -req.visualFeature.pose.pose.position.x, ', z=', req.visualFeature.pose.pose.position.y
#
