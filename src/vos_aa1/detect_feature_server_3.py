#!/usr/bin/env python

import sys
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped  # https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf

from tf import transformations
import numpy as np
from visualization_msgs.msg import Marker
import threading
from pprint import pprint
import std_msgs
import time


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

# "2|-0.5|-3|1.15|0|0|-0.7071|0.7071"
tag_2 = VisualFeatureInWorld()
tag_2.algorithm = 'AprilTags_Kaess_36h11'
tag_2.id = '2'
tag_2.pose = Pose()
tag_2.pose.position.x=-0.5
tag_2.pose.position.y=-3
tag_2.pose.position.z=1.15
tag_2.pose.orientation.x=0
tag_2.pose.orientation.y=0
tag_2.pose.orientation.z=-0.7071
tag_2.pose.orientation.w=0.7071
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

# 0|2.5|-1.5|1.25|0|0|-0.7071|0.7071
tag_0 = VisualFeatureInWorld()
tag_0.algorithm = 'AprilTags_Kaess_36h11'
tag_0.id = '0'
tag_0.pose = Pose()
tag_0.pose.position.x=2.5
tag_0.pose.position.y=-1.5
tag_0.pose.position.z=1.25
tag_0.pose.orientation.x=0
tag_0.pose.orientation.y=0
tag_0.pose.orientation.z=-0.7071
tag_0.pose.orientation.w=0.7071
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
tag_210.pose.orientation.z=-0.5571
tag_210.pose.orientation.w=0.7071
fixed_features.append(tag_210)

# features_present = (0,2,3,9)
# features_present = (170, 210, 250, 290, 330, 370, 410, 450, 490, 530, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59)
# features_present = (210, 1210, 2210, 3210, 4210, 5210, 6210, 7210, 8210, 9210, 10210,   22210)
features_present = (9210 , -9000)

vision_sources = []


marker_publisher = 1
pose_publisher = 1


tfBroadcaster__ = []

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
    listener = tf.TransformListener()

    #  1) set up the tf names to use as variables
    #  2) use those variables to pull the tfs and respond to the camera self-localisation request
    for x in fixed_features:
        print '---- fixed_feature exists with id "',x.id,'"'

    if any( x.id == visual_feature_descriptor or str(x.id) == str(visual_feature_descriptor) for x in fixed_features):
        print '---- localise_from_a_feature_callback: fixed feature "',x,'" matches descriptor "',visual_feature_descriptor,'"'
        tag_same_fixed = '%s%s'%(algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)
        c2_from_fixed_t55 = '%s_from_fixed_%s'%(cN,tag_same_fixed)
        try:
            listener.waitForTransform('map', c2_from_fixed_t55, rospy.Time(), rospy.Duration(4.0))      # from the map origin, to the camera, through a fixed point
            position_, quaternion_ = listener.lookupTransform('map', c2_from_fixed_t55,  rospy.Time())
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
        response = DetectedFeatureResponse()
        response.acknowledgement="feature not present"
        return response

    # # TODO - something about this conversion is not right: the translation and quaternion match those found by AprilTags_Kaess and sent by DetectedFeatureClient, but the RPY are different
    # euler = tf.transformations.euler_from_quaternion([req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w])
    # print "  feature x_y_z_w roll=%.4f pitch=%.4f yaw=%.4f"%(euler[0], euler[1], euler[2])

    tfBroadcaster = tf.TransformBroadcaster()
    time_now = rospy.Time.now()



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

#   dummy frame for the camera: 1m up away from origin, same orientation
    dum_c2 = "dum_%s"%(c2)
    tfBroadcaster.sendTransform(
        (0,0,1),
        (0,0,0,1),
        time_now,
        dum_c2,        # to
        'map')                          # from

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

    t55_transrot_from_dum_c2_pre90y = "dum_%s_trans_rot_to_%s_pre90y"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (0, 0.7071, 0, 0.7071),
        time_now,
        t55_transrot_from_dum_c2_pre90y,
        t55_trans_from_dum_c2)

    t55_transrot_from_dum_c2_pre90y90z = "dum_%s_trans_rot_to_%s_pre90y90z"%(c2,t55)
    tfBroadcaster.sendTransform(
        (0,0,0),
        (0, 0, 0.7071, 0.7071),
        time_now,
        t55_transrot_from_dum_c2_pre90y90z,
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


    ori = req.visualFeature.pose.pose.orientation
    pos = req.visualFeature.pose.pose.position



    publish_pose_xyz_xyzw(pose_publisher, time_now, t55_transrot_from_dum_c2, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)

    #
    # t55_transrot_from_dum_c2 = "dum_%s_rospy_to_%s"%(c2,t55)
    # tfBroadcaster.sendTransform(
    #     (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
    #     (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
    #     time_now,
    #     t55_transrot_from_dum_c2,
    #     t55_trans_from_dum_c2)
    # #
    # create the robot-convention camera body frame, step 1 : -90Z
    # t55_from_dum_c2_negz
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, 0.0, -0.7071, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        t55_transrot_from_dum_c2 + 'negz' ,  # to
        t55_transrot_from_dum_c2 )        # from

    # create the robot-convention camera body frame, step 2 : -90Y
    # t55_from_dum_c2_negznegy
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, -0.7071, 0.0, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        t55_transrot_from_dum_c2 + 'negznegy' ,  # to
        t55_transrot_from_dum_c2 + 'negz' )        # from

    # create the robot-convention camera body frame, step 2 : +90x
    # t55_from_dum_c2_negzposx
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 1.0, 0.0, 0.0, 0.0 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        t55_transrot_from_dum_c2 + 'negz180x' ,  # to
        t55_transrot_from_dum_c2 + 'negz' )        # from

    # create the robot-convention camera body frame, step 2 : +90x
    # t55_from_dum_c2_negzposx
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.7071, 0.0, 0.0, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        t55_transrot_from_dum_c2 + 'negzposx' ,  # to
        t55_transrot_from_dum_c2 + 'negz' )        # from
    axisMarker(req.visualFeature.id, t55_transrot_from_dum_c2 + 'negzposx')

    mirror_in_xy_origin_point     = np.zeros(4)
    mirror_in_xy_origin_point[3]  = 1.0                   # homogeneous
    mirror_in_xy_normal_is_z_axis = np.zeros(3)
    mirror_in_xy_normal_is_z_axis[2] = 1.0
    mirror_in_xy_matrix  = tf.transformations.reflection_matrix(mirror_in_xy_origin_point, mirror_in_xy_normal_is_z_axis)  # point is homogeneous 4-vec, normal is 3-vec
    mirror_in_xy_quat    = tf.transformations.quaternion_from_matrix(mirror_in_xy_matrix)
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( mirror_in_xy_quat[0], mirror_in_xy_quat[1], mirror_in_xy_quat[2], mirror_in_xy_quat[3] ), # https://github.com/ros/geometry/blob/indigo-devel/tf/src/tf/transformations.py
        time_now,
        t55_transrot_from_dum_c2 + 'negzposxmirrorxy' ,  # to
        t55_transrot_from_dum_c2 + 'negzposx' )        # from

##  DOESNT MIRROR - JUST ROTATES
    # tfBroadcaster.sendTransform(
    #     (0.0, 0.0, 0.0),
    #     ( 0.0, 0.0, -1.0, -1.0 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
    #     time_now,
    #     t55_transrot_from_dum_c2 + 'negzposxmirrorxy' ,  # to
    #     t55_transrot_from_dum_c2 + 'negzposx' )        # from

# now need to negate the Z i.e. mirror through the XY plane


    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, 0, 1, 0 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        t55_transrot_from_dum_c2 + '180z' ,  # to
        t55_transrot_from_dum_c2 )        # from

# END working through BoofCV transforms and orientations: BoofCV uses yet another frame for tags

    #
    tfBroadcaster.sendTransform(
        (req.cameraPose.pose.position.x,req.cameraPose.pose.position.y,req.cameraPose.pose.position.z),
        (req.cameraPose.pose.orientation.x,req.cameraPose.pose.orientation.y,req.cameraPose.pose.orientation.z,req.cameraPose.pose.orientation.w),
        time_now,
        req.cameraPose.header.frame_id,            # to      '/cam/%s/pose' % req.cameraPose.header.frame_id e.g. "/cam/c_1/pose"    # TODO - remove hardcoding to base namespace
        'map')                                                      # from frame


    # publish the map-to-camera_pose tf
    tfBroadcaster.sendTransform(
        (req.cameraPose.pose.position.x,req.cameraPose.pose.position.y,req.cameraPose.pose.position.z),
        (req.cameraPose.pose.orientation.x,req.cameraPose.pose.orientation.y,req.cameraPose.pose.orientation.z,req.cameraPose.pose.orientation.w),
        time_now,
        req.cameraPose.header.frame_id + '_in_detect_feature_req_header',            # to      '/cam/%s/pose' % req.cameraPose.header.frame_id e.g. "/cam/c_1/pose"    # TODO - remove hardcoding to base namespace
        'map')                                                      # from frame






    # create the robot-convention camera body frame, step 1 : +90Y
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, 0.7071, 0.0, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        req.cameraPose.header.frame_id + 'y' ,  # to
        req.cameraPose.header.frame_id )        # from

    # create the robot-convention camera body frame, step 2 : +90Z
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, 0.0, 0.7071, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        req.cameraPose.header.frame_id + 'yz' ,  # to
        req.cameraPose.header.frame_id + 'y' )        # from

    camera_tag_frame_id = '%s_%s%s' % (req.cameraPose.header.frame_id, algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)

    t55 = '%s%s'%(algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)


        # THIS IS THE GOOD TAG POSITION, AND GOOD TAG ORIENTATION BUT THE TAG FACES AWAY FROM THE CAMERA
        # INTIIALLY GOOD AND ALIGNED WITH c2_from_fixed_t55 BUT THEN ROTATES STRANGELY IF THE CAMERA ROTATES
    t55_from_c2 = '%s%s_from_%s'% (algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id, req.cameraPose.header.frame_id)
    tfBroadcaster.sendTransform(
        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        #(req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        (req.visualFeature.pose.pose.orientation.z, -req.visualFeature.pose.pose.orientation.x, -req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.w),
        time_now,
        t55_from_c2,                          # to   tag-from-camera-body t55_from_c2
        req.cameraPose.header.frame_id)                     # from camera-body c2

        # THIS IS THE GOOD TAG POSITION, AND GOOD TAG ORIENTATION BUT THE TAG FACES AWAY FROM THE CAMERA
        # INTIIALLY GOOD AND ALIGNED WITH c2_from_fixed_t55 BUT THEN ROTATES STRANGELY IF THE CAMERA ROTATES
    t55_from_c2_boof = '%s%s_from_%s_boof'% (algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id, req.cameraPose.header.frame_id)
    tfBroadcaster.sendTransform(
        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        #(req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        t55_from_c2_boof,                          # to   tag-from-camera-body t55_from_c2
        req.cameraPose.header.frame_id)                     # from camera-body c2







    # GOOD:  t55_from_c2_180x
    # GOOD with translationRotationWithAxisChange --> tagDetection.getRelativeTranslationRotation
    # Problem is the once-off localisation against the fixed tag
        #  THIS IS THE GOOD TAG POSE, with the visible tag facing toward the camera
        #  INTIIALLY GOOD  BUT THEN ROTATES STRANGELY WITH t55_from_c2 IF THE CAMERA ROTATES
    t55_from_c2_180x_boof = t55_from_c2_boof + '_180x'
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        time_now,
        t55_from_c2_180x_boof,                   # to
        t55_from_c2_boof)                        # from


    # GOOD:  t55_from_c2_180x
    # GOOD with translationRotationWithAxisChange --> tagDetection.getRelativeTranslationRotation
    # Problem is the once-off localisation against the fixed tag
        #  THIS IS THE GOOD TAG POSE, with the visible tag facing toward the camera
        #  INTIIALLY GOOD  BUT THEN ROTATES STRANGELY WITH t55_from_c2 IF THE CAMERA ROTATES
    t55_from_c2_180x = t55_from_c2 + '_180x'
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        time_now,
        t55_from_c2_180x,                   # to
        t55_from_c2)                        # from

    # simplify the below (c2_from_t55_from_c2_180x is GOOD, but maybe simplify)
    quat_t55_from_c2_plain = [req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w]
    quat_c2_from_t55_plain = inverse(quat_t55_from_c2_plain)
    fixed_t55 = 'fixed_%s'%(t55)


    # GOOD:  c2_from_t55_from_c2_180x
    # GOOD with translationRotationWithAxisChange --> tagDetection.getRelativeTranslationRotation
    # Problem is the once-off localisation against the fixed tag
        # NOTE 1:  t55 from c2  is  [ +z -x -y +w ]
        # NOTE 2:  c2 from t55  is  inverse( [ +z -x -y +w ] )
    quat_t55_from_c2 = [req.visualFeature.pose.pose.orientation.z, -req.visualFeature.pose.pose.orientation.x, -req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.w]
    quat_c2_from_t55 = inverse(quat_t55_from_c2)
    c2_from_t55_from_c2_180x = '%s_from_%s'%(cN,t55_from_c2_180x)
    c2_from_t55_from_c2_180x_tmp = c2_from_t55_from_c2_180x+'_tmp'
    tfBroadcaster.sendTransform(
        #(req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, -req.visualFeature.pose.pose.position.z),
        (0.0,0.0,0.0),
        quat_c2_from_t55,
        time_now,
        c2_from_t55_from_c2_180x_tmp,           # to
        t55_from_c2)                   # from
    tfBroadcaster.sendTransform(
        (-req.visualFeature.pose.pose.position.x, -req.visualFeature.pose.pose.position.y, -req.visualFeature.pose.pose.position.z),
        (0.0,0.0,0.0,1.0),
        time_now,
        c2_from_t55_from_c2_180x,           # to
        c2_from_t55_from_c2_180x_tmp)                   # from

    #  from fixed tag to camera
    t55_mirrored      = t55+'_mirrored'           # tag is rot+-180Z from robot pov: AprilTags gives tag pose as away from camera i.e. tag x-axis is into the tag / robot-convention frame aligned with back of tag, I treat tag x-axis as out of the tag / robot-convention axes aligned with face of tag
    #   t55 = '%s%s'%(algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)
    c2_from_fixed_t55       = '%s_from_fixed_%s'%(cN,t55)
    c2_from_fixed_t55_tmp   = c2_from_fixed_t55+'_tmp'
    c2_from_fixed_t55_tmp180x   = c2_from_fixed_t55+'_tmp180x'

    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),                # same position: _t55_tmp180x is the same position as t55 / t55 ...
        (1.0, 0.0, 0.0, 0.0),           # 180 around x:  ... but rotated 180 around x
        time_now,                       # simultaneous 'now'
        c2_from_fixed_t55_tmp180x,      # to
        t55)                 # from
    tfBroadcaster.sendTransform(
        #(req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, -req.visualFeature.pose.pose.position.z),
        (0.0,0.0,0.0),                  # _tmp is same position as the fixed tag ...
        quat_c2_from_t55,               # ... but inverse quaternion to point back toward the camera
        time_now,                       # simultaneous 'now'
        c2_from_fixed_t55_tmp,          # to
        c2_from_fixed_t55_tmp180x)      # from
    tfBroadcaster.sendTransform(
        (-req.visualFeature.pose.pose.position.x, -req.visualFeature.pose.pose.position.y, -req.visualFeature.pose.pose.position.z),    # inverse/reversed translation from fixed tag ...
        (0.0,0.0,0.0,1.0),              # ... same orientation as the _tmp
        time_now,                       # simultaneous 'now'
        c2_from_fixed_t55,              # to
        c2_from_fixed_t55_tmp)          # from
    axisMarker(req.visualFeature.id,c2_from_fixed_t55)



    tfBroadcaster.sendTransform(
        (0,0,0),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        camera_tag_frame_id,
        camera_tag_frame_id + 't')              # from      '/cam/%s/pose' % req.cameraPose.header.frame_id e.g. "/cam/c_1/pose"    # TODO - remove hardcoding to base namespace



    tag_label = '%s%s'%(algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)



    print camera_tag_frame_id + '_mirrored_translation', ': from camera to tag: x=' , req.visualFeature.pose.pose.position.z, ', y=', -req.visualFeature.pose.pose.position.x, ', z=', -req.visualFeature.pose.pose.position.y
    print camera_tag_frame_id + '_mirrored_translation', ': from tag to camera: x=' , req.visualFeature.pose.pose.position.z, ', y=', -req.visualFeature.pose.pose.position.x, ', z=', req.visualFeature.pose.pose.position.y




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
    def __init__(self, vision_source_id_, base_url_):
        self.vision_source_id = vision_source_id_
        self.base_url         = base_url_


def register_vision_source_callback(req_registerVisionSource):
    new_vision_source = VisionSource(req_registerVisionSource.vision_source_id,req_registerVisionSource.vision_source_base_url)
    vision_sources.append(new_vision_source)
    response = RegisterVisionSourceResponse()
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

def publish_pose_xyz_rpy(pose_publisher,time_now, id_, x, y, z, r, p, yaw):
    # next, we'll publish the pose message over ROS
    pose_quat = tf.transformations.quaternion_from_euler(r,p,yaw)
    pose = PoseStamped()
    pose.header.stamp = time_now
    pose.header.frame_id = id_
    pose.pose.pose = Pose(Point(x, y, z), Quaternion(*pose_quat))  # NOTE: '*' means turn into a list of arguments or some such
    pose_publisher.publish(pose)


def publish_pose_xyz_xyzw(pose_publisher,time_now, id_, x, y, z, qx, qy, qz, qw):
    # next, we'll publish the pose message over ROS
    pose = PoseStamped()
    pose.header.stamp = time_now
    pose.header.frame_id = id_
    pose.pose = Pose(Point(x, y, z), Quaternion(qx, qy, qz, qw))
    pose_publisher.publish(pose)



def detect_feature_server():
    rospy.init_node('detect_feature_server')

#######    write_fixed_tags_to_file("/tmp/fixed_tags")
#######    load_fixed_tags_from_file("/tmp/fixed_tags")
#######    write_fixed_tags_to_file("/tmp/fixed_tags2")
#######    load_fixed_tags_from_file("/tmp/fixed_tags2")

    detect_feature_server = rospy.Service('/androidvosopencvros/detected_feature', DetectedFeature, detect_feature_callback)
    print "Ready to receive detected features."
    localise_from_a_feature_server = rospy.Service('/androidvosopencvros/localise_from_a_feature', LocaliseFromAFeature, localise_from_a_feature_callback)
    print "Ready to receive localise from individual features."
    global marker_publisher
    marker_publisher = rospy.Publisher('axis_markers', Marker, queue_size = 100)
    print "Ready to publish markers"
    print marker_publisher
    vision_source_registration_server = rospy.Service('/androidvosopencvros/register_vision_source',RegisterVisionSource,register_vision_source_callback)
    print "Ready to register vision sources"
    display_status_subscriber = rospy.Subscriber('/androidvosopencvros/display_status',std_msgs.msg.String, display_status_callback)
    print "Ready to display current status to console"
    fixed_tag_pose_subscriber = rospy.Subscriber('/androidvosopencvros/fixed_tag_pose',std_msgs.msg.String, fixed_tag_pose_callback)
    print "Ready to update fixed tags"

    global pose_publisher
    pose_publisher = rospy.Publisher("poses_from_requests", PoseStamped, queue_size=50)
    print "Ready to publish poses"

    tfBroadcaster = tf.TransformBroadcaster()
    set_tfBroadcaster(tfBroadcaster)
    time_now = rospy.Time.now()

#    publish_map_coordinate_frame_origin_tf(tfBroadcaster,time_now)
#    publish_map_to_vos_base_frame_tf_threaded(tfBroadcaster,time_now)
    publish_map_to_vos_base_frame_tf(tfBroadcaster,time_now)
    print "Done: publish_map_to_vos_base_frame_tf"
    publish_fixed_tags_tf(tfBroadcaster, time_now)
    print "Done: publish_fixed_tags_tf"

    rospy.spin()


if __name__ == "__main__":
    detect_feature_server()
