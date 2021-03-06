#!/usr/bin/env python

import sys
import rospy
import tf
from geometry_msgs.msg import Pose
from tf import transformations
import numpy as np
from visualization_msgs.msg import Marker
import threading
from pprint import pprint
import std_msgs

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


fixed_features = []
tag_55 = VisualFeatureInWorld()
tag_55.algorithm = 'AprilTags_Kaess_36h11'
tag_55.id = '55'
tag_55.pose = Pose()
tag_55.pose.position.x=0
tag_55.pose.position.y=0
tag_55.pose.position.z=0.8
tag_55.pose.orientation.x=0
tag_55.pose.orientation.y=0
tag_55.pose.orientation.z=0
tag_55.pose.orientation.w=1
fixed_features.append(tag_55)
tag_32 = VisualFeatureInWorld()
tag_32.algorithm = 'AprilTags_Kaess_36h11'
tag_32.id = '32'
tag_32.pose = Pose()
tag_32.pose.position.x=0
tag_32.pose.position.y=0
tag_32.pose.position.z=1.18
tag_32.pose.orientation.x=0
tag_32.pose.orientation.y=0
tag_32.pose.orientation.z=0
tag_32.pose.orientation.w=1
fixed_features.append(tag_32)


vision_sources = []


marker_publisher = 1

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
    print "----------------------------------------------"
    print "detect_feature_callback: "
    print "  camera frame_id [%s] :"%(req.cameraPose.header.frame_id)
    print "  camera translation from map [%.4f,%.4f,%.4f] : "%(req.cameraPose.pose.position.x,req.cameraPose.pose.position.y,req.cameraPose.pose.position.z)
    print "  camera orientation (quaternion) [%.4f,%.4f,%.4f,%.4f] : "%(req.cameraPose.pose.orientation.x,req.cameraPose.pose.orientation.y,req.cameraPose.pose.orientation.z,req.cameraPose.pose.orientation.w)
    print "  feature algorithm [%s] : "%(req.visualFeature.algorithm)
    print "  feature id [%d] : "%(req.visualFeature.id)
    print '  feature frame_id : %s_%s%s' % (req.cameraPose.header.frame_id, algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)
    print "  feature translation [%.4f,%.4f,%.4f] : "%(req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z)
    print "  feature orientation (quaternion) [%.4f,%.4f,%.4f,%.4f] "%(req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w)
    
    cN = req.cameraPose.header.frame_id
    


    # TODO - something about this conversion is not right: the translation and quaternion match those found by AprilTags_Kaess and sent by DetectedFeatureClient, but the RPY are different
    euler = tf.transformations.euler_from_quaternion([req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w])
    print "  feature x_y_z_w roll=%.4f pitch=%.4f yaw=%.4f"%(euler[0], euler[1], euler[2])

    tfBroadcaster = tf.TransformBroadcaster()
    time_now = rospy.Time.now()
    
    # publish the map-to-camera_pose tf
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
        
        
        #  USELESS
    t55_from_c2_plain = t55_from_c2 + '_plain'      
    tfBroadcaster.sendTransform(
        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),        
        time_now,
        t55_from_c2_plain,
        req.cameraPose.header.frame_id)         # c5
        
        
        
    t55_from_c2_plain_trans = t55_from_c2 + '_plain_trans'      
    tfBroadcaster.sendTransform(
        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        (0.0, 0.0, 0.0, 1.0),        
        time_now,
        t55_from_c2_plain_trans,
        req.cameraPose.header.frame_id)
        
    t55_from_c2_plain_trans_then_rot = t55_from_c2 + '_plain_trans_then_rot'      
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),        
        time_now,
        t55_from_c2_plain_trans_then_rot,
        t55_from_c2_plain_trans)
        
        
        
    t55_from_c2_plain_rot = t55_from_c2 + '_plain_rot'      
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),        
        time_now,
        t55_from_c2_plain_rot,
        req.cameraPose.header.frame_id)
        
    t55_from_c2_plain_rot_then_trans = t55_from_c2 + '_plain_rot_then_trans'      
    tfBroadcaster.sendTransform(
        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        (0.0, 0.0, 0.0, 1.0),            
        time_now,
        t55_from_c2_plain_rot_then_trans,
        t55_from_c2_plain_rot)
        
        
        
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
        
    # a node 10cm foward-left-up of the tag centre, to aid visualisation    
    t55_from_c2_180x_plus1 = t55_from_c2_180x + '_plus1'    
    tfBroadcaster.sendTransform(
        (0.1, 0.1, 0.1),
        (0.0, 0.0, 0.0, 1.0),        
        time_now,
        t55_from_c2_180x_plus1,             # to
        t55_from_c2_180x)                   # from
        
        
    # simplify the below (c2_from_t55_from_c2_180x is GOOD, but maybe simplify)
    quat_t55_from_c2_plain = [req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w]        
    quat_c2_from_t55_plain = inverse(quat_t55_from_c2_plain)
    fixed_t55 = 'fixed_%s'%(t55)
    c2_from_fixed_t55_simple_a = '%s_from_fixed_%s_simple_a'%(cN,t55)
    c2_from_fixed_t55_simple_b = '%s_from_fixed_%s_simple_b'%(cN,t55)
    c2_from_fixed_t55_simple_bc = '%s_from_fixed_%s_simple_bc'%(cN,t55)
    
    # inverse transform in one go: check order this applies rotation and translation
    tfBroadcaster.sendTransform(
        (-req.visualFeature.pose.pose.position.x, -req.visualFeature.pose.pose.position.y, -req.visualFeature.pose.pose.position.z),
        quat_c2_from_t55_plain,        
        time_now,
        c2_from_fixed_t55_simple_a,         # to
        fixed_t55)                          # from
        
    # first inverse rotation, then inverse translation    
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        (quat_c2_from_t55_plain),        
        time_now,
        c2_from_fixed_t55_simple_b,         # to
        fixed_t55)                          # from
    tfBroadcaster.sendTransform(
        (-req.visualFeature.pose.pose.position.x, -req.visualFeature.pose.pose.position.y, -req.visualFeature.pose.pose.position.z),
        (0.0, 0.0, 0.0, 1.0),
        time_now,
        c2_from_fixed_t55_simple_bc,         # to
        c2_from_fixed_t55_simple_b)         # from
        
    
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
        
        
    
    # mirror the tag     
    t55_from_c2_mirrored = '%s%s_from_%s_mirrored'% (algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id, req.cameraPose.header.frame_id)
    tfBroadcaster.sendTransform(
        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        (-req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, -req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        t55_from_c2_mirrored,                          # to   tag-from-camera-body t55_from_c2
        req.cameraPose.header.frame_id)                     # from camera-body c2
    # correct the tag axes, step 1
    #- 90 Y
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, -0.7071, 0.0, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        t55_from_c2 + 'y' ,  # to
        t55_from_c2 )        # from
    # correct the tag axes, step 2: the correct tag axes are now published as e.g. c5_t7yx
    #+ 90 X
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.7071, 0, 0.0, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        t55_from_c2 + 'yx' ,  # to
        t55_from_c2 + 'y' )        # from
    

    # publish the camera_pose-to-tag_pose tf  -  from the robot-convention camera body frame, to the tag frame
    tfBroadcaster.sendTransform(
        #(req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        (0,0,0,1),
        time_now,
        #camera_tag_frame_id,
        camera_tag_frame_id + 't',                          # to   e.g. c1_t1 '/feature/%s/%s/pose' % algorithm, req.visualFeature.id   e.g. "/feature/t/1"     # TODO - remove hardcoding to base namespace
        req.cameraPose.header.frame_id + 'zy')              # from      '/cam/%s/pose' % req.cameraPose.header.frame_id e.g. "/cam/c_1/pose"    # TODO - remove hardcoding to base namespace

    tfBroadcaster.sendTransform(
        (0,0,0),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        camera_tag_frame_id,
        camera_tag_frame_id + 't')              # from      '/cam/%s/pose' % req.cameraPose.header.frame_id e.g. "/cam/c_1/pose"    # TODO - remove hardcoding to base namespace

    # correct the tag axes, step 1
    #- 90 Y
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, -0.7071, 0.0, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        camera_tag_frame_id + 'y' ,  # to
        camera_tag_frame_id )        # from

    # correct the tag axes, step 2: the correct tag axes are now published as e.g. c5_t7yx
    #+ 90 X
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.7071, 0, 0.0, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        camera_tag_frame_id + 'yx' ,  # to
        camera_tag_frame_id + 'y' )        # from


    tag_label = '%s%s'%(algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)


    tfBroadcaster.sendTransform(
        (0,0,0),
        (-req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, -req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        camera_tag_frame_id + '_mirrored_rotation' ,                        # to
        tag_label  )              # from
        ## NOTE: have to undo any mirroring before applying a straight inverse rotation

    tfBroadcaster.sendTransform(
        # (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, -req.visualFeature.pose.pose.position.z),  ?? Apriltags in optical-frame-format ??
        (req.visualFeature.pose.pose.position.z, -req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y),
        (0,0,0,1),
        time_now,
        camera_tag_frame_id + '_mirrored_rotation_mirrored_translation' ,                        # to
        camera_tag_frame_id + '_mirrored_rotation')              # from

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


def detect_feature_server():
    rospy.init_node('detect_feature_server')
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
    
    
    tfBroadcaster = tf.TransformBroadcaster()
    time_now = rospy.Time.now()
    for fixed_feature in fixed_features:
        tfBroadcaster.sendTransform(
            (fixed_feature.pose.position.x, fixed_feature.pose.position.y, fixed_feature.pose.position.z ),
            (fixed_feature.pose.orientation.x, fixed_feature.pose.orientation.y, fixed_feature.pose.orientation.z, fixed_feature.pose.orientation.w),
            time_now,
            '%s%s' % (algorithm_abreviations[fixed_feature.algorithm], fixed_feature.id),           # /map -> t55 
            'map')
        tfBroadcaster.sendTransform(
            (fixed_feature.pose.position.x, fixed_feature.pose.position.y, fixed_feature.pose.position.z ),
            (fixed_feature.pose.orientation.x, fixed_feature.pose.orientation.y, fixed_feature.pose.orientation.z, fixed_feature.pose.orientation.w),
            time_now,
            'fixed_%s%s' % (algorithm_abreviations[fixed_feature.algorithm], fixed_feature.id),     # /map -> fixed_t55 
            'map')
    
    rospy.spin()


if __name__ == "__main__":
    detect_feature_server()
