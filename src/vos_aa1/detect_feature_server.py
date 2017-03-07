#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose
from tf import transformations

from vos_aa1.srv import DetectedFeature
from vos_aa1.srv import DetectedFeatureRequest
from vos_aa1.srv import DetectedFeatureResponse
from vos_aa1.srv import LocaliseFromAFeature
from vos_aa1.srv import LocaliseFromAFeatureRequest
from vos_aa1.srv import LocaliseFromAFeatureResponse
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
    time_now = rospy.Time.now()
    tfBroadcaster = tf.TransformBroadcaster()
    response = LocaliseFromAFeatureResponse()
    print "----------------------------------------------"
    print "localise_from_a_feature_callback: "
    camera_tag_frame_id = '%s_%s%s' % (req.visualFeature.pose.header.frame_id, algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)
    camera_frame_id = req.visualFeature.pose.header.frame_id
    camera_body_frame_id = req.visualFeature.pose.header.frame_id + 'zy'
    corrected_tag_frame_from_camera = camera_tag_frame_id + 'yx'
    # wanted : the camera_body_frame_id from the corrected_tag_frame_from_camera
    # feature_tf_frame_id = '%s_%s%s' % (req.visualFeature.pose.header.frame_id, algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)
    # print "localise_from_a_feature_callback: feature_tf_frame_id = %s"%feature_tf_frame_id

    listener = tf.TransformListener()

    # from  http://wiki.ros.org/tf/TfUsingPython
    # if listener.frameExists(feature_tf_frame_id) and listener.frameExists('/map'):
      #  time_ = listener.getLatestCommonTime('/map',feature_tf_frame_id)
       # position_, quaternion_ = listener.lookupTransform('/map', feature_tf_frame_id, time_)
    #try:
    # listener.waitForTransform(feature_tf_frame_id, '/map', rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform(camera_tag_frame_id + 'yx', req.visualFeature.pose.header.frame_id + 'zy', rospy.Time(), rospy.Duration(4.0))
    position_, quaternion_ = listener.lookupTransform(camera_tag_frame_id + 'yx'  , req.visualFeature.pose.header.frame_id + 'zy' ,  rospy.Time())
    #print feature_tf_frame_id, ' to ' , '/map = ', position_, quaternion_


    tfBroadcaster.sendTransform(
        (position_[0],position_[1],position_[2]),
        (quaternion_[0],quaternion_[1],quaternion_[2],quaternion_[3]),
        time_now,
        'tag_frame_%s_to_cam_body_%s'%(camera_tag_frame_id + 'yx', req.visualFeature.pose.header.frame_id + 'zy'),     # map_to_c12_t55
        camera_tag_frame_id + 'yx')                                                      # from frame

    response.pose.position.x = position_[0]
    response.pose.position.y = position_[1]
    response.pose.position.z = position_[2]
    response.pose.orientation.x = quaternion_[0]
    response.pose.orientation.y = quaternion_[1]
    response.pose.orientation.z = quaternion_[2]
    response.pose.orientation.w = quaternion_[3]
    #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
     #   print 'got an exception'
    return response

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

    # create the robot-convention camera body frame, step 1
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, 0.7071, 0.0, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        req.cameraPose.header.frame_id + 'z' ,  # to
        req.cameraPose.header.frame_id )        # from

    # create the robot-convention camera body frame, step 2
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, 0.0, 0.7071, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        req.cameraPose.header.frame_id + 'zy' ,  # to
        req.cameraPose.header.frame_id + 'z' )        # from

    camera_tag_frame_id = '%s_%s%s' % (req.cameraPose.header.frame_id, algorithm_abreviations[req.visualFeature.algorithm], req.visualFeature.id)

    # publish the camera_pose-to-tag_pose tf  -  from the robot-convention camera body frame, to the tag frame
    tfBroadcaster.sendTransform(
        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        camera_tag_frame_id,                                # to   e.g. c1_t1 '/feature/%s/%s/pose' % algorithm, req.visualFeature.id   e.g. "/feature/t/1"     # TODO - remove hardcoding to base namespace
        req.cameraPose.header.frame_id + 'zy')              # from      '/cam/%s/pose' % req.cameraPose.header.frame_id e.g. "/cam/c_1/pose"    # TODO - remove hardcoding to base namespace

    # correct the tag axes, step 1
    #- 90 Y
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, -0.7071, 0.0, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        camera_tag_frame_id + 'y' ,  # to
        camera_tag_frame_id )        # from

    # correct the tag axes, step 2: the correct tag axes are now published as e.g. c5_t7yx
    #- 90 X
    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( -0.7071, 0, 0.0, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        camera_tag_frame_id + 'yx' ,  # to
        camera_tag_frame_id + 'y' )        # from




    for fixed_feature in fixed_features:
        tfBroadcaster.sendTransform(
            (fixed_feature.pose.position.x, fixed_feature.pose.position.y, fixed_feature.pose.position.z ),
            (fixed_feature.pose.orientation.x, fixed_feature.pose.orientation.y, fixed_feature.pose.orientation.z, fixed_feature.pose.orientation.w),
            time_now,
            '%s%s' % (algorithm_abreviations[fixed_feature.algorithm], fixed_feature.id),
            'map')


    response = DetectedFeatureResponse()
    response.acknowledgement="bob"
    return response


def detect_feature_server():
    rospy.init_node('detect_feature_server')
    detect_feature_server = rospy.Service('/androidvosopencvros/detected_feature', DetectedFeature, detect_feature_callback)
    print "Ready to receive detected features."
    localise_from_a_feature_server = rospy.Service('/androidvosopencvros/localise_from_a_feature', LocaliseFromAFeature, localise_from_a_feature_callback)
    print "Ready to receive localise from individual features."
    rospy.spin()


if __name__ == "__main__":
    detect_feature_server()
