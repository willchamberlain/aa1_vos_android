#!/usr/bin/env python

from vos_aa1.srv import DetectedFeature
from vos_aa1.srv import DetectedFeatureRequest
from vos_aa1.srv import DetectedFeatureResponse
from vos_aa1.msg import VisualFeature
import rospy
import tf

def detect_feature_callback(req):
    print "----------------------------------------------"
    print "detect_feature_callback: "
    print "  camera frame_id [%s] :"%(req.cameraPose.header.frame_id)
    print "  camera translation from map [%.4f,%.4f,%.4f] : "%(req.cameraPose.pose.position.x,req.cameraPose.pose.position.y,req.cameraPose.pose.position.z)
    print "  camera orientation (quaternion) [%.4f,%.4f,%.4f,%.4f] : "%(req.cameraPose.pose.orientation.x,req.cameraPose.pose.orientation.y,req.cameraPose.pose.orientation.z,req.cameraPose.pose.orientation.w)
    print "  feature algorithm [%s] : "%(req.visualFeature.algorithm)
    print "  feature id [%d] : "%(req.visualFeature.id)
    print '  feature frame_id : %s_%s%s' % (req.cameraPose.header.frame_id, req.visualFeature.algorithm, req.visualFeature.id)
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


    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, 0.7071, 0.0, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        req.cameraPose.header.frame_id + 'z' ,  # to
        req.cameraPose.header.frame_id )        # from

    tfBroadcaster.sendTransform(
        (0.0, 0.0, 0.0),
        ( 0.0, 0.0, 0.7071, 0.7071 ), #  http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
        time_now,
        req.cameraPose.header.frame_id + 'zy' ,  # to
        req.cameraPose.header.frame_id + 'z' )        # from

    # publish the camera_pose-to-tag_pose tf
    tfBroadcaster.sendTransform(
        (req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        '%s_%s%s' % (req.cameraPose.header.frame_id, req.visualFeature.algorithm, req.visualFeature.id),  # to   e.g. c1_t1 '/feature/%s/%s/pose' % algorithm, req.visualFeature.id   e.g. "/feature/t/1"     # TODO - remove hardcoding to base namespace
        req.cameraPose.header.frame_id + 'zy')            # from      '/cam/%s/pose' % req.cameraPose.header.frame_id e.g. "/cam/c_1/pose"    # TODO - remove hardcoding to base namespace

    response = DetectedFeatureResponse()
    response.acknowledgement="bob"
    return response


def detect_feature_server():
    rospy.init_node('detect_feature_server')
    s = rospy.Service('/androidvosopencvros/detected_feature', DetectedFeature, detect_feature_callback)
    print "Ready to receive detected features. c"
    rospy.spin()


if __name__ == "__main__":
    detect_feature_server()
