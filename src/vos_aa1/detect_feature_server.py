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
    print "  received feature:  frame_id [%s] :"%(req.cameraPose.header.frame_id)
    print "  algorithm [%s] : "%(req.visualFeature.algorithm)
    print "  id [%d] : "%(req.visualFeature.id)
    print "  translation [%.4f,%.4f,%.4f] : "%(req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z)
    print "  orientation (quaternion) [%.4f,%.4f,%.4f,%.4f] "%(req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w)


    # TODO - something about this conversion is not right: the translation and quaternion match those found by AprilTags_Kaess and sent by DetectedFeatureClient, but the RPY are different
    euler = tf.transformations.euler_from_quaternion([req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w])
    print "  feature x_y_z_w roll=%.4f pitch=%.4f yaw=%.4f"%(euler[0], euler[1], euler[2])

    tfBroadcaster = tf.TransformBroadcaster()
    time_now = rospy.Time.now()

    # publish the map-to-camera_pose
    tfBroadcaster.sendTransform((req.cameraPose.pose.position.x, req.cameraPose.pose.position.y, 1), # TODO - hardcoding to 1m up to display in RViz
        tf.transformations.quaternion_from_euler(0, 0, 0), # TODO - hardcoding to the map origin
        time_now,
        req.cameraPose.header.frame_id,
        "map")

    # publish the camera_pose-to-tag_pose
    tfBroadcaster.sendTransform((req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z),
        (req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w),
        time_now,
        req.cameraPose.header.frame_id + "_" + str(req.visualFeature.id),
        req.cameraPose.header.frame_id)


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
