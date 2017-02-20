#!/usr/bin/env python

from vos_aa1.srv import DetectedFeature
from vos_aa1.srv import DetectedFeatureRequest
from vos_aa1.srv import DetectedFeatureResponse
from vos_aa1.msg import VisualFeature
import rospy

def detect_feature_callback(req):
    print "detect_feature_callback: "
    print "  received feature:  frame_id [%s] :"%(req.cameraPose.header.frame_id)
    print "  algorithm [%s] : "%(req.visualFeature.algorithm)
    print "  id [%d] : "%(req.visualFeature.id)
    print "  translation [%.4f,%.4f,%.4f] : "%(req.visualFeature.pose.pose.position.x, req.visualFeature.pose.pose.position.y, req.visualFeature.pose.pose.position.z)
    print "  orientation (quaternion) [%.4f,%.4f,%.4f,%.4f] "%(req.visualFeature.pose.pose.orientation.x, req.visualFeature.pose.pose.orientation.y, req.visualFeature.pose.pose.orientation.z, req.visualFeature.pose.pose.orientation.w)
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
