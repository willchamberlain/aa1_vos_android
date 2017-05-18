#!/usr/bin/env python

import logging
import logging.handlers

from start_logger import start_logger

import roslib
import rospy
import math
import tf
import geometry_msgs.msg

logger = []



if __name__ == '__main__':
    rospy.init_node('tf_logger', log_level=rospy.INFO)  # see  http://wiki.ros.org/rospy/Overview/Logging , http://wiki.ros.org/Verbosity%20Levels
        
    logger = start_logger('file_logger','/mnt/nixbig/ownCloud/project_AA1__1_1/results/111_logs/logit.log')

    listener = tf.TransformListener()

    # # test: log some messages
    # for i in range(20):
    #     logger.info('i = %d' % i)

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
            rospy.loginfo('tf|datetime=|frame_id=%s|parent_id=%s|x=%d|y=%d|z=%d|qx=%d|qy=%d|qz=%d|qw=%d'%(tf.frame_id,tf.parent_id,trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            rospy.loginfo('tf|datetime=|frame_id=%s|parent_id=%s|x=%d|y=%d|z=%d|qx=%d|qy=%d|qz=%d|qw=%d'%(tf.frame_id,tf.parent_id,trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            rospy.loginfo('tf|datetime=|frame_id=%s|parent_id=%s|x=%d|y=%d|z=%d|qx=%d|qy=%d|qz=%d|qw=%d'%(tf.frame_id,tf.parent_id,trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
