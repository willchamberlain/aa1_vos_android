#!/usr/bin/env python

# - see https://docs.ros.org/indigo/api/fake_localization/html/static__odom__broadcaster_8py_source.html
# - see   https://docs.ros.org/jade/api/fake_localization/html/static__odom__broadcaster_8py_source.html
# Similar to static_transform_broadcaster, this node constantly publishes
# static odometry information (Odometry msg and tf). This can be used
# with fake_localization to evaluate planning algorithms without running
# an actual robot with odometry or localization
#
# Author: Armin Hornung
# License: BSD
 
import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point, PoseWithCovariance, PoseWithCovarianceStamped, Twist, TwistWithCovariance, Vector3
# from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Twist, TwistWithCovariance  # https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf


initialpose_poseWCS_publisher = 1
fakelocalisation_poseWCS_publisher = 1

def publishOdom():
	rospy.init_node('fake_odom')
	base_frame_id = rospy.get_param("~base_frame_id", "base_link")
	odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
	publish_frequency = rospy.get_param("~publish_frequency", 10.0)
	pub = rospy.Publisher('odom', Odometry)
	tf_pub = tf.TransformBroadcaster()

	#TODO: static pose could be made configurable (cmd.line or parameters)
	quat = tf.transformations.quaternion_from_euler(0, 0, 0)

	odom = Odometry()
	odom.header.frame_id = odom_frame_id
	odom.pose.pose = Pose(Point(0, 0, 0), Quaternion(*quat))

	rospy.loginfo("Publishing static odometry from \"%s\" to \"%s\"", odom_frame_id, base_frame_id)
	r = rospy.Rate(publish_frequency)
	while not rospy.is_shutdown():
		odom.header.stamp = rospy.Time.now()
		pub.publish(odom)
		tf_pub.sendTransform((0, 0, 0), quat,
		                    odom.header.stamp, base_frame_id, odom_frame_id)
		r.sleep()


def publishPose():  
	covariance_ = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
	x = 0 ; y = 0 ; z = 0 ;    qx = 0 ; qy = 0 ; qz = 0 ; qw = 1
	time_now = rospy.Time.now()
	id_ = '/map'

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


def doMain():
	rospy.init_node('fake_odom')

	publish_frequency = rospy.get_param("~publish_frequency", 10.0)	
	r = rospy.Rate(publish_frequency)

	global initialpose_poseWCS_publisher
	initialpose_poseWCS_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=50, latch=True)  # latch to make sure that AMCL has an intitial pose to use
	print "Ready to publish poses with covariance"
	rospy.loginfo("Ready to publish poses with covariance")
	global fakelocalisation_poseWCS_publisher  # fake_localisation / fake_localization
	fakelocalisation_poseWCS_publisher = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=50, latch=True)  # latch to make sure that AMCL has an intitial pose to use
	print "Ready to publish /base_pose_ground_truth for fake_localization"
	rospy.loginfo( "Ready to publish /base_pose_ground_truth for fake_localization" )

	while not rospy.is_shutdown():
		publishPose()
		r.sleep()


		
if __name__ == '__main__':
	doMain()

