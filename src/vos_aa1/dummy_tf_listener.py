#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import tf2_ros
import geometry_msgs.msg
#import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('dummy_tf_listener')

    listener = tf.TransformListener()
    
#    tf2Buffer = tf2_ros.Buffer(rospy.Duration(10.0))
    tf2Buffer = tf2_ros.Buffer()

#    rospy.wait_for_service('spawn')
#    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
#    spawner(4, 2, 0, 'turtle2')

#    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print 'position: x=%f,y=%f,z=%f'%(trans[0],trans[1],trans[2])
                

#        angular = 4 * math.atan2(trans[1], trans[0])
#        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
#        cmd = geometry_msgs.msg.Twist()
#        cmd.linear.x = linear
#        cmd.angular.z = angular
#        turtle_vel.publish(cmd)



            
        transformStampedNow = tf2Buffer.lookup_transform('map', 'base_link', rospy.Time(0))
        print 'time = %s'%(transformStampedNow.header.stamp)
        print 'parent frame id = "%s"'%(transformStampedNow.header.frame_id)
        print 'child frame id = "%s"'%(transformStampedNow.child_frame_id)
        trans = transformStampedNow.transform.translation
        print 'position: x=%f,y=%f,z=%f'%(trans[0],trans[1],trans[2])
        
        

        rate.sleep()
