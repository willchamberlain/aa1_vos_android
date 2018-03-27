#!/usr/bin/env python


# from Rabe (https://answers.ros.org/users/15571/rabe/) on this post https://answers.ros.org/question/193512/rebroadcast-tf-transforms-with-different-frame_id-on-the-fly/?answer=193526#post-id-193526
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node("transform_inverter")
    rospy.sleep(0.5)

    child_frame         = rospy.get_param("~child_frame")
    new_child_frame     = rospy.get_param("~new_child_frame")
    parent_frame        = rospy.get_param("~parent_frame")
    new_parent_name     = rospy.get_param("~new_parent_frame")

    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(100)

    print 'child_frame=%s'%child_frame
    print 'new_child_frame=%s'%new_child_frame
    print 'parent_frame=%s'%parent_frame
    print 'new_parent_name=%s'%new_parent_name

    while not rospy.is_shutdown():
            print 'Use tf_rebroadcast_2 instead'
            continue
        #try:
            listener.waitForTransform(               parent_frame,  child_frame,  rospy.Time(0),  rospy.Duration(5.0))
            print ' -- listener.waitForTransform completed -- '
            (trans, rot) = listener.lookupTransform( parent_frame,  child_frame,  rospy.Time(0))
            print ' -- listener.lookupTransform completed -- '
            print trans
            # sendTransform( translation , rotation , timestamp , to/child , from/parent  )  -  https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
            #  
            #  C++ :  " Note: sendTransform and StampedTransform have opposite ordering of parent and child. " - https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20(C++)
            #       sendTransform  (  
            #           tf::StampedTransform(transform, ros::Time::now(), from/parent , to/child )   )
            broadcaster.sendTransform(trans,rot, rospy.Time.now(), new_child_frame, new_parent_name)
            print ' -- broadcaster.sendTransform completed -- '
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
        #    rospy.loginfo("Couldn't find a transform")
        #    continue


    #   rosrun vos_aa1 tf_rebroadcast.py _child_frame:=laser _new_child_frame:=laser _parent_frame:=Pioneer_p3dx _new_parent_frame:=base_link 