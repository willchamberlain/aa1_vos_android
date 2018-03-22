#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# Pick up each frame and repeat it after a delay - expected_update_rate isn't being taken into account for some reason.
#    /mnt/nixbig/data/project_AA1_backup_pioneer2/2018_01_06_0012_vrep/p2os_launch/launch/costmap_common_params.yaml

# 2018_03_22_22:00
# Switch to fast range sensor
#  1) topic name 
#       /vrep/laser_scan_Hokuyo     -->  /vrep/laser_scan_Hokuyo_fast
#  2) tf frame_id in the LaserScan message
#       frame_id: /Hokuyo_URG_04LX_UG01_ROS   -->  frame_id: /fastHokuyo

class Odometry_frame_adapter_1:
    def __init__(self):
        self.sub  = rospy.Subscriber("/vrep/base_pose_ground_truth_vrep", Odometry, self.odom_callback)
        self.pub  = rospy.Publisher( "/vrep/base_pose_ground_truth", Odometry, queue_size=10)
        self.calls_per_second = 10                     # 10hz  
        self.future_dating_duration = rospy.Duration(0.0) #rospy.Duration(1.0/self.calls_per_second)                     # 10hz  
        self.rate = rospy.Rate(self.calls_per_second)  # 10hz

    def publish_odom_now(self, odom_msg):
        repeated_odom = odom_msg  
        repeated_odom.header.stamp = rospy.get_rostime() + self.future_dating_duration   
        self.pub.publish(repeated_odom)

    def odom_callback(self, odom_msg):
        rospy.loginfo(rospy.get_caller_id() + ": laser_odom_frame_adapter.py:  odom_callback:  I heard %s", odom_msg.header.frame_id)
        self.odom_in = odom_msg
        for ii_ in range(1,self.calls_per_second):
            self.publish_odom_now(odom_msg)
            self.rate.sleep()

class Odometry_frame_adapter:
    def __init__(self):
        self.sub  = rospy.Subscriber("/vrep/base_pose_ground_truth", Odometry, self.odom_callback)
        self.pub  = rospy.Publisher( "/vrep/base_pose_ground_truth_vrep", Odometry, queue_size=10)
        self.calls_per_second = 4                     # 4hz  
        self.rate = rospy.Rate(self.calls_per_second) # 4hz
        self.odom_in = None

    def publish_odom_now(self):
        if self.odom_in is not None:
            repeated_odom = self.odom_in  
            repeated_odom.header.stamp = rospy.get_rostime()     
            self.pub.publish(repeated_odom)

    def odom_callback(self, odom_msg):
        rospy.loginfo(rospy.get_caller_id() + ": repeater_odometry.py:  odom_callback:  I heard %s", odom_msg.header.frame_id)
        self.odom_in = odom_msg

    def republish(self):
        publish_odom_now()        


    
def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    odometry_frame_adapter = Odometry_frame_adapter_1()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


    # odometry_frame_adapter = Odometry_frame_adapter()
    # rate_ = rospy.Rate(10)
    # should_continue=True
    # while should_continue:
    #     rospy.spinOnce()
    #     odometry_frame_adapter.republish()
    #     rate_.sleep()    

if __name__ == '__main__':
    main()

 