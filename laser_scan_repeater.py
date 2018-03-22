#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

# Pick up each frame and repeat it after a delay - expected_update_rate isn't being taken into account for some reason.
#    /mnt/nixbig/data/project_AA1_backup_pioneer2/2018_01_06_0012_vrep/p2os_launch/launch/costmap_common_params.yaml

# 2018_03_22_22:00
# Switch to fast range sensor
#  1) topic name 
#       /vrep/laser_scan_Hokuyo                 -->  /vrep/laser_scan_Hokuyo_fast
#       /vrep/laser_scan                        -->  /vrep/laser_scan_Hokuyo_fast
#  2) tf frame_id in the LaserScan message
#       frame_id: /Hokuyo_URG_04LX_UG01_ROS     -->  frame_id: /fastHokuyo

class Laser_scan_frame_adapter_1:
    def __init__(self):
        self.pub  = rospy.Publisher("/vrep/laser_scan_repeated", LaserScan, queue_size=10)
        self.sub  = rospy.Subscriber("/vrep/laser_scan_Hokuyo_fast", LaserScan, self.laser_scan_callback)
        self.calls_per_second = 4                     # 4hz  
        self.rate = rospy.Rate(self.calls_per_second) # 4hz

    def publish_scan_now(self, scan):
        repeated_scan = scan  
        repeated_scan.header.stamp = rospy.get_rostime()     
        self.pub.publish(repeated_scan)

    def laser_scan_callback(self, scan):
        rospy.loginfo(rospy.get_caller_id() + ": laser_scan_frame_adapter.py:  laser_scan_callback:  I heard %s", scan.header.frame_id)
        self.scan_in = scan
        for ii_ in range(1,self.calls_per_second):
            self.publish_scan_now(scan)
            self.rate.sleep()

class Laser_scan_frame_adapter:
    def __init__(self):
        self.pub  = rospy.Publisher("/vrep/laser_scan_repeated", LaserScan, queue_size=10)
        self.sub  = rospy.Subscriber("/vrep/laser_scan_Hokuyo_fast", LaserScan, self.laser_scan_callback)
        self.calls_per_second = 4                     # 4hz  
        self.rate = rospy.Rate(self.calls_per_second) # 4hz
        self.scan_in = None

    def publish_scan_now(self):
        if self.scan_in is not None:
            repeated_scan = self.scan_in  
            repeated_scan.header.stamp = rospy.get_rostime()     
            self.pub.publish(repeated_scan)

    def laser_scan_callback(self, scan):
        rospy.loginfo(rospy.get_caller_id() + ": laser_scan_frame_adapter.py:  laser_scan_callback:  I heard %s", scan.header.frame_id)
        self.scan_in = scan

    def republish(self):
        publish_scan_now()        


    
def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    laser_scan_frame_adapter = Laser_scan_frame_adapter_1()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


    # laser_scan_frame_adapter = Laser_scan_frame_adapter()
    # rate_ = rospy.Rate(10)
    # should_continue=True
    # while should_continue:
    #     rospy.spinOnce()
    #     laser_scan_frame_adapter.republish()
    #     rate_.sleep()    

if __name__ == '__main__':
    main()

 