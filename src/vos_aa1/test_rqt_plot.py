#!/usr/bin/env python

import rospy
import numpy as np
import time
import random
from std_msgs.msg import Duration, Time, Int32, Float64

def produce_random_times():
    rospy.init_node('random_time_pairs')
    time_difference_publisher = rospy.Publisher('time_difference', Duration, queue_size=1)
    timeOne_publisher = rospy.Publisher('timeOne', Time, queue_size=1)
    timeTwo_publisher = rospy.Publisher('timeTwo', Time, queue_size=1)
    time_difference_secs = rospy.Publisher('time_difference_secs', Int32, queue_size=1)
    time_difference_nsecs = rospy.Publisher('time_difference_nsecs', Int32, queue_size=1)
    time_difference_float = rospy.Publisher('time_difference_float', Float64, queue_size=1)
    
    timeTwo = rospy.Time.now()
    while True:
        timeOne = rospy.Time.now()
        sleepTime = random.uniform(0.01, 0.1)
        time.sleep(sleepTime)
        timeTwo = rospy.Time.now()
        timeDifference = timeTwo - timeOne
        
        time_difference_publisher.publish(timeDifference)
        timeOne_publisher.publish(timeOne)
        timeTwo_publisher.publish(timeTwo)
        time_difference_secs.publish(timeDifference.secs)
        time_difference_nsecs.publish(timeDifference.nsecs)
        time_difference_float.publish(timeDifference.secs + 0.000000001*timeDifference.nsecs)
        time.sleep(0.1)


if __name__ == "__main__":
    produce_random_times()
