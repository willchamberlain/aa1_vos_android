#!/usr/bin/env python

import sys
import rospy
import tf
from geometry_msgs.msg import Point32, Polygon, PolygonStamped


class FieldOfView() :
    
    def __init__(self) :
        self.x_low  = rospy.get_param('~x_low',  0.0)
        self.x_high = rospy.get_param('~x_high', 1.0)
        self.y_low  = rospy.get_param('~y_low',  0.0)
        self.y_high = rospy.get_param('~y_high', 1.0)
        
    def big(self) :
        
        self.x_low  = -2.0
        self.x_high =  3.0
        self.y_low  = -2.0
        self.y_high =  3.0    
        
        
class FieldOfViewDisplayNode() :     
    
    
    def __init__(self, topic_name_, parent_frame_) :   
        self.topic_name   = topic_name_
        self.parent_frame = parent_frame_
        self.publisher    = rospy.Publisher(topic_name_, PolygonStamped, queue_size=2, latch=True)
        self.rate         = rospy.get_param('~rate', 1.0)
        self.fields_of_view = []
        
    def run_ros(self) :
        while not rospy.is_shutdown():
            if len(self.fields_of_view) > 0 :
                print 'len(self.fields_of_view) > 0'
                for fov_ in self.fields_of_view :
                    self.paint(fov_, self.parent_frame, self.publisher)
            else :
                print 'len(self.fields_of_view) <= 0'
            # Sleep for a while before publishing new messages. Division is so rate != period.
            if self.rate:
                rospy.sleep(1/self.rate)
            else:
                rospy.sleep(1.0)
        
        
    def fov(self, fieldOfView_) :  
        self.fields_of_view.append(fieldOfView_)
    
    
    def paint(self, fieldOfView_, parent_frame_, publisher_) :  
        poly = PolygonStamped()
        poly.header.frame_id=parent_frame_
        poly.header.stamp = rospy.Time.now()
        poly.polygon.points = [Point32() for i in range(4)]
        poly.polygon.points[0].x = fieldOfView_.x_low
        poly.polygon.points[0].y = fieldOfView_.y_low
        poly.polygon.points[0].z = 1.0
        poly.polygon.points[1].x = fieldOfView_.x_low
        poly.polygon.points[1].y = fieldOfView_.y_high
        poly.polygon.points[1].z = 1.0
        poly.polygon.points[2].x = fieldOfView_.x_high
        poly.polygon.points[2].y = fieldOfView_.y_high
        poly.polygon.points[2].z = 1.0
        poly.polygon.points[3].x = fieldOfView_.x_high
        poly.polygon.points[3].y = fieldOfView_.y_low
        poly.polygon.points[3].z = 1.0
        publisher_.publish(poly)
        
    
      

if __name__ == '__main__' :
    
    rospy.init_node('field_of_view_display')
    try:
        fov            = FieldOfView()
        fov.big()
        fovDisplayNode = FieldOfViewDisplayNode('field_of_view_display_node','/map')
        fovDisplayNode.fov(fov)
        fovDisplayNode.run_ros()
    except rospy.ROSInterruptException: pass
    
