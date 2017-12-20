    #include <ros/ros.h>
    
    #include <string>
    #include <cstdio>
    #include <iostream>
    
    #include <geometry_msgs/Twist.h>
    #include <geometry_msgs/Point.h>
    #include <geometry_msgs/PointStamped.h>
    #include <geometry_msgs/PoseStamped.h>
    
    #include <tf/transform_listener.h>
    #include <tf/transform_broadcaster.h>
    #include <tf/LinearMath/Vector3.h>
    


#include "vos_aa1/GetTf.h"


/*
Test/demo TF transform interpolation and the VOS service <vos_aa1::GetTf>("/androidvosopencvros/look_up_transform"): 
    publishes a transform /map --> /pose_that_keeps_on_moving_along_y_axis  on a fixed rate, and
    calls <vos_aa1::GetTf>("/androidvosopencvros/look_up_transform") to look up that transform at intermmediate times 
*/
    
    int main(int argc, char** argv){ ros::init(argc, argv, "pose_1hz_dummy_that_keeps_on_moving_along_y_axis");
    
    ros::NodeHandle node;
    
    //  http://wiki.ros.org/tf/Overview/Broadcasting%20Transforms
    //  http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
    //  void sendTransform(const StampedTransform & transform);
    //  void sendTransform(const geometry_msgs::TransformStamped & transform);
    
      static tf::TransformListener    tfListener(ros::Duration(100));
      static tf::TransformBroadcaster tfBroadcaster;
      tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
      tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
        q.setRPY(0, 0, 0);
      transform.setRotation(q);
    
       
      ros::Rate rate( 1.0 );                         // b/a Hz 
      char str_buffer [100];
      ros::Time time_now;
      ros::Time time_to_get;
      tf::Vector3 pose_origin;
      int i_ = 0;
      while (node.ok()){
        pose_origin.setValue( 0.5+(0.5*i_) ,  0.5+(0.5*i_) ,  0.0 );
        transform.setOrigin( pose_origin );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1.0) );
        time_now = ros::Time::now();
        tfBroadcaster.sendTransform( tf::StampedTransform( transform , time_now , "map" , "/pose_that_keeps_on_moving_along_y_axis" ));
        
        sprintf(str_buffer,"time = %d = %ds %dns: position= %f, %f, %f: orientation= %f, %f, %f, %f", 
            time_now.toNSec()/(1000*1000), 
            time_now.sec, time_now.nsec, 
            pose_origin.getX(), pose_origin.getY(), pose_origin.getZ(),
            transform.getRotation.getX(),  transform.getRotation.getY(), transform.getRotation.getZ(), transform.getRotation.getW()
            );
        std::cout << str_buffer << "\n";        
        
        i_++;
        
        rate.sleep();
        
      }
      
    return 0;
    
    };
