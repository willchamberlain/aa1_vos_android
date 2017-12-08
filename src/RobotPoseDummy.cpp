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
    
       
      ros::Rate rate( 1.0 / 4.0 );                         // b/a Hz 
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
        
        sprintf(str_buffer,"time = %ds %dns: position= %f, %f, %f",time_now.sec, time_now.nsec, pose_origin.getX(), pose_origin.getY(), pose_origin.getZ() );
        std::cout << str_buffer << "\n";
        
        
          ros::ServiceClient client = node.serviceClient<vos_aa1::GetTf>("/look_up_transform");
          vos_aa1::GetTf service_;
          service_.request.target_frame_id.data = "map";
          service_.request.source_frame_id.data = "/pose_that_keeps_on_moving_along_y_axis";
          
          service_.request.time.data            = time_to_get.fromSec(time_now.sec-5);
          if ( client.call(service_) ) {
            std::cout << "SERVICE CALL SUCCEEDED" << "\n";
          } else {
            std::cout << "SERVICE CALL FAILED" << "\n";
          }          
          service_.request.time.data            = time_to_get.fromSec(time_now.sec-4);
          if ( client.call(service_) ) {
            std::cout << "SERVICE CALL SUCCEEDED" << "\n";
          } else {
            std::cout << "SERVICE CALL FAILED" << "\n";
          }          
          service_.request.time.data            = time_to_get.fromSec(time_now.sec-3);
          if ( client.call(service_) ) {
            std::cout << "SERVICE CALL SUCCEEDED" << "\n";
          } else {
            std::cout << "SERVICE CALL FAILED" << "\n";
          }
          service_.request.time.data            = time_to_get.fromSec(time_now.sec-2);
          if ( client.call(service_) ) {
            std::cout << "SERVICE CALL SUCCEEDED" << "\n";
          } else {
            std::cout << "SERVICE CALL FAILED" << "\n";
          }
          service_.request.time.data            = time_to_get.fromSec(time_now.sec-1);
          if ( client.call(service_) ) {
            std::cout << "SERVICE CALL SUCCEEDED" << "\n";
          } else {
            std::cout << "SERVICE CALL FAILED" << "\n";
          }
          service_.request.time.data            = time_to_get.fromSec(time_now.sec-0);
          if ( client.call(service_) ) {
            std::cout << "SERVICE CALL SUCCEEDED" << "\n";
          } else {
            std::cout << "SERVICE CALL FAILED" << "\n";
          }
        
        
        i_++;
        
        rate.sleep();
        
      }
      
    return 0;
    
    };
