#include <ros/ros.h>

#include <string>
#include <cstdio>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Vector3.h>


#include "vos_aa1/GetTf.h"

#include <string>
#include <cstdio>


tf::TransformListener    * tfListener = NULL; 
tf::TransformBroadcaster * tfBroadcaster = NULL;
int lookup_num = 0;


bool look_up_transform (
    vos_aa1::GetTf::Request &req,   
    vos_aa1::GetTf::Response &res) {
        
    std::cout << "look_up_transform: start" << "\n";
    std::cout.flush();
    
    //    tf::TransformListener listener;
    tf::StampedTransform transform_found_;
    std::string source_frame_id_str = req.source_frame_id.data;
    std::string target_frame_id_str = req.target_frame_id.data;
    ros::Time time_                 = req.time.data;
            try{
              tfListener->waitForTransform(
                source_frame_id_str, target_frame_id_str, time_, 
                ros::Duration(1.0));
              tfListener->lookupTransform(
                source_frame_id_str, target_frame_id_str, time_, 
                transform_found_);
            }
            catch (tf::TransformException &ex) {
              std::cout << ex.what() << "\n";
              ROS_ERROR("%s",ex.what());
              return false;
            }
    tf::transformStampedTFToMsg( transform_found_ , res.transform_found );  // http://docs.ros.org/indigo/api/tf/html/c++/transform__datatypes_8h.html
    
    lookup_num++;   char target_topic_name [50];    sprintf(target_topic_name,"/look_up_transform_%d",lookup_num);
    tfBroadcaster->sendTransform(tf::StampedTransform( transform_found_ , time_ , "/map" , target_topic_name) );
    
//    res.transform_found = transform_found_;        
    
/*            
    //geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Quaternion  quat;
    tf::Quaternion             tfQuat; // tf library object for quaternion
    tfQuat = transform_found_.getRotation(); // member fnc to extract the quaternion from a transform
    quat.x = tfQuat.x(); // copy the data from tf-style quaternion to geometry_msgs-style quaternion
    quat.y = tfQuat.y();
    quat.z = tfQuat.z();
    quat.w = tfQuat.w();  
    res.pose_found.pose.orientation = quat; //set the orientation of our PoseStamped object from result
*/    
    //res.pose = pose_stamped;
    return true;
}
  

int main(int argc, char** argv){ 

    ros::init(argc, argv, "pose_repeater_service");
    
    tfListener = new tf::TransformListener(ros::Duration(100));
    tfBroadcaster = new tf::TransformBroadcaster;

    ros::NodeHandle node;

//  http://wiki.ros.org/tf/Overview/Broadcasting%20Transforms
//  http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
//  void sendTransform(const StampedTransform & transform);
//  void sendTransform(const geometry_msgs::TransformStamped & transform);

  
  
  ros::ServiceServer service = node.advertiseService("look_up_transform", look_up_transform);
  ROS_INFO("Ready to look_up_transform.");
  
  ros::spin();
  
  /*
  ros::Rate rate(100.0);
  while (node.ok()){
    //  ROS_INFO("Ready to look_up_transform.");
    rate.sleep();
  }
  */
  
return 0;

};
