#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <math.h>
using namespace std;

ros::Publisher filtered_scan_pub;
ros::Subscriber input_scan_sub;
sensor_msgs::LaserScan filtered_scan;
float UPPER_CUTOFF = 4.0;

bool my_update(const sensor_msgs::LaserScan& input_scan)
{
//    std::stringstream ss; 
//    ss.str(""); ss<<"my_update: "; ROS_INFO_STREAM(ss.str());
  
  filtered_scan.ranges.resize(input_scan.ranges.size());
//    ss.str(""); ss<<"my_update: first resize: input_scan.ranges.size() = "<<input_scan.ranges.size()<<"filtered_scan.ranges.size() = "<<filtered_scan.ranges.size(); ROS_INFO_STREAM(ss.str());
  filtered_scan.intensities.resize(input_scan.intensities.size());
//    ss.str(""); ss<<"my_update: second resize: input_scan.intensities.size()="<<input_scan.intensities.size()<<"filtered_scan.intensities.size()="<<filtered_scan.intensities.size(); ROS_INFO_STREAM(ss.str());

  unsigned int n = input_scan.ranges.size();
//    ss.str(""); ss<<"my_update: n = "<<n; ROS_INFO_STREAM(ss.str());
  

  //loop through the scan and reassign range values 
  for (unsigned int i = 0; i < n; i++){
    if(input_scan.ranges[i] > UPPER_CUTOFF ||
       isnan(input_scan.ranges[i]) ||
       isinf(input_scan.ranges[i])) {
      filtered_scan.ranges[i] = UPPER_CUTOFF;
    }
    else {
      filtered_scan.ranges[i] = input_scan.ranges[i];
    }
    // filtered_scan.ranges[i] = 1.5; // fixed-radius circle, to test that clients are receiving
  }
//    ss.str(""); ss<<"my_update: after scan ranges value substitution"; ROS_INFO_STREAM(ss.str());
    
    
  n = input_scan.intensities.size();
//    ss.str(""); ss<<"my_update: n = "<<n; ROS_INFO_STREAM(ss.str());
  for (unsigned int i = 0; i < n; i++){
      filtered_scan.intensities[i] = input_scan.intensities[i];
  }
//    ss.str(""); ss<<"my_update: after scan intensities value substitution"; ROS_INFO_STREAM(ss.str());
 
  //make sure to set all the needed fields on the filtered scan
  filtered_scan.header.frame_id = input_scan.header.frame_id;
  filtered_scan.header.stamp = input_scan.header.stamp;
  filtered_scan.angle_min = input_scan.angle_min;
  filtered_scan.angle_max = input_scan.angle_max;
  filtered_scan.angle_increment = input_scan.angle_increment;
  filtered_scan.time_increment = input_scan.time_increment;
  filtered_scan.scan_time = input_scan.scan_time;
  filtered_scan.range_min = input_scan.range_min;
  filtered_scan.range_max = input_scan.range_max;
//    ss.str(""); ss<<"my_update: after setting attributes"; ROS_INFO_STREAM(ss.str());
 
  return true;
}

void input_scan_sub_callback(const sensor_msgs::LaserScan& input_scan)
{
//    std::stringstream ss; 
//    ss.str(""); ss<<"input_scan_sub_callback: "; ROS_INFO_STREAM(ss.str());
  my_update(input_scan);
  filtered_scan_pub.publish(filtered_scan);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "mybot_laser_obstacle_clearing_filter");
//    std::stringstream ss; 
//    ss.str(""); ss<<"starting: "; ROS_INFO_STREAM(ss.str());
  ros::NodeHandle n1, n2;
  input_scan_sub = n1.subscribe("scan_hokuyo", 0, input_scan_sub_callback);
  filtered_scan_pub = n2.advertise<sensor_msgs::LaserScan>("scan_filtered_clearer", 50);
  ros::Rate loop_rate(10);
  
//    ss.str(""); ss<<"starting: "; ROS_INFO_STREAM(ss.str());

//  ros::spin();
//  return 0;
  
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
}
