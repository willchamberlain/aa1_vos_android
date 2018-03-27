#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <math.h>
using namespace std;

sensor_msgs::LaserScan laser_scan_msg;
ros::Publisher fake_laser_scan_pub;

//
float num_measurements = 40.0f;
int num_measurements_int = 40;    
float f_angle_min;
float f_angle_max;
float f_angle_increment;
float f_time_increment;
float f_scan_time;
float f_range_min;
float f_range_max;
float f_ranges[40];//[num_measurements_int]; // max of 30 measurements
float f_intensities[40];//[num_measurements_int];


float UPPER_CUTOFF = 4.0;  // see /mnt/nixbig/data/project_AA1_backup_pioneer2/2018_01_06_0012_vrep/mybot_catkin_ws/src/mybot/src/mybot_laser_obstacle_clearing_filter.cpp

// ros::Publisher pub_Laser("LaserData", &laser_scan_msg);
// ros::NodeHandle nh;


int main(int argc, char** argv){  
  ros::init(argc, argv, "fake_laser_scan");
  ros::NodeHandle nodeHandle;
  fake_laser_scan_pub = nodeHandle.advertise<sensor_msgs::LaserScan>("fake_laser_scan", 50);

  f_angle_min = -1.57f; // start angle of the scan [rad]
  f_angle_max =  1.57f; // end angle of the scan [rad]
  f_angle_increment = 3.141592653589793/num_measurements;  // 3.14/40   - 50 measurement points
  f_scan_time =  0.1f;  // time between scans [seconds]
  f_time_increment = f_scan_time/num_measurements;  // time between measurements of a scan [seconds]
  f_range_min =  0.1f; // minimum range value [m]
  f_range_max = 10.0f; // maximum range value [m]
  char str_buffer [100];
  //laser_scan_msg.ranges_length = num_measurements_int; // # range data [m] (Note: values < range_min or > range_max should be discarded)
  //laser_scan_msg.intensities_length = num_measurements_int; //# intensity data [device-specific units].  If your device does not provide intensities, please leave the array empty.

        std::cout << " 46 \n";   
  // create the test data
  for (int z = 0 ; z<num_measurements_int; z++)
  {
    f_ranges[z] = f_range_max-0.5;
    f_intensities[z] = 25.0f; // arbitrary //z*z;
  }

        std::cout << " 53 \n";   
  ros::Rate loop_rate(1.0/f_scan_time);
  while (nodeHandle.ok()){
    laser_scan_msg.ranges.resize(num_measurements_int);   laser_scan_msg.intensities.resize(num_measurements_int);
        std::cout << " 57 \n";   
    laser_scan_msg.header.stamp = ros::Time::now();
    laser_scan_msg.header.frame_id = "laser_frame";
    laser_scan_msg.angle_min = f_angle_min;
    laser_scan_msg.angle_max = f_angle_max;
    laser_scan_msg.angle_increment = f_angle_increment;
    laser_scan_msg.time_increment = f_time_increment;
    laser_scan_msg.scan_time = f_scan_time;
    laser_scan_msg.range_min = f_range_min;
    laser_scan_msg.range_max = f_range_max;
        std::cout << " 68 \n";   

    for (int z = 0 ; z<num_measurements_int; z++)
    {  // sprintf(str_buffer,"%d",z); std::cout << " z = "<< str_buffer << "\n";   sprintf(str_buffer,"%f",f_ranges[z]); std::cout << " f_ranges[z] = "<< str_buffer << "\n";
      laser_scan_msg.ranges[z] = f_ranges[z];
    }
        std::cout << " 74 \n";   

    for (int z = 0 ; z<num_measurements_int; z++)
    {
      laser_scan_msg.intensities[z] = f_intensities[z];
    }

        std::cout << " 81 \n";   
    fake_laser_scan_pub.publish(laser_scan_msg);


        std::cout << " 85 \n";   
    ros::spinOnce();
        std::cout << " 87 \n";   
    loop_rate.sleep();
        std::cout << " 89 \n";   
  }
  return 0;
}
