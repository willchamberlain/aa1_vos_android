    #include <ros/ros.h>
    
    #include <string>
    
    #include <geometry_msgs/Twist.h>
    #include <geometry_msgs/Point.h>
    #include <geometry_msgs/PointStamped.h>
    #include <geometry_msgs/PoseStamped.h>
    
    #include <tf/transform_listener.h>
    #include <tf/transform_broadcaster.h>
    #include <tf/LinearMath/Vector3.h>
    
    
    int main(int argc, char** argv){ ros::init(argc, argv, "pose_repeater_service");
    
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
      
      ros::Time time_1;
      time_1 = ros::Time::now();
      tfBroadcaster.sendTransform(tf::StampedTransform(transform, time_1, "/map", "/pose_at_current_time"));
      tfBroadcaster.sendTransform(tf::StampedTransform(transform, time_1, "/map", "/pose_at_time_1"));
        
      ros::Duration(1.0).sleep();
      ros::Time time_2 = ros::Time::now();
      
        transform.setOrigin( tf::Vector3(5.0, 5.0, 0.0) );
        q = tf::Quaternion(0.0, 0.0, 0.707106781, 0.707106781);
        transform.setRotation(q);
      ros::Duration(1.0).sleep();
    //  ros::Time time_3(1000000,2000000);
      ros::Time time_3;
      time_3 = ros::Time::now();
      tfBroadcaster.sendTransform(tf::StampedTransform(transform, time_3, "/map", "/pose_at_current_time"));
      tfBroadcaster.sendTransform(tf::StampedTransform(transform, time_3, "/map", "/pose_at_time_2"));
    
    
      ros::Rate rate(10.0);                         // 10Hz 
      int i_ = 0;
      while (node.ok()){
        transform.setOrigin( tf::Vector3(0.0, 2.0+(0.01*i_), 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1.0) );
        tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "/pose_that_keeps_on_moving_along_y_axis"));
        i_++;
        
        for(int j_=0; j_<=10; j_++) {               // 10 iterations in 1 second
            tf::StampedTransform transformFound;
        
            ros::Time time_;
            time_ = time_2;
            uint64_t time_2_ns = time_2.toNSec()+(j_*100*1000*1000); // 10 iterations in 1 second = 100ms = 100*1,000,000ns
            
            time_.fromNSec(time_2_ns);
                try{
                  tfListener.waitForTransform("/pose_at_current_time", "/map", time_, ros::Duration(1.0)); //get transform with interpolation
                  tfListener.lookupTransform( "/pose_at_current_time", "/map", time_, transformFound);
                  tfBroadcaster.sendTransform(tf::StampedTransform(transformFound, ros::Time::now(), "map", "transform_from__pose_at_current_time__to__map"));        
                }
                catch (tf::TransformException &ex) {
                  ROS_ERROR("%s",ex.what());
                  ros::Duration(1.0).sleep();
                  continue;
                }
                try{
                  tfListener.waitForTransform("/map", "/pose_at_current_time", time_, ros::Duration(1.0)); //get inverse transform with the same interpolation
                  tfListener.lookupTransform( "/map", "/pose_at_current_time", time_, transformFound);
                  tfBroadcaster.sendTransform(tf::StampedTransform(transformFound, ros::Time::now(), "map", "transform_from__map__to__pose_at_current_time"));        
                }
                catch (tf::TransformException &ex) {
                  ROS_ERROR("%s",ex.what());
                  ros::Duration(1.0).sleep();
                  continue;
                }
            rate.sleep();
        }
        
        rate.sleep();
        
      }
      
    return 0;
    
    };
