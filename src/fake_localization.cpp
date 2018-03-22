/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


/** \author Ioan Sucan */

/**
  @mainpage
  @htmlinclude manifest.html
  @b odom_localization Takes in ground truth pose information for a robot
  base (e.g., from a simulator or motion capture system) and republishes
  it as if a localization system were in use.
  <hr>
  @section usage Usage
  @verbatim
  $ fake_localization
  @endverbatim
  <hr>
  @section topic ROS topics
  Subscribes to (name/type):
  - @b [base_pose_ground_truth_topic_name] nav_msgs/Odometry : robot's odometric pose.  Only the position information is used (velocity is ignored).
  Publishes to (name / type):
  - @b "amcl_pose" geometry_msgs/PoseWithCovarianceStamped : robot's estimated pose in the map, with covariance
  - @b "particlecloud" geometry_msgs/PoseArray : fake set of poses being maintained by the filter (one paricle only).
  <hr>
  @section parameters ROS parameters
  - "~odom_frame_id" (string) : The odometry frame to be used, default: "odom"
 **/
/** 
Adapted from  https://github.com/ros-planning/navigation/blob/indigo-devel/fake_localization/fake_localization.cpp
*/
#include <ros/ros.h>
#include <ros/time.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <angles/angles.h>

#include "ros/console.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"


class FakeOdomNode
{
  public:
    FakeOdomNode(void)
    {
      m_posePub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,true);
      m_particlecloudPub = m_nh.advertise<geometry_msgs::PoseArray>("particlecloud",1,true);
      m_tfServer = new tf::TransformBroadcaster();	
      m_tfListener = new tf::TransformListener();

      m_base_pos_received = false;

      ros::NodeHandle private_nh("~");
      private_nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
      private_nh.param("base_frame_id", base_frame_id_, std::string("base_link")); 
      private_nh.param("global_frame_id_in", global_frame_id_in_, std::string("/map"));
      private_nh.param("global_frame_id_out", global_frame_id_out_, std::string("/map"));
      private_nh.param("delta_x", delta_x_, 0.0);
      private_nh.param("delta_y", delta_y_, 0.0);
      private_nh.param("delta_yaw", delta_yaw_, 0.0);      
      private_nh.param("transform_tolerance", transform_ts_pad4_tolerance_future_date_, 0.3);   
      private_nh.param("initialpose_topic_name", initialpose_topic_name, std::string("initialpose"));   
      private_nh.param("base_pose_ground_truth_topic_name", base_pose_ground_truth_topic_name, std::string("base_pose_ground_truth"));   
      ROS_INFO("parameters: %s %s %s %s , %f  %f %f , %f , %s %s\n", odom_frame_id_.c_str(), base_frame_id_.c_str(), global_frame_id_in_.c_str(), global_frame_id_out_.c_str() , delta_x_, delta_y_, delta_yaw_ , transform_ts_pad4_tolerance_future_date_ , initialpose_topic_name.c_str() , base_pose_ground_truth_topic_name.c_str() );


      m_particleCloud.header.stamp = ros::Time::now();
      m_particleCloud.header.frame_id = global_frame_id_out_;
      m_particleCloud.poses.resize(1);
      ros::NodeHandle nh;

      m_offsetTf = tf::Transform(tf::createQuaternionFromRPY(0, 0, -delta_yaw_ ), tf::Point(-delta_x_, -delta_y_, 0.0));

      base_pose_sub_ = nh.subscribe(base_pose_ground_truth_topic_name, 100, &FakeOdomNode::stuffFilter, this);

      odom_filter_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "", 100);                         //  passes messages from a ROS subscription through to the filters which have connected to it
      odom_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(*odom_filter_sub_, *m_tfListener, base_frame_id_, 100);  // target_frame   The frame this filter should attempt to transform to  //  implement a filter which only passes messages through once there is transform data available
      odom_filter_->registerCallback(boost::bind(&FakeOdomNode::update, this, _1));

      // subscription to "2D Pose Estimate" from RViz:
      m_initPoseSub = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh, initialpose_topic_name, 1);
      m_initPoseFilter = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*m_initPoseSub, *m_tfListener, global_frame_id_in_, 1);
      m_initPoseFilter->registerCallback(boost::bind(&FakeOdomNode::initPoseReceived, this, _1));
    }

    ~FakeOdomNode(void)
    {
      if (m_tfServer)
        delete m_tfServer; 
    }


  private:
    ros::NodeHandle                 m_nh;
    ros::Publisher                  m_posePub;
    ros::Publisher                  m_particlecloudPub;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseSub;
    tf::TransformBroadcaster        *m_tfServer;
    tf::TransformListener           *m_tfListener;
    tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseFilter;
    tf::MessageFilter<nav_msgs::Odometry>*           odom_filter_;
    ros::Subscriber                 base_pose_sub_; 
    message_filters::Subscriber<nav_msgs::Odometry>* odom_filter_sub_;

    double                         delta_x_, delta_y_, delta_yaw_;
    bool                           m_base_pos_received;
    double transform_ts_pad4_tolerance_future_date_;

    nav_msgs::Odometry                        m_basePosMsg;
    geometry_msgs::PoseArray                  m_particleCloud;
    geometry_msgs::PoseWithCovarianceStamped  m_currentPos;
    tf::Transform                             m_offsetTf;

    //parameter for what odom to use
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string global_frame_id_in_;
    std::string global_frame_id_out_;
    std::string initialpose_topic_name;
    std::string base_pose_ground_truth_topic_name;

  public:
    void stuffFilter(const nav_msgs::OdometryConstPtr& odom_msg){  // called from  base_pose_sub_
      //we have to do this to force the message filter to wait for transforms
      //from odom_frame_id_ to base_frame_id_ to be available at time odom_msg.header.stamp
      //really, the base_pose_ground_truth should come in with no frame_id b/c it doesn't make sense
      boost::shared_ptr<nav_msgs::Odometry> new_odom_msg(new nav_msgs::Odometry);
      ROS_INFO("stuffFilter: old   odom_msg->header.frame_id =  %s", odom_msg->header.frame_id.c_str());
      *new_odom_msg = *odom_msg; 
      new_odom_msg->header.frame_id = odom_frame_id_;
      ROS_INFO("stuffFilter: new_odom_msg->header.frame_id = odom_frame_id_ = %s", odom_frame_id_.c_str());
      odom_filter_->add(new_odom_msg);
    }

    void update(const nav_msgs::OdometryConstPtr& message){
      ROS_INFO("update: message->header.frame_id=%s", message->header.frame_id.c_str());
      tf::Pose current_pose_4tf;
      tf::Stamped<tf::Pose> odom_to_map;
      tf::poseMsgToTF(message->pose.pose, current_pose_4tf);
      current_pose_4tf = m_offsetTf * current_pose_4tf;
      try {
        m_tfListener->transformPose(odom_frame_id_, tf::Stamped<tf::Pose>(current_pose_4tf.inverse(), message->header.stamp, base_frame_id_), odom_to_map);
      } catch(tf::TransformException &e) {
        ROS_ERROR("Failed to transform to %s from %s: %s\n", odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
        return; 
      }
      m_tfServer->sendTransform(tf::StampedTransform(odom_to_map.inverse(),
                                                     message->header.stamp + ros::Duration(transform_ts_pad4_tolerance_future_date_),
                                                     global_frame_id_out_,        // frame_id
                                                     message->header.frame_id));  // child_frame_id
      ROS_INFO("update: after m_tfServer->sendTransform with global_frame_id_out_=%s , message->header.frame_id=%s " , global_frame_id_out_.c_str() , message->header.frame_id.c_str());

      // start: easier to multiply as a TF 
      tf::Pose current_pose;
      geometry_msgs::Pose current_pose_msg;
      tf::poseMsgToTF(message->pose.pose, current_pose);      
      current_pose = m_offsetTf * current_pose;   //also apply the offset to the pose
      tf::poseTFToMsg(current_pose, current_pose_msg);
      // end: easier to multiply as a TF 

      // Publish localized pose
      m_currentPos.header = message->header;
      m_currentPos.header.frame_id = global_frame_id_out_;
      m_currentPos.pose.pose = current_pose_msg;
      m_posePub.publish(m_currentPos);

      // The particle cloud is the current position. Quite convenient.
      m_particleCloud.header = m_currentPos.header;
      m_particleCloud.poses[0] = m_currentPos.pose.pose;
      m_particlecloudPub.publish(m_particleCloud);
    }

    void initPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
      tf::Pose pose;
      tf::poseMsgToTF(msg->pose.pose, pose);

      if (msg->header.frame_id != global_frame_id_out_){
        ROS_WARN("Frame ID of \"%s\" (%s) is different from the global frame %s", initialpose_topic_name.c_str(), msg->header.frame_id.c_str(), global_frame_id_out_.c_str());
      }

      // set offset so that current pose is set to [initialpose_topic_name]    
      tf::StampedTransform baseInMap;
      try{
	// just get the latest
        m_tfListener->lookupTransform(base_frame_id_, global_frame_id_out_, ros::Time(0), baseInMap);
      } catch(tf::TransformException){
        ROS_WARN("Failed to lookup transform!");
        return;
      }

      tf::Transform delta = pose * baseInMap;
      m_offsetTf = delta * m_offsetTf;

    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_localization");

  FakeOdomNode odom;

  ros::spin();

  return 0;
}