#include <string>
#include <stdlib.h>
#include "ros/ros.h"
#include <cstdlib>
#include "vos_aa1/RobotTaskDemandManualManagement.h"
#include "vos_aa1/where_is_alg_desc.h"

/**
 * @brief Temporary (2017_07_18) controller for Pioneer 3DX
 * pioneer2 with hardcoded robot model requests and target model requests.
 */

ros::ServiceClient vos_server_where_is_client;
bool should_continue = true;


int loadAndheckArguments(int argc, char **argv) {
  // load arguments from file, parameter server, etc
  // check arguments then print something or whatever
  return 0;
}

bool add_target_taskDemand(std::string boofcv_binary_fiducial_marker_tag_number, int rate_) {
  std::cout << "add_target_taskDemand: start" << std::endl;
  vos_aa1::where_is_alg_desc serviceMsg;
  serviceMsg.request.algorithm  = "boofcv";
  serviceMsg.request.descriptor = boofcv_binary_fiducial_marker_tag_number;
  serviceMsg.request.rate       = rate_;
  std::cout << "add_target_taskDemand: algorithm=" << serviceMsg.request.algorithm << ", descriptor=" << serviceMsg.request.descriptor << ", rate=" << serviceMsg.request.rate << std::endl;
  vos_server_where_is_client.call(serviceMsg);
  return true;
}

bool add_robot_taskDemand(std::string boofcv_binary_fiducial_marker_tag_number, int rate_) {
  return add_target_taskDemand(boofcv_binary_fiducial_marker_tag_number, rate_);
}

bool addRobotTaskDemand(
    vos_aa1::RobotTaskDemandManualManagement::Request &req,
    vos_aa1::RobotTaskDemandManualManagement::Response &resp) {
  std::cout << "addRobotTaskDemand: start" << std::endl;
  vos_aa1::where_is_alg_desc serviceMsg;
  serviceMsg.request.algorithm=req.alg;
  serviceMsg.request.descriptor=req.descriptor;
  serviceMsg.request.rate=atoi( req.rate.c_str() );
  std::cout << "addRobotTaskDemand: algorithm=" << serviceMsg.request.algorithm << ", descriptor=" << serviceMsg.request.descriptor << ", rate=" << serviceMsg.request.rate << std::endl;
  vos_server_where_is_client.call(serviceMsg);
  return true;
}


ros::ServiceServer setup_RTDMMS_node(std::string serviceName, ros::NodeHandle nh)
{
  ros::ServiceServer robotTaskDemandManualManagementServer = nh.advertiseService(serviceName, addRobotTaskDemand);
  return robotTaskDemandManualManagementServer;
}


int main(int argc, char **argv)
{
  std::cout << "argc = " << argc << std::endl;


  if(0 != loadAndheckArguments(argc, argv)) {
    return 1;
  }

  std::string node_namespace = "pioneer2_robot_task_demand_manager";
  std::string serviceName = "pioneer2_robot_task_demand_manual_management";
  std::string vos_server__where_is__service__name = "/vos_server/where_is";

  ros::init(argc, argv, node_namespace);
  ros::NodeHandle nh;
  ros::ServiceServer robotTaskDemandManualManagementServer = setup_RTDMMS_node(serviceName, nh);
  vos_server_where_is_client = nh.serviceClient<vos_aa1::where_is_alg_desc>(vos_server__where_is__service__name);

  std::cout << "argc = " << argc << std::endl;
  int max_iterations_ = 0;
  if (argc>1) {
    max_iterations_ = std::atoi(argv[1]);
    std::cout << "max_iterations_ = " << max_iterations_ << std::endl;
  }

  //ros::spin();  // run input checks then wait, until node killed externally
  ros::Rate rateAsHz(2.0); //  2.0hz
  int iteration_ = 0;
  int target_model_iteration_ = 0;
  int robot_model_iteration_ = 0;
  while (ros::isInitialized() && !ros::isShuttingDown() && should_continue )  //  see  http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
  {
    //... do some work, publish some messages, etc. ...
    iteration_++;

    // target model
    if (1 == iteration_ % 3) { // 3/2.0Hz
      target_model_iteration_++;
      add_target_taskDemand("210",1);  // find target tag once
    }

    // robot model
    if (1 == iteration_ % 2) { // 2/2.0Hz
      robot_model_iteration_++;
      add_robot_taskDemand("170",1);  // find target tag once
      add_robot_taskDemand("250",1);  // find target tag once
      add_robot_taskDemand("290",1);  // find target tag once
      add_robot_taskDemand("330",1);  // find target tag once
    }

    if(max_iterations_ > 0 && ( robot_model_iteration_ >= max_iterations_) || (target_model_iteration_ >= max_iterations_) ) { // cut off after enough calls to one model or another, rather than base iterations
      should_continue = false;
    }

    ros::spinOnce();
    rateAsHz.sleep();
  }
  ros::requestShutdown();


  return 0;
}
