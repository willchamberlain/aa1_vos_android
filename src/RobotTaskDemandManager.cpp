#include <string>
#include <stdlib.h>
#include "ros/ros.h"
#include <cstdlib>
#include "vos_aa1/RobotTaskDemandManualManagement.h"
#include "vos_aa1/where_is_alg_desc.h"

ros::ServiceClient vos_server_where_is_client;

int loadAndheckArguments(int argc, char **argv) {
  // load arguments from file, parameter server, etc
  // check arguments then print something or whatever
  return 0;
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


  if(0 != loadAndheckArguments(argc, argv)) {
    return 1;
  }

  std::string node_namespace = "robot_task_demand_manager";
  std::string serviceName = "robot_task_demand_manual_management";
  std::string vos_server__where_is__service__name = "/vos_server/where_is";

  ros::init(argc, argv, node_namespace);
  ros::NodeHandle nh;
  ros::ServiceServer robotTaskDemandManualManagementServer = setup_RTDMMS_node(serviceName, nh);
  vos_server_where_is_client = nh.serviceClient<vos_aa1::where_is_alg_desc>(vos_server__where_is__service__name);

  ros::spin();  // run input checks then wait, until node killed externally

  return 0;
}
