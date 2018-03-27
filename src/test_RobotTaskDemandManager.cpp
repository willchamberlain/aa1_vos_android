#include <string>
#include "ros/ros.h"
#include <cstdlib>
#include "vos_aa1/RobotTaskDemandManualManagement.h"

int loadAndheckArguments(int argc, char **argv) {
  // load arguments from file, parameter server, etc
  // check arguments then print something or whatever
  return 0;
}

bool addRobotTaskDemand(
    vos_aa1::RobotTaskDemandManualManagement::Request &req,
    vos_aa1::RobotTaskDemandManualManagement::Response &resp) {
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

  ros::init(argc, argv, node_namespace);
  ros::NodeHandle nh;
  ros::ServiceServer robotTaskDemandManualManagementServer = setup_RTDMMS_node(serviceName, nh);

  ros::spin();  // run input checks then wait, until node killed externally

  return 0;
}
