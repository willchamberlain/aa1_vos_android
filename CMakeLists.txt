cmake_minimum_required(VERSION 2.8.3)
project(vos_aa1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
    message_generation
    message_runtime
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  VisualFeatureInWorld.msg
  VisualFeatureObservation.msg
  WhereIsAsPub.msg
  localise_by_visual_descriptor_msg.msg
  Observation2D3D.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
    LocaliseFromView.srv
    LocaliseFromAFeature.srv
    DetectedFeature.srv
    DetectedFeatures.srv
    FindItems.srv
    RegisterVisionSource.srv
    SetPose.srv
    there_is_alg_desc.srv
    where_is_alg_desc.srv
    localise_by_visual_descriptor.srv
    RobotTaskDemandManualManagement.srv
    GetTf.srv
    RobotId.srv  # robot id service message
    InitialPoseToRobot.srv
    PoseToRobot.srv
    PoseStampedToRobot.srv
    PoseFrom2D3D.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
    sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vos_aa1
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    std_msgs
    sensor_msgs
    tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)




################################################################################
### 2017_03_20 :  publish dummy odometry for the fake_localization amcl-replacement node to run from: see  http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom  and  http://wiki.ros.org/fake_localization, respectively

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})   # the include/header files/dirs
# Build the executable
add_executable(odometry_publisher  src/odometry_publisher.cpp)  # the executable and the list of files to compile and link into it
target_link_libraries(odometry_publisher                        # Link against the catkin libraries
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )
# Add a build order dependency on nav_msgs
# This ensures that nav_msgs' msg headers are built before your executable
if(nav_msgs_EXPORTED_TARGETS)
  add_dependencies(odometry_publisher ${nav_msgs_EXPORTED_TARGETS})
endif()
add_dependencies(odometry_publisher nav_msgs_generate_messages_cpp)
add_dependencies(odometry_publisher geometry_msgs_generate_messages_cpp)
# Install the built executable
install(
    TARGETS
        odometry_publisher
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
### 2018_03_15 fake laser scanner: should send a bunch of no-detections
# Build the executable
add_executable(fake_laser_scanner  src/fake_laser_scan.cpp)  # the executable and the list of files to compile and link into it
## cmake target dependencies of the executable, including cmake steps, project modules, and compilation dependencies
add_dependencies(fake_laser_scanner
  vos_aa1_gencpp                  # generate cpp ?? - see http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29#roscpp_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.Writing_a_Service_Node
  vos_aa1_generate_messages_cpp   # generation of messages, services, actions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
# link dependencies
target_link_libraries(fake_laser_scanner  ${catkin_LIBRARIES}
)

################################################################################
### 2018_03_21 fake_localization with parameters for topic names
# Build the executable
add_executable(fake_localization  src/fake_localization.cpp)  # the executable and the list of files to compile and link into it
## cmake target dependencies of the executable, including cmake steps, project modules, and compilation dependencies
add_dependencies(fake_localization
  vos_aa1_gencpp                  # generate cpp ?? - see http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29#roscpp_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.Writing_a_Service_Node
  vos_aa1_generate_messages_cpp   # generation of messages, services, actions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
# link dependencies
target_link_libraries(fake_localization  ${catkin_LIBRARIES}
)

################################################################################

## Declare a C++ library
# add_library(vos_aa1
#   src/${PROJECT_NAME}/vos_aa1.cpp
# )
  ## Add cmake target dependencies of the library - as an example, code may need to be generated before libraries - either from message generation or dynamic reconfigure
# add_dependencies(vos_aa1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  ## Specify libraries to link a library or executable target against
# target_link_libraries(vos_aa1_node
#   ${catkin_LIBRARIES}
# )


## executable to demo pose interpolation in tf C++: RosJava doesn't have full tf implementation
add_executable( tf_demo_interpolation_over_time  src/TFDemoInterpolationOverTime.cpp )
## cmake target dependencies of the executable, including cmake steps, project modules, and compilation dependencies
add_dependencies(tf_demo_interpolation_over_time
  vos_aa1_gencpp                  # generate cpp ?? - see http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29#roscpp_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.Writing_a_Service_Node
  vos_aa1_generate_messages_cpp   # generation of messages, services, actions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
# link dependencies
target_link_libraries(tf_demo_interpolation_over_time
  ${catkin_LIBRARIES}
)


## executable for phones to look up tf: RosJava doesn't have full tf implementation
add_executable( robot_pose_look_up_service_node  src/RobotPoseLookUpServiceNode.cpp )
## cmake target dependencies of the executable, including cmake steps, project modules, and compilation dependencies
add_dependencies(robot_pose_look_up_service_node
  vos_aa1_gencpp                  # generate cpp ?? - see http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29#roscpp_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.Writing_a_Service_Node
  vos_aa1_generate_messages_cpp   # generation of messages, services, actions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
# link dependencies
target_link_libraries(robot_pose_look_up_service_node  ${catkin_LIBRARIES}
)


## executable for phones to look up tf: RosJava doesn't have full tf implementation
add_executable( test_robot_pose_lookup_service_fixed_rate  src/test_RobotPoseLookUpServiceNode.cpp )
## cmake target dependencies of the executable, including cmake steps, project modules, and compilation dependencies
add_dependencies(test_robot_pose_lookup_service_fixed_rate
  vos_aa1_gencpp                  # generate cpp ?? - see http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29#roscpp_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.Writing_a_Service_Node
  vos_aa1_generate_messages_cpp   # generation of messages, services, actions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
# link dependencies
target_link_libraries(test_robot_pose_lookup_service_fixed_rate  ${catkin_LIBRARIES}
)


## executable for phones to look up tf: RosJava doesn't have full tf implementation
add_executable( test_robot_pose_lookup_service_cmdline  src/test2_RobotPoseLookUpServiceNode.cpp )
## cmake target dependencies of the executable, including cmake steps, project modules, and compilation dependencies
add_dependencies(test_robot_pose_lookup_service_cmdline
  vos_aa1_gencpp                  # generate cpp ?? - see http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29#roscpp_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.Writing_a_Service_Node
  vos_aa1_generate_messages_cpp   # generation of messages, services, actions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
# link dependencies
target_link_libraries(test_robot_pose_lookup_service_cmdline  ${catkin_LIBRARIES}
)


## Declare a C++ executable
add_executable( RobotTaskDemandManager  src/RobotTaskDemandManager.cpp )
## Specify libraries to link a library or executable target against
target_link_libraries(RobotTaskDemandManager
  ${catkin_LIBRARIES}
)
  ## Add cmake target dependencies of the executable
add_dependencies(RobotTaskDemandManager
  vos_aa1_gencpp                  # generate cpp ?? - see http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29#roscpp_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.Writing_a_Service_Node
  vos_aa1_generate_messages_cpp   # generation of messages, services, actions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


## Declare a C++ executable
add_executable( RobotTaskDemandManagerPioneer2  src/RobotTaskDemandManagerPioneer2.cpp )
## Specify libraries to link a library or executable target against
target_link_libraries(RobotTaskDemandManagerPioneer2
  ${catkin_LIBRARIES}
)
  ## Add cmake target dependencies of the executable
add_dependencies(RobotTaskDemandManagerPioneer2
  vos_aa1_gencpp                  # generate cpp ?? - see http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29#roscpp_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.Writing_a_Service_Node
  vos_aa1_generate_messages_cpp   # generation of messages, services, actions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS vos_aa1 vos_aa1_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vos_aa1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
