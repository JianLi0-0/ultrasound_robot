cmake_minimum_required(VERSION 3.0.2)
project(ultrasound_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  tf
  dynamic_reconfigure
  message_generation
  control_msgs
  pcl_ros
  pcl_conversions
  trajectory_msgs
  trac_ik_lib
)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/force_controller.cfg
)

add_service_files(
  FILES
  wipe_bb.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES supersound_robot
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(admittance_controller
  src/admittance_controller.cpp
  src/StatesHub.cpp
  src/ForceTorqueController.cpp
)
add_dependencies(admittance_controller 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(admittance_controller
  ${catkin_LIBRARIES}
)

add_executable(wipe_blackboard
  src/wipe_blackboard.cpp
  src/StatesHub.cpp
  src/ForceTorqueController.cpp
  src/PointCloudCustomLib.cpp
)
add_dependencies(wipe_blackboard 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(wipe_blackboard
  ${catkin_LIBRARIES}
)

add_executable(us_image_acquisition
  src/us_image_acquisition.cpp
  src/StatesHub.cpp
  src/ForceTorqueController.cpp
  src/CustomRosLib.cpp
)
add_dependencies(us_image_acquisition 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(us_image_acquisition
  ${catkin_LIBRARIES}
)
