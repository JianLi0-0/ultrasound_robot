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
  cv_bridge
  image_transport
  nav_msgs
)
# find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(VISP REQUIRED)

SET(ITK_DIR "/home/kuka/SVR/ITK/build")
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
include_directories( include/svr/optim )
include_directories( include/svr/itk_img )
include_directories( include/svr/optimization )
include_directories( include/svr/optimization )
include_directories( include/visual_servo )
include_directories( include/eigen )

set(ARMADILLO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/armadillo/include)
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories( include/extended_kalman_filter )
include_directories(/usr/include/python2.7)

generate_dynamic_reconfigure_options(
  cfg/svr.cfg
)

add_service_files(
  FILES
  wipe_bb.srv
  bayesian_svr.srv
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
  ${OpenCV_INCLUDE_DIRS}
  # ${VISP_INCLUDE_DIRS}
)


add_executable(admittance_controller src/admittance_controller.cpp src/StatesHub.cpp src/ForceTorqueController.cpp src/extended_kalman_filter/PoseKalmanFilter.cpp)
target_link_libraries(admittance_controller ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})


# add_executable(wipe_blackboard src/wipe_blackboard.cpp src/StatesHub.cpp src/ForceTorqueController.cpp src/PointCloudCustomLib.cpp)
# target_link_libraries(wipe_blackboard ${catkin_LIBRARIES})


add_executable(us_image_acquisition src/us_image_acquisition.cpp src/StatesHub.cpp src/ForceTorqueController.cpp src/CustomRosLib.cpp src/extended_kalman_filter/PoseKalmanFilter.cpp)
target_link_libraries(us_image_acquisition ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})


# test registratration
add_executable(HelloInsight src/svr/HelloInsight.cpp src/svr/ITKImgProcess.cpp src/svr/Optimization.cpp)
target_link_libraries(HelloInsight ${ITK_LIBRARIES})


# interventional navigation
add_executable(thyroid_biopsy src/thyroid_biopsy.cpp src/svr/ITKImgProcess.cpp src/svr/Optimization.cpp src/StatesHub.cpp 
  src/ForceTorqueController.cpp src/CustomRosLib.cpp src/visual_servo/VisualServo.cpp 
  src/extended_kalman_filter/PoseKalmanFilter.cpp)
target_link_libraries(thyroid_biopsy ${ITK_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(thyroid_biopsy ${PROJECT_NAME}_gencfg)


add_executable(visual_servo_test src/visual_servo/visual_servo_test src/visual_servo/VisualServo.cpp src/StatesHub.cpp src/CustomRosLib.cpp src/extended_kalman_filter/PoseKalmanFilter.cpp)
target_link_libraries(visual_servo_test ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

# kalman filter
add_executable(KF_example src/extended_kalman_filter/kf_example.cpp)
target_link_libraries(KF_example ${ARMADILLO_LIBRARIES} libpython2.7.so)

add_executable(EKF_example src/extended_kalman_filter/ekf_example.cpp)
target_link_libraries(EKF_example ${ARMADILLO_LIBRARIES} libpython2.7.so)