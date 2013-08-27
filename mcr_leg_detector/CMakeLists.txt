cmake_minimum_required(VERSION 2.8.3)
project(mcr_leg_detector)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation dynamic_reconfigure mcr_algorithms message_filters tf)
find_package(OpenCV)

add_message_files(
  FILES
  PositionMeasurement.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  ros/config/LegDetectorConfig.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime dynamic_reconfigure
)

include_directories(
  ros/include/
  cfg/cpp/
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(leg_detection_node 
  ros/src/leg_detector_node.cpp
  ros/src/laser_processor.cpp
  ros/src/calc_leg_features.cpp
  ros/src/tracker_kalman.cpp
)

target_link_libraries(leg_detection_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}   
  /opt/ros/groovy/lib/liborocos-bfl.so
  
)

add_dependencies(leg_detection_node ${PROJECT_NAME}_gencfg) 


install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
