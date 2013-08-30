cmake_minimum_required(VERSION 2.8.3)
project(mcr_leg_detection)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation dynamic_reconfigure mcr_algorithms message_filters tf)
find_package(OpenCV)

generate_dynamic_reconfigure_options(
  ros/config/LegDetection.cfg
)

add_message_files(
  FILES
  PositionMeasurement.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime dynamic_reconfigure mcr_algorithms std_msgs geometry_msgs
)

include_directories(
  ros/include/
  cfg/cpp/
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(leg_detection_node 
  ros/src/leg_detection_node.cpp
  ros/src/laser_processor.cpp
  ros/src/calc_leg_features.cpp
  ros/src/tracker_kalman.cpp
)
add_dependencies(leg_detection_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(leg_detection_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}   
  /opt/ros/groovy/lib/liborocos-bfl.so
  
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
