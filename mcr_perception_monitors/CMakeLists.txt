cmake_minimum_required(VERSION 2.8.3)
project(mcr_perception_monitors)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  dynamic_reconfigure
  mcr_manipulation_msgs
)

generate_dynamic_reconfigure_options(
  ros/config/BlobTrackingErrorMonitor.cfg
)

catkin_package(
  INCLUDE_DIRS
    ros/include
  CATKIN_DEPENDS 
    geometry_msgs
    roscpp
    std_msgs
    dynamic_reconfigure
    mcr_manipulation_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(blob_tracking_error_monitor_node
  ros/src/blob_tracking_error_monitor_node.cpp
)

add_dependencies(blob_tracking_error_monitor_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(blob_tracking_error_monitor_node
  ${catkin_LIBRARIES}
)

install(TARGETS blob_tracking_error_monitor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)