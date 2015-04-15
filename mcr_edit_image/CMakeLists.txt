cmake_minimum_required(VERSION 2.8.3)
project(mcr_edit_image)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/EditImage.cfg
)

catkin_package(
  INCLUDE_DIRS
    ros/include
  CATKIN_DEPENDS
    cv_bridge
    dynamic_reconfigure
    image_transport
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS
    OpenCV
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(edit_image_node
  ros/src/edit_image_node.cpp
)

add_dependencies(edit_image_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(edit_image_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

install(TARGETS edit_image_node
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