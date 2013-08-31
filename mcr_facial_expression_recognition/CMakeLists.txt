cmake_minimum_required(VERSION 2.8.3)
project(mcr_facial_expression_recognition)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    image_transport
    mcr_common_msgs
    mcr_perception_msgs
    roscpp
    sensor_msgs
    std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    mcr_common_msgs 
    mcr_perception_msgs 
    sensor_msgs 
    std_msgs
)

include_directories(
  common/include/${PROJECT_NAME}
  common/src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(facial_expression_recognition_node 
  ros/src/facial_expression_recognition_2d_gabor_filters_node.cpp
)
add_dependencies(facial_expression_recognition_node 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(facial_expression_recognition_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS facial_expression_recognition_node facial_expression_recognition_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)