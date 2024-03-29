cmake_minimum_required(VERSION 2.8.3)
project(mcr_perception_tools)

find_package(PCL 1.7 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
)


catkin_package(
  CATKIN_DEPENDS
    smach
    smach_ros
    cv_bridge
    image_transport
  DEPENDS
    OpenCV
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

### EXECUTABLES ###############################################
add_executable(save_pointcloud
  ros/src/save_pointcloud.cpp
)

add_executable(webcam_publisher
  ros/src/webcam_publisher.cpp
)

target_link_libraries(save_pointcloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(webcam_publisher
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch input_topic:=/dummy_topic)
endif()


### INSTALLS
install(TARGETS save_pointcloud
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS webcam_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

# roslint does not detect the scripts being installed...
install(DIRECTORY ros/scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/scripts
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(PROGRAMS
  ros/scripts/collect_object_pointclouds
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
