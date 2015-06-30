cmake_minimum_required(VERSION 2.8.3)
project(mcr_object_recognition_height_based)

find_package(catkin REQUIRED
  COMPONENTS
    mcr_algorithms
    mcr_tabletop_segmentation
    mcr_perception_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf
    visualization_msgs
)

find_package(PCL 1.5 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    mcr_perception_msgs
    sensor_msgs
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(object_recognition_height_based_node
  ros/src/object_recognition_height_based_node.cpp
)
add_dependencies(object_recognition_height_based_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(object_recognition_height_based_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(TARGETS object_recognition_height_based_node object_recognition_height_based_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}ros/launch
)
