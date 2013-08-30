cmake_minimum_required(VERSION 2.8.3)
project(mcr_people_tracking)

find_package(catkin REQUIRED COMPONENTS roscpp mcr_algorithms tf pcl_ros mcr_perception_msgs)
find_package(PCL 1.5 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS mcr_algorithms mcr_perception_msgs
)

include_directories(
  ros/include
  common/include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(waist_tracking_node  
  ros/src/waist_tracking_node.cpp
  common/src/particle_filter.cpp
)
add_dependencies(waist_tracking_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(waist_tracking_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}   
  ${PCL_LIBRARIES}   
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
