cmake_minimum_required(VERSION 2.8.3)
project(mcr_perception_msgs)
find_package(catkin REQUIRED roscpp rospy std_msgs geometry_msgs message_generation)

add_message_files(
  FILES
  Person.msg
  PersonList.msg
  LaserScanSegment.msg
  LaserScanSegmentList.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs  
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
