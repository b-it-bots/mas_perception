cmake_minimum_required(VERSION 2.8.3)
project(mcr_perception_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation sensor_msgs)

add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxList.msg
  Person.msg
  PersonList.msg
  LaserScanSegment.msg
  LaserScanSegmentList.msg
  Object.msg
  ObjectList.msg
  Plane.msg
  Face.msg
  FaceList.msg
)

add_service_files(
  FILES
  SetFaceName.srv
  GetPersonList.srv
  GetObjectList.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs  
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
