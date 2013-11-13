cmake_minimum_required(VERSION 2.8.3)
project(mcr_ar_marker_detection)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    ar_kinect
    ar_pose
    geometry_msgs
    mcr_perception_msgs
    rospy
    tf
)

install(PROGRAMS
  ros/scripts/goal_tf_publisher
  ros/scripts/marker_detector
  ros/scripts/tf_filter
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
