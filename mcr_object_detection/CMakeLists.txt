cmake_minimum_required(VERSION 2.8.3)
project(mcr_object_detection)

find_package(catkin REQUIRED
  COMPONENTS
    mcr_scene_segmentation
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    mcr_perception_msgs
    mcr_scene_segmentation
    rospy
    smach
    smach_ros
    tf
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch workspace_finder_config_file:=${mcr_scene_segmentation_workspace_constraints_file})
endif()

install(PROGRAMS
  ros/scripts/object_detector
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
