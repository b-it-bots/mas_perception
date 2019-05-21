cmake_minimum_required(VERSION 2.8.3)
project(mcr_perception_states)

find_package(catkin REQUIRED
  COMPONENTS
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    mas_perception_msgs
    rospy
    smach
    smach_ros
    std_msgs
)

### TESTS
roslint_python()
