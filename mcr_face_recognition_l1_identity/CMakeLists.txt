cmake_minimum_required(VERSION 2.8.3)
project(mcr_face_recognition_l1_identity)

find_package(catkin REQUIRED
  COMPONENTS
    mcr_common_msgs
    mcr_perception_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
)
#find_package(L1IDENTITY REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    mcr_common_msgs
    mcr_perception_msgs
    sensor_msgs
    std_msgs
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${L1IDENTITY_INCLUDE_DIRS}
)

#add_executable(face_recognition_node 
#  ros/src/face_recognition_node.cpp
#)
#add_dependencies(face_recognition_node 
#  ${catkin_EXPORTED_TARGETS}
#)
#target_link_libraries(face_recognition_node
#  ${catkin_LIBRARIES}
#  ${L1IDENTITY_LIBRARIES}
#)

#install(TARGETS face_recognition_node face_recognition_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)