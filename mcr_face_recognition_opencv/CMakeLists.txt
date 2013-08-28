cmake_minimum_required(VERSION 2.8.3)
project(mcr_face_recognition_opencv)

find_package(catkin REQUIRED roscpp libface mcr_perception_msgs mcr_common_msgs cv_bridge)
find_package(OpenCV REQUIRED)
catkin_package()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(face_recognition_node
  ros/src/face_recognition_node.cpp
  ros/src/face_recognition.cpp
)
add_dependencies(face_recognition_node
    mcr_perception_msgs_gencpp
    mcr_common_msgs_gencpp
)

target_link_libraries(face_recognition_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  face
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
