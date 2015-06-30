cmake_minimum_required(VERSION 2.8.3)
project(mcr_face_recognition_opencv)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    libface
    mcr_perception_msgs
    roscpp
)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    libface
    mcr_perception_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(face_recognition_node
  ros/src/face_recognition_node.cpp
  ros/src/face_recognition.cpp
)
add_dependencies(face_recognition_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(face_recognition_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
