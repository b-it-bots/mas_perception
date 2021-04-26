cmake_minimum_required(VERSION 2.8.3)
project(mcr_pose_estimation)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    roscpp
    sensor_msgs
    geometry_msgs
)

find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


### EXECUTABLES ###############################################
add_executable(pca_pose_estimator
  ros/src/pca_pose_estimator_node.cpp
  ros/src/pca_pose_estimator_ros.cpp
)

target_link_libraries(pca_pose_estimator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

### INSTALLS
install(TARGETS pca_pose_estimator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)


