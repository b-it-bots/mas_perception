cmake_minimum_required(VERSION 2.8.3)
project(mcr_scene_segmentation)

set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    dynamic_reconfigure
    mcr_perception_msgs
    mas_perception_libs
    pcl_ros
    roscpp
    roslint
    tf
    visualization_msgs
    mcr_algorithms
)
find_package(PCL 1.10 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/SceneSegmentation.cfg
)

catkin_package(
  INCLUDE_DIRS
    ros/include
    common/include
  LIBRARIES
    scene_segmentation
    mcr_segmentation
  CATKIN_DEPENDS
    mcr_perception_msgs
    visualization_msgs
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

add_definitions(-fpermissive)


### LIBRARIES ####################################################
add_library(scene_segmentation
  common/src/cloud_accumulation.cpp
  common/src/scene_segmentation.cpp
)

add_dependencies(scene_segmentation
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(scene_segmentation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(mcr_segmentation
  ros/src/laserscan_segmentation.cpp
  ros/src/pointcloud_segmentation.cpp
)
add_dependencies(mcr_segmentation
  ${catkin_EXPORTED_TARGETS}
)


### EXECUTABLES ###############################################
add_executable(scene_segmentation_node
  ros/src/scene_segmentation_node.cpp
)
add_dependencies(scene_segmentation_node
  ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg
)
target_link_libraries(scene_segmentation_node
  ${catkin_LIBRARIES}
  scene_segmentation
)

roslint_cpp()

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

### INSTALLS
install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(TARGETS scene_segmentation
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS scene_segmentation_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	TARGETS
		mcr_segmentation
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
