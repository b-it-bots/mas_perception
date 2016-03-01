cmake_minimum_required(VERSION 2.8.3)
project(mcr_scene_segmentation)

find_package(catkin REQUIRED
  COMPONENTS
    mcr_perception_msgs
    message_filters
    roscpp
    rospy
    tf
    visualization_msgs
)
find_package(PCL 1.6 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    ros/include
    common/include
  LIBRARIES
    scene_segmentation
    bounding_box
  CATKIN_DEPENDS
    mcr_perception_msgs
    visualization_msgs
  CFG_EXTRAS
    export_config.cmake
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
)
add_dependencies(scene_segmentation 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(scene_segmentation
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(bounding_box
  common/src/bounding_box.cpp
)
add_dependencies(bounding_box 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(bounding_box
  ${OpenCV_LIBRARIES}
)

### EXECUTABLES ###############################################
add_executable(workspace_finder
  ros/src/workspace_finder_node.cpp
)
add_dependencies(workspace_finder 
  ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg
)
target_link_libraries(workspace_finder
  ${catkin_LIBRARIES}
  scene_segmentation
)

add_executable(tabletop_cloud_accumulator
  ros/src/tabletop_cloud_accumulator_node.cpp
)
target_link_libraries(tabletop_cloud_accumulator
  scene_segmentation
)

add_executable(tabletop_cloud_clusterer
  ros/src/tabletop_cloud_clusterer_node.cpp
)
target_link_libraries(tabletop_cloud_clusterer
  ${catkin_LIBRARIES}
  scene_segmentation
)

add_executable(bounding_box_maker
  ros/src/bounding_box_maker_node.cpp
)
target_link_libraries(bounding_box_maker
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  bounding_box 
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch workspace_finder_config_file:=${CMAKE_CURRENT_SOURCE_DIR}/ros/config/workspace_constraints.yaml)
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

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(TARGETS scene_segmentation
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS bounding_box
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS workspace_finder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS bounding_box_maker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS tabletop_cloud_clusterer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS tabletop_cloud_accumulator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
