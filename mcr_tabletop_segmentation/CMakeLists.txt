cmake_minimum_required(VERSION 2.8.3)
project(mcr_tabletop_segmentation)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    geometry_msgs
    image_transport
    mcr_perception_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf
)
  
find_package(PCL 1.5 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
    ros/include/
  LIBRARIES
    mcr_object_candidate_extraction
    mcr_plane_extraction
    mcr_toolbox_ros
    
  CATKIN_DEPENDS
    geometry_msgs
    mcr_perception_msgs
    sensor_msgs
    std_srvs
)

include_directories(
  ros/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(mcr_toolbox_ros 
  ros/src/toolbox_ros.cpp
)
target_link_libraries(mcr_toolbox_ros 
  ${PCL_LIBRARIES}
)

add_library(mcr_plane_extraction 
  ros/src/plane_extraction.cpp
)
target_link_libraries(mcr_plane_extraction 
  ${PCL_LIBRARIES}
)

add_library(mcr_object_candidate_extraction
  ros/src/object_candidate_extraction.cpp
)
target_link_libraries(mcr_object_candidate_extraction 
  mcr_plane_extraction
)

add_executable(tabletop_segmentation_node
	ros/src/tabletop_segmentation_node.cpp
	ros/src/object_segmentation.cpp
	#ros/src/roi_extraction.cpp
)
add_dependencies(tabletop_segmentation_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tabletop_segmentation_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  mcr_object_candidate_extraction
  mcr_toolbox_ros
)

install(TARGETS tabletop_segmentation_node tabletop_segmentation_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)