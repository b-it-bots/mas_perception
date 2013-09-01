cmake_minimum_required(VERSION 2.8.3)
project(mcr_object_recognition_linemod)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    dynamic_reconfigure
    image_transport
    mcr_perception_msgs
    mcr_tabletop_segmentation
    pcl_ros
    #pr2_controllers_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf
    trajectory_msgs
)
find_package(PCL 1.5 REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/ObjectRecognitionLinemod.cfg
)

catkin_package(
  CATKIN_DEPENDS 
    dynamic_reconfigure
    mcr_perception_msgs
    mcr_tabletop_segmentation
    #pr2_controllers_msgs
    sensor_msgs
    std_srvs
    trajectory_msgs
)

include_directories(
  common/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(linemod_recognition 
  common/src/linemod_recognition.cpp
)
target_link_libraries(linemod_recognition
  ${OpenCV_LIBRARIES}
)  

add_executable(object_recognition_node
  ros/src/object_recognition_node.cpp
)
add_dependencies(object_recognition_node 
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(object_recognition_node 
  linemod_recognition
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
) 

add_executable(object_learning_node
  ros/src/object_learning_node.cpp
)
target_link_libraries(object_learning_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS object_recognition_node object_recognition_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS object_learning_node object_learning_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)