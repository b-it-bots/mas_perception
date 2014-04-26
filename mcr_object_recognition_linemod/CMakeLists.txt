cmake_minimum_required(VERSION 2.8.3)
project(mcr_object_recognition_linemod)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    image_transport
    mcr_perception_msgs
    pcl_ros
    roscpp
    rostest
    sensor_msgs
    tf
    mcr_tabletop_segmentation
    visualization_msgs
    geometry_msgs
)
find_package(PCL 1.7 REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/ObjectRecognitionLinemod.cfg
)

catkin_package(
  CATKIN_DEPENDS 
    dynamic_reconfigure
    mcr_perception_msgs
    sensor_msgs
    mcr_tabletop_segmentation
    visualization_msgs
    geometry_msgs
)

include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(linemod_implementation
  common/src/linemod.cpp
)

target_link_libraries(linemod_implementation
  ${PCL_LIBRARIES}
)  

add_executable(object_recognition_node
  ros/src/object_recognition_node.cpp
)

add_dependencies(object_recognition_node 
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(object_recognition_node 
  linemod_implementation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
) 

add_executable(object_learning_node
  ros/src/object_learning_node.cpp
)

add_dependencies(object_learning_node   
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(object_learning_node 
  linemod_implementation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
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

add_executable(integration_test_linemod EXCLUDE_FROM_ALL ros/test/integration.test ros/test/integration_test.cpp)

target_link_libraries(integration_test_linemod
  ${GTEST_LIBRARIES} 
  ${catkin_LIBRARIES}
)

add_dependencies(tests
  integration_test_linemod
)

add_dependencies(integration_test_linemod
  object_recognition_node
)

add_rostest(ros/test/integration.test)