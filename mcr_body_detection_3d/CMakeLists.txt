cmake_minimum_required(VERSION 2.8.3)
project(mcr_body_detection_3d)

find_package(catkin REQUIRED roscpp sensor_msgs std_srvs tf visualization_msgs mcr_perception_msgs pcl_ros mcr_algorithms dynamic_reconfigure rostest)
find_package(PCL 1.6 REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/BodyDetection3D.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure mcr_algorithms mcr_perception_msgs visualization_msgs sensor_msgs
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  
)

add_library(mcr_body_detection_3d ros/src/body_detection_3d.cpp)
add_dependencies(mcr_body_detection_3d ${catkin_EXPORTED_TARGETS})

add_executable(body_detection_3d_node ros/src/body_detection_3d_node.cpp)
add_dependencies(body_detection_3d_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(body_detection_3d_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS mcr_body_detection_3d mcr_body_detection_3d
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS body_detection_3d_node body_detection_3d_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(FILES ros/config/body_detection_model.mdl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config/body_detection_model.mdl
)

add_executable(integration_test EXCLUDE_FROM_ALL ros/test/integration.test ros/test/integration_test.cpp)
target_link_libraries(integration_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(tests integration_test)
add_dependencies(integration_test body_detection_3d_node)
add_rostest(ros/test/integration.test)
