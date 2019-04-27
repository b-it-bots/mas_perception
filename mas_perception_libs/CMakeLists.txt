cmake_minimum_required(VERSION 2.8.3)
project(mas_perception_libs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  rospy
  roslint
  dynamic_reconfigure
  cv_bridge
  image_geometry
  pcl_ros
  mcr_perception_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  ros/config/PlaneFitting.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    ros/include
    common/include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    mcr_perception_msgs
)

###########
## Build ##
###########
include_directories(
  common/include
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

###################################
# add mas_perception_libs library
add_library(${PROJECT_NAME}
  common/src/bounding_box.cpp
  common/src/bounding_box_2d.cpp
  common/src/point_cloud_utils.cpp
  common/src/sac_plane_segmenter.cpp
  ros/src/image_bounding_box.cpp
  ros/src/point_cloud_utils_ros.cpp
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

###################################
# python wrapper for cpp libraries
add_library(_cpp_wrapper SHARED
  common/src/init_numpy_api.cpp
  common/src/pyboost_cv3_converter.cpp
  ros/src/bounding_box_wrapper.cpp
  ros/src/boost_python_module.cpp
)
# change output directory, so python can find the module, and set prefix to ""
# to omit the default "lib".
set_target_properties(_cpp_wrapper PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
target_link_libraries(_cpp_wrapper
  mas_perception_libs
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

###################################
# Node for testing plane segmentation
add_executable(cloud_processing_cpp_test
  ros/src/cloud_processing_test_node.cpp
)
target_link_libraries(cloud_processing_cpp_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Testing ##
#############
roslint_python()
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch DEPENDENCIES cloud_processing_cpp_test)
  catkin_add_nosetests(ros/test/ DEPENDENCIES _cpp_wrapper)
endif()

#############
## Install ##
#############
# includes
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

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# python wrapper
install(TARGETS _cpp_wrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# executables
install(PROGRAMS
  ros/scripts/image_detection_action_server
  ros/scripts/image_detection_test
  ros/scripts/image_recognition_client_test
  ros/scripts/image_recognition_server
  ros/scripts/cloud_processing_python_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS cloud_processing_cpp_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
