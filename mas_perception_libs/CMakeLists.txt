cmake_minimum_required(VERSION 2.8.3)
project(mas_perception_libs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  rospy
  roslint
  cv_bridge
  image_geometry
  pcl_ros
  mcr_perception_msgs
  message_generation
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    ros/include
    common/include
  LIBRARIES
    mas_perception_libs
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
###################################
add_library(mas_perception_libs
  common/src/bounding_box.cpp
  common/src/bounding_box_2d.cpp
  ros/src/image_bounding_box.cpp
  ros/src/point_cloud_utils.cpp
)
target_link_libraries(mas_perception_libs
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

###################################
# python wrapper for cpp libraries
###################################
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

#############
## Testing ##
#############
roslint_python()
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
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

install(TARGETS mas_perception_libs
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# python wrapper
install(TARGETS _cpp_wrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# executables
install(PROGRAMS
  ros/scripts/image_recognition_server
  ros/scripts/image_detection_server
  ros/scripts/object_detection_test_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)