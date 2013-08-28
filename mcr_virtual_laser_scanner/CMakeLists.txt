cmake_minimum_required(VERSION 2.8.3)
project(mcr_virtual_laser_scanner)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure tf pcl_ros sensor_msgs std_srvs)
find_package(PCL 1.5 REQUIRED)


generate_dynamic_reconfigure_options(
  ros/config/VirtualLaserScanner.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(virtual_laser_scanner_node ros/src/virtual_laser_scanner_node.cpp)
target_link_libraries(virtual_laser_scanner_node     
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(virtual_laser_scanner_node ${PROJECT_NAME}_gencfg) 

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
