cmake_minimum_required(VERSION 2.8.3)
project(mcr_text_extraction)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    dynamic_reconfigure
    image_transport
    libsvm
    message_generation
    roscpp
    rospy
    std_msgs
    std_srvs
)
find_package(OpenCV REQUIRED)

add_service_files(
  FILES
    GetExtractedText.srv
    GetRawText.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    libsvm
    message_runtime
    std_msgs
    std_srvs
)

include_directories(
  ros/include/${PROJECT_NAME}/
  common/include/${PROJECT_NAME}/
  ${OpenCV_INCLUDE_DIRS}
)

# Base path to the models
set(CMAKE_CXX_FLAGS "-fpermissive")
add_definitions(-DMODEL_DATA_INSTALL_PATH=\"${PROJECT_SOURCE_DIR}/common/config/\")

add_library(atie_old
  common/src/atie_old/cc_classification.cpp
  common/src/atie_old/complex_moments.cpp
  common/src/atie_old/cc_based_tie.cpp
  common/src/atie_old/chen_niblack_binarization.cpp
  common/src/atie_old/feature_extractors.cpp
  common/src/atie_old/parameters.cpp
  common/src/atie_old/util.cpp
)
target_link_libraries(atie_old
  ${catkin_LIBRARIES}
)

add_library(active_tie
  common/src/active_tie/active_camera.cpp
  common/src/active_tie/sharpness_measures.cpp
  common/src/active_tie/active_camera_values.cpp
  common/src/active_tie/gradients.cpp
  common/src/active_tie/integrals.cpp
  common/src/active_tie/sony_dfwvl500_values.cpp
  common/src/active_tie/ov_active_camera.cpp
  common/src/active_tie/ocr_interface.cpp
  common/src/active_tie/acquisition_parameters_optimization.cpp
  common/src/active_tie/image_preprocessing.cpp
)

add_executable(atie_old_node
  ros/src/atie_old_node.cpp
  ros/src/active_camera_ros.cpp
  ros/src/old_tie_ros_interface.cpp
  ros/src/tie_ros_interface.cpp
)
add_dependencies(atie_old_node
  ${PROJECT_NAME}_gencpp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(atie_old_node
  ${catkin_LIBRARIES}
  atie_old
  active_tie
)

install(TARGETS atie_old_node atie_old_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY common/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/config
)
