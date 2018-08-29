# C++ Library

## Classes and structs

### `BoundingBox2D`
Struct containing info relevant to a bounding box in RGB images. Used for visualization functions and point cloud
utilities. Python interface described in [documentation for the python package](python_package.md). Defined in:
* [`bounding_box_2d.h`](../common/include/mas_perception_libs/bounding_box_2d.h)
* [`bounding_box_2d.cpp`](../common/src/bounding_box_2d.cpp)

### `BoundingBox`
Create a bounding box with 4 3D points as vertices given a point cloud and a plane normal. Defined in:
* [`bounding_box.h`](../common/include/mas_perception_libs/bounding_box.h)
* [`bounding_box.cpp`](../common/src/bounding_box.cpp)

### `BoundingBoxWrapper`
Boost Python port of `BoundingBox`. Python interface described in
[documentation for the python package](python_package.md). Defined in:
* [`bounding_box_wrapper.h`](../ros/include/mas_perception_libs/bounding_box_wrapper.h)
* [`bounding_box_wrapper.cpp`](../ros/src/bounding_box_wrapper.cpp)

## Utilities

### ROS message serialization
Contains method to serialize and deserialize ROS messages, for passing them between C++ and Python code. Defined in
[`ros_message_serialization.hpp`](../ros/include/mas_perception_libs/impl/ros_message_serialization.hpp).

### Boost Python conversion between `cv::Mat` and NumPy array
Boost conversion between NumPy array, seen in C++ code as `PyObject *`, and `cv::Mat` can be done using functions
`pbcvt::fromMatToNDArray` and `pbcvt::fromNDArrayToMat`. These are defined in:
* [`pyboostcvconverter.hpp`](../common/include/mas_perception_libs/impl/pyboostcvconverter.hpp)
* [`pyboost_cv3_converter.cpp`](../common/src/pyboost_cv3_converter.cpp)

### Visualization utilities
* Functions to draw bounding boxes on images and image messages are defined in:
    - [`bounding_box_2d.cpp`](../common/src/bounding_box_2d.cpp)
    - [`bounding_box_2d.h`](../common/include/mas_perception_libs/bounding_box_2d.h)
    - [`image_bounding_box.h`](../ros/include/mas_perception_libs/image_bounding_box.h)
    - [`image_bounding_box.cpp`](../ros/src/image_bounding_box.cpp)
* Functions to fit bounding boxes to image sizes are defined in:
    - [`bounding_box_2d.cpp`](../common/src/bounding_box_2d.cpp)
    - [`bounding_box_2d.h`](../common/include/mas_perception_libs/bounding_box_2d.h)

### Point cloud utilites
Functions to crop point clouds or to extract images and coordinates from point clouds are defined in:
* [`point_cloud_utils.h`](../ros/include/mas_perception_libs/point_cloud_utils.h)
* [`point_cloud_utils.cpp`](../ros/src/point_cloud_utils.cpp)
