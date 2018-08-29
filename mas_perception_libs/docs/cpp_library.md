# C++ Library

## Classes and structs
### `BoundingBox`
Create a bounding box with 4 3D points as vertices given a point cloud and a plane normal. Defined in:
* [`bounding_box.h`](../common/include/mas_perception_libs/bounding_box.h)
* [`bounding_box.cpp`](../common/src/bounding_box.cpp)

### `BoundingBoxWrapper`
Boost Python port of `BoundingBox`. Python interface described in
[documentation for the python package](python_package.md). Defined in:
* [`bounding_box_wrapper.h`](../ros/include/mas_perception_libs/bounding_box_wrapper.h)
* [`bounding_box_wrapper.cpp`](../ros/src/bounding_box_wrapper.cpp)

### `BoundingBox2D`
Struct containing info relevant to a bounding box in RGB images. Used for visualization functions and point cloud
utilities. Python interface described in [documentation for the python package](python_package.md). Defined in:
*

## Utility functions
### ROS message serialization
Contains method to serialize and deserialize ROS messages, for passing them between C++ and Python
code. Defined in [`ros_message_serialization.hpp`](../ros/include/mas_perception_libs/impl/ros_message_serialization.hpp).