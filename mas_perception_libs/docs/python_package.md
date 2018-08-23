# Python Package

## Classes

### `SceneDetectionActionServer`
An abstract class which creates an `actionlib.SimpleActionServer` object to handle object detection action goals using
action specifications in
[`mcr_perception_msgs/DetectScene.action`](../../mcr_perception_msgs/action/DetectScene.action). An extension of this
class needs to implement:
* `_initialize`: initialization procedures before starting the action servers (i.e. loading models).
* `_execute_cb`: perform object detection and respond to the action client.

### `SceneDetectionTestActionServer`
Test extension of `SceneDetectionActionServer`, used by
[`object_detection_test_server`](../ros/scripts/object_detection_test_server) for testing without an actual object
detection action server.

### `ImageClassifier`
Abstract class for image classification.
* An extension of this class needs to implement the following functions:
    - `classify`: receive a list of `sensor_msgs/Image` as argument, returns a list of corresponding classes.
* Any extension of this class can be used by
[`image_recognition_server`](../ros/scripts/image_recognition_server) for classifying images.

Defined in [`image_classifier.py`](../ros/src/mas_perception_libs/image_classifier.py).

#### `ImageClassifierTest`
An example implementation of `ImageClassifier` which return random classes for each image message.
Used by `mdr_perceive_plane_action` for testing.

Defined in [`image_classifier.py`](../ros/src/mas_perception_libs/image_classifier.py).

#### `KerasImageClassifier`
An implementation of  `ImageClassifier` which use the Keras framework to classify images.

Defined in [`keras_image_classifier.py`](../ros/src/mas_perception_libs/keras_image_classifier.py).

### `ImageRecognitionServiceProxy`
Interact with [`image_recognition_server`](../ros/scripts/image_recognition_server) to get image classification results.

Defined in [`image_recognition_service.py`](../ros/src/mas_perception_libs/image_recognition_service_proxy.py).

### `ObjectDetector`
* Interact with a [`mcr_perception_msgs/DetectScene.action`](../../mcr_perception_msgs/action/DetectScene.action)
  action server to get a list of planes containing objects.
* Perform common preprocessing steps on the objects (i.e. create bounding box, transform to desired frame,...).
* Detection service is triggered by method `start_detect_objects`. A callback param can be passed in to be executed at
  the end of the method.
* The list of planes and objects can be accessed through property `plane_list`.
* An example usage is written in the `DetectObjects` state, defined in file `action_states.py` of the
  `mdr_perceive_plane_action` package.

Defined in [`object_detector.py`](../ros/src/mas_perception_libs/object_detector.py).

### `BoundingBox`
Port of bounding box creation for point clouds from C++. Provide the following API methods:
* `get_pose`: returns `geometry_msgs/PoseStamped` of the bounding box.
* `get_ros_message`: returns `mcr_perception_msgs/BoundingBox` version of the bounding box.

Defined in [`bounding_box.py`](../ros/src/mas_perception_libs/bounding_box.py).

## Utilities
### [`utils.py`](../ros/src/mas_perception_libs/utils.py)
#### `get_classes_in_data_dir`
Returns a list of strings as class names for a directory. This directory structure
```
data
├── class_1
└── class_2
```
should returns
```
['class_1', 'class_2']
```
when called on `data`.

#### `process_image_message`
Converts `sensor_msgs/Image` to CV image, then resizes and/or runs a preprocessing function if specified.

#### `case_insensitive_glob`
glob files ignoring case.

### `ros_message_serialization.py`
Serialize and deserialize ROS messages for interation with C++ code.

### [`constants.py`](../ros/src/mas_perception_libs/constants.py)
Common place for all the perception related constants.
