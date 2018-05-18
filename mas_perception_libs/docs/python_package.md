# Python Package

## Classes

### `DetectionServiceProxy`
Abstract class to interact with any object detection service.
* An extension of this class needs to implement the following functions:
    - `_get_segmentation_req`: create a detection service request.
    - `_get_objects_and_planes_from_response`: convert the service response into a
    `mcr_perception_msgs/PlaneList` object, where each plane contains a list of detected objects.
* Any extension of this class can be passed into `ObjectDetector` class for interacting with the
desired detection service.

Defined in `ros/src/mas_perception_libs/detection_service_proxy.py`.

#### `DetectionServiceProxyTest`
An example implementation of the `DetectionServiceProxy` class which return an empty
`mcr_perception_msgs/PlaneList`. This is used in `mdr_perceive_plane_action` for testing.

Defined in `ros/src/mas_perception_libs/detection_service_proxy.py`.

### `ImageClassifier`
Abstract class for image classification.
* An extension of this class needs to implement the following functions:
    - `classify`: receive a list of `sensor_msgs/Image` as argument, returns a list of corresponding
    classes.
* Any extension of this class can be used by `ros/script/image_recognition_server` for classifying
images.

Defined in `ros/src/mas_perception_libs/image_classifier.py`.

#### `ImageClassifierTest`
An example implementation of `ImageClassifier` which return random classes for each image message.
Used by `mdr_perceive_plane_action` for testing.

Defined in `ros/src/mas_perception_libs/image_classifier.py`.

#### `KerasImageClassifier`
An implementation of  `ImageClassifier` which use the Keras framework to classify images.

Defined in `ros/src/mas_perception_libs/keras_image_classifier.py`.

### `ImageRecognitionServiceProxy`
Interact with `ros/script/image_recognition_server` to get image classification results.

Defined in `ros/src/mas_perception_libs/image_recognition_service_proxy.py`.

### `ObjectDetector`
* Interact with instances of `DetectionServiceProxy` to get a list of planes containing objects.
* Perform common preprocessing steps on the objects (i.e. create bounding box, transform to desired
frame,...).

Defined in `ros/src/mas_perception_libs/object_detector.py`.

### `BoundingBox`
Port of bounding box creation for point clouds from C++. Provide the following API methods:
* `get_pose`: returns `geometry_msgs/PoseStamped` of the bounding box.
* `get_ros_message`: returns `mcr_perception_msgs/BoundingBox` version of the bounding box.

Defined in `ros/src/mas_perception_libs/bounding_box.py`.

## Utilities
### `utils.py`
#### `get_classes_in_data_dir`
Returns a list of strings as class names for a directory. This
directory structure
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
Converts `sensor_msgs/Image` to CV image, then resizes and/or runs a preprocessing function
if specified.

#### `case_insensitive_glob`
glob files ignoring case.

### `ros_message_serialization.py`
Serialize and deserialize ROS messages for interation with C++ code.

### `constants.py`
Common place for all the perception related constants.
