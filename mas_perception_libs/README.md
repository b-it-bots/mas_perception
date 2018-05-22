# `mas_perception_libs`

Library containing shared perception functionalities.

## [C++ Library](docs/cpp_library.md)

Contains shared perception defintions in C++.

## [Python Package](docs/python_package.md)

Contains shared perception definitions in Python.

## Executables
### [`image_recognition_server`](ros/scripts/image_recognition_server)
Server which uses an instance of `ImageClassifier` class (See
[documentation](docs/python_package.md)) to classify images.

Parameters:
* `service_name`: name of recognition service to advertise (default: `'~recognize_image'`).
* `model_directory`: directory containing the trained classification model (default: `''`)
* `recognition_module`: module containing the `ImageClassifier` instance
(default: `'mas_perception_libs'`)
* `recognition_class`: class name of the `ImageClassifier` instance
(default: `'ImageClassifierTest'`)

### [`object_detection_test_server`](ros/scripts/object_detection_test_server)
Service of type `std_srvs/Empty` which generates empty `mcr_perception_msgs/PlaneList` for testing.

### [`image_recognition_client_test`](ros/scripts/image_recognition_client_test)
Script to test the `image_recognition_server`.

```
usage: image_recognition_client_test [-h] --test-dir TEST_DIR --service-name
                                     SERVICE_NAME [--num-samples NUM_SAMPLES]
                                     [--preprocess-input-module PREPROCESS_INPUT_MODULE]
                                     model_name

Tool to test model with test images using KerasImageClassifier class.Assuming
images to be of type jpg

positional arguments:
  model_name            Keras model to be tested

optional arguments:
  -h, --help            show this help message and exit
  --test-dir TEST_DIR, -t TEST_DIR
                        directory with test images
  --service-name SERVICE_NAME, -s SERVICE_NAME
                        name of recognition service
  --num-samples NUM_SAMPLES, -n NUM_SAMPLES
                        number of samples to test, if left blank, take all
                        samples.
  --preprocess-input-module PREPROCESS_INPUT_MODULE, -p PREPROCESS_INPUT_MODULE
                        module containing image preprocessing function.
```

Example execution:
```
image_recognition_client_test -s <service_name> -t <folder_with_test_images> <model_name>
```

## Launch Files
### [`image_recognition.launch`](ros/launch/image_recognition.launch)
Launch the `image_recognition_server`. Arguments:
* `service_name`: name of recognition service to advertise (default: `'~recognize_image'`).
* `model_directory`: directory containing the trained classification model
(default: `$(find mas_perception_libs)/models`)
* `recognition_module`: module containing the `ImageClassifier` instance
(default: `'mas_perception_libs'`)
* `recognition_class`: class name of the `ImageClassifier` instance
(default: `'ImageClassifierTest'`)

## Directory structure

```
.
├── CMakeLists.txt
├── common
│   ├── include
│   │   └── mas_perception_libs
│   │       ├── aliases.h
│   │       └── bounding_box.h
│   └── src
│       └── bounding_box.cpp
├── docs
│   ├── cpp_library.md
│   └── python_package.md
├── models
│   └── test_model.txt
├── package.xml
├── README.md
├── ros
│   ├── include
│   │   └── mas_perception_libs
│   │       ├── bounding_box_wrapper.h
│   │       └── impl
│   │           └── ros_message_serialization.hpp
│   ├── launch
│   │   └── image_recognition.launch
│   ├── scripts
│   │   ├── image_recognition_client_test
│   │   ├── image_recognition_server
│   │   └── object_detection_test_server
│   └── src
│       ├── bounding_box_wrapper.cpp
│       └── mas_perception_libs
│           ├── bounding_box.py
│           ├── constants.py
│           ├── detection_service_proxy.py
│           ├── image_classifier.py
│           ├── image_recognition_service_proxy.py
│           ├── __init__.py
│           ├── keras_image_classifier.py
│           ├── object_detector.py
│           ├── ros_message_serialization.py
│           └── utils.py
├── setup.py
└── srv
    └── ImageRecognition.srv
```
