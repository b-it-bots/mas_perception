import os
from abc import ABCMeta, abstractmethod
import yaml
from enum import Enum
import numpy as np

import rospy
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge

from .utils import process_image_message
from .bounding_box import BoundingBox2D
from .visualization import bgr_dict_from_classes, draw_labeled_boxes_img_msg


class ImageDetectionKey(Enum):
    CLASS = 0
    CONF = 1
    X_MIN = 2
    X_MAX = 3
    Y_MIN = 4
    Y_MAX = 5


class ImageDetectorBase(object):
    """
    Abstract class for detecting things in images
    """
    __metaclass__ = ABCMeta

    _classes = None                 # type: dict
    _class_colors = None            # type: dict
    _cv_bridge = None               # type: CvBridge
    # input size of model, will be ignored if left None
    _target_size = None             # type: tuple
    # preprocess function for each input image, will be ignored if left None
    _img_preprocess_func = None     # type: function

    def __init__(self, **kwargs):
        # for ROS image message conversion
        self._cv_bridge = CvBridge()

        # load dictionary of classes
        self._classes = kwargs.get('classes', None)
        if self._classes is None:
            class_file = kwargs.get('class_file', None)
            if class_file is not None and os.path.exists(class_file):
                with open(class_file, 'r') as infile:
                    self._classes = yaml.load(infile)

        if self._classes is None:
            raise ValueError("no valid 'class_file' or 'classes' parameter specified")

        # generate colors for each class for visualization
        self._class_colors = bgr_dict_from_classes(self._classes.values())

        # load kwargs file and call load_model()
        model_kwargs_file = kwargs.get('model_kwargs_file', None)
        if model_kwargs_file is not None and os.path.exists(model_kwargs_file):
            with open(model_kwargs_file, 'r') as infile:
                load_model_kwargs = yaml.load(infile)
        else:
            load_model_kwargs = {}

        self.load_model(**load_model_kwargs)

    @property
    def classes(self):
        """ dictionary which maps prediction value (int) to class name (str) """
        return self._classes

    @property
    def class_colors(self):
        """ dictionary which maps from class name (str) to RGB colors (3-tuple) """
        return self._class_colors

    @abstractmethod
    def load_model(self, **kwargs):
        """
        To be implemented by extensions, where detection model is loaded

        :param kwargs: key word arguments necessary for the detection model
        :return: None
        """
        pass

    @abstractmethod
    def _detect(self, np_images, orig_img_sizes):
        """
        To be implemented by extensions, detect objects in given image messages

        :param np_images: list of numpy images extracted from image messages
        :param orig_img_sizes: list of original images' (width, height), necessary to map detected bounding boxes back
                               to the original images if the images are resized to fit the detection model input
        :return: List of predictions for each image. Each prediction is a list of dictionaries representing the detected
                 classes with their bounding boxes and confidences. The dictionary keys are values of the
                 ImageDetectionKey Enum.
        """
        pass

    def detect(self, image_messages):
        """
        Preprocess image messages then call abstract method _detect() on the processed images

        :param image_messages: list of sensor_msgs/Image
        :return: same with _detect()
        """
        if len(image_messages) == 0:
            return []

        np_images = []
        orig_img_sizes = []
        for msg in image_messages:
            np_images.append(process_image_message(msg, self._cv_bridge, self._target_size, self._img_preprocess_func))
            orig_img_sizes.append((msg.width, msg.height))

        return self._detect(np_images, orig_img_sizes)

    def visualize_detection(self, img_msg, bounding_boxes):
        """
        Draw detected classes on an image message

        :param img_msg: sensor_msgs/Image message to be drawn on
        :param bounding_boxes: list of BoundingBox2D objects created from prediction
        :return: sensor_msgs/Image message with detected boxes drawn on top
        """
        return draw_labeled_boxes_img_msg(self._cv_bridge, img_msg, bounding_boxes)

    @staticmethod
    def prediction_to_bounding_boxes(prediction, color_dict=None):
        """
        Create BoundingBox2D objects from a prediction result

        :param prediction: List of dictionaries representing detected classes in an image. Keys are values of
                           ImageDetectionKey Enum
        :param color_dict: Dictionary mapping class name to a color tuple (r, g, b). Default color is blue.
        :return: List of BoundingBox2D objects, one for each predicted class
        """
        boxes = []
        classes = []
        confidences = []
        for box_dict in prediction:
            box_geometry = (box_dict[ImageDetectionKey.X_MIN],
                            box_dict[ImageDetectionKey.Y_MIN],
                            box_dict[ImageDetectionKey.X_MAX] - box_dict[ImageDetectionKey.X_MIN],
                            box_dict[ImageDetectionKey.Y_MAX] - box_dict[ImageDetectionKey.Y_MIN])

            label = '{}: {:.2f}'.format(box_dict[ImageDetectionKey.CLASS], box_dict[ImageDetectionKey.CONF])

            if color_dict is None:
                color = (0, 0, 255)     # default color: blue
            else:
                color = color_dict[box_dict[ImageDetectionKey.CLASS]]

            bounding_box = BoundingBox2D(label, color, box_geometry)
            boxes.append(bounding_box)
            classes.append(box_dict[ImageDetectionKey.CLASS])
            confidences.append(box_dict[ImageDetectionKey.CONF])

        return boxes, classes, confidences


class ImageDetectorTest(ImageDetectorBase):
    """
    Sample extension of ImageDetectorBase for testing
    """
    def __init__(self, **kwargs):
        self._min_box_ratio = None
        self._max_num_detection = None
        super(ImageDetectorTest, self).__init__(**kwargs)

    def load_model(self, **kwargs):
        self._min_box_ratio = kwargs.get('min_box_ratio', 0.2)
        self._max_num_detection = kwargs.get('max_num_detection', 7)

    def _detect(self, _, orig_img_sizes):
        """ Generate random detection results based on classes and parameters in example configuration files """
        predictions = []
        for img_size in orig_img_sizes:
            boxes = []
            min_box_width = int(img_size[0] * self._min_box_ratio)
            min_box_height = int(img_size[1] * self._min_box_ratio)
            num_detection = np.random.randint(1, self._max_num_detection)
            for _ in range(1, num_detection + 1):
                # generate random class and confidence
                detected_class = self._classes[np.random.choice(self._classes.keys())]
                confidence = int(np.random.rand() * 100) / 100.

                # calculate random box
                x_min = np.random.randint(img_size[0] - min_box_width)
                y_min = np.random.randint(img_size[1] - min_box_height)

                width = np.random.randint(min_box_width, img_size[0] - x_min)
                height = np.random.randint(min_box_height, img_size[1] - y_min)

                x_max = x_min + width
                y_max = y_min + height

                # create box dictionary
                box_dict = {ImageDetectionKey.CLASS: detected_class, ImageDetectionKey.CONF: confidence,
                            ImageDetectionKey.X_MIN: x_min, ImageDetectionKey.Y_MIN: y_min,
                            ImageDetectionKey.X_MAX: x_max, ImageDetectionKey.Y_MAX: y_max}
                boxes.append(box_dict)

            predictions.append(boxes)

        return predictions


class SingleImageDetectionHandler(object):
    """
    Simple handler for ImageDetectorBase class which publishes visualized detection result for a single image message
    on a specified topic if there're subscribers. Needs to be run within a node.
    """
    _detector = None    # type: ImageDetectorBase
    _result_pub = None  # type: rospy.Publisher

    def __init__(self, detection_class, class_annotation_file, kwargs_file, result_topic):
        self._detector = detection_class(class_file=class_annotation_file, model_kwargs_file=kwargs_file)
        self._result_pub = rospy.Publisher(result_topic, ImageMsg, queue_size=1)

    def process_image_msg(self, img_msg):
        """
        Draw detected boxes and publishes if there're subscribers on self._result_pub

        :type img_msg: ImageMsg
        :return: 3-tuple:
                 - list of bounding boxes created from prediction
                 - list of detected classes
                 - list of detection confidences
        """
        predictions = self._detector.detect([img_msg])
        if len(predictions) < 1:
            raise RuntimeError('no prediction returned for image message')
        bounding_boxes, classes, confidences \
            = ImageDetectorBase.prediction_to_bounding_boxes(predictions[0], self._detector.class_colors)
        if self._result_pub.get_num_connections() > 0:
            rospy.loginfo("publishing detection result")
            drawn_img_msg = self._detector.visualize_detection(img_msg, bounding_boxes)
            self._result_pub.publish(drawn_img_msg)

        return bounding_boxes, classes, confidences
