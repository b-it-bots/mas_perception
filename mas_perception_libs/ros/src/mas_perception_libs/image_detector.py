import os
from abc import ABCMeta, abstractmethod
import yaml
from enum import Enum
import numpy as np

import rospy
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge

from .bounding_box import BoundingBox2D
from .visualization import bgr_dict_from_classes, draw_labeled_boxes_img_msg


class ImageDetectionKey(Enum):
    CLASS = 0
    CONF = 1
    X_MIN = 2
    X_MAX = 3
    Y_MIN = 4
    Y_MAX = 5


class ImageDetector(object):
    __metaclass__ = ABCMeta
    _classes = None         # type: dict
    _class_colors = None    # type: dict

    def __init__(self, **kwargs):
        # load dictionary of classes
        self._classes = kwargs.get('classes', None)
        if self._classes is None:
            class_file = kwargs.get('class_file', None)
            if class_file is not None and os.path.exists(class_file):
                with open(class_file, 'r') as infile:
                    self._classes = yaml.load(infile)

        if self._classes is None:
            raise ValueError("no valid 'class_file' or 'classes' parameter specified")
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
        return self._classes

    @property
    def class_colors(self):
        return self._class_colors

    @abstractmethod
    def load_model(self, **kwargs):
        pass

    @abstractmethod
    def detect(self, image_messages):
        pass

    @staticmethod
    def prediction_to_bounding_boxes(prediction, color_dict=None):
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


class ImageDetectorTest(ImageDetector):
    def __init__(self, **kwargs):
        self._min_box_ratio = None
        self._max_num_detection = None
        super(ImageDetectorTest, self).__init__(**kwargs)

    def load_model(self, **kwargs):
        self._min_box_ratio = kwargs.get('min_box_ratio', 0.2)
        self._max_num_detection = kwargs.get('max_num_detection', 7)

    def detect(self, image_messages):
        predictions = []
        for image_msg in image_messages:
            boxes = []
            min_box_width = int(image_msg.width * self._min_box_ratio)
            min_box_height = int(image_msg.height * self._min_box_ratio)
            num_detection = np.random.randint(1, self._max_num_detection)
            for _ in range(1, num_detection + 1):
                # generate random class and confidence
                detected_class = self._classes[np.random.choice(self._classes.keys())]
                confidence = int(np.random.rand() * 100) / 100.

                # calculate random box
                x_min = np.random.randint(image_msg.width - min_box_width)
                y_min = np.random.randint(image_msg.height - min_box_height)

                width = np.random.randint(min_box_width, image_msg.width - x_min)
                height = np.random.randint(min_box_height, image_msg.height - y_min)

                x_max = x_min + width
                y_max = y_min + height

                # create box dictionary
                box_dict = {ImageDetectionKey.CLASS: detected_class, ImageDetectionKey.CONF: confidence,
                            ImageDetectionKey.X_MIN: x_min, ImageDetectionKey.Y_MIN: y_min,
                            ImageDetectionKey.X_MAX: x_max, ImageDetectionKey.Y_MAX: y_max}
                boxes.append(box_dict)

            predictions.append(boxes)

        return predictions


class ImageDetectorROS(object):
    _detector = None    # type: ImageDetector
    _result_pub = None  # type: rospy.Publisher
    _cv_bridge = None   # type: CvBridge

    def __init__(self, detection_class, class_annotation_file, kwargs_file, result_topic):
        self._detector = detection_class(class_file=class_annotation_file, model_kwargs_file=kwargs_file)
        self._result_pub = rospy.Publisher(result_topic, ImageMsg, queue_size=1)
        self._cv_bridge = CvBridge()

    def process_image_msg(self, img_msg):
        predictions = self._detector.detect([img_msg])
        if len(predictions) < 1:
            raise RuntimeError('no prediction returned for image message')
        bounding_boxes, classes, confidences = ImageDetector.prediction_to_bounding_boxes(predictions[0],
                                                                                          self._detector.class_colors)
        if self._result_pub.get_num_connections() > 0:
            rospy.loginfo("publishing detection result")
            drawn_img_msg = draw_labeled_boxes_img_msg(self._cv_bridge, img_msg, bounding_boxes)
            self._result_pub.publish(drawn_img_msg)

        return bounding_boxes, classes, confidences
