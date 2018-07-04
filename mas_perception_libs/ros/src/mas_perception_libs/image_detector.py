import os
import yaml
from abc import ABCMeta, abstractmethod
from enum import Enum
import numpy as np

import rospy
from mcr_perception_msgs.srv import DetectImage, DetectImageResponse
from mcr_perception_msgs.msg import ImageDetection, BoundingBox2D


class ImageDetectionKey(Enum):
    CLASS = 'class'
    CONF = 'confidence'
    X_MIN = 'x_min'
    X_MAX = 'x_max'
    Y_MIN = 'y_min'
    Y_MAX = 'y_max'


class ImageDetector(object):
    __metaclass__ = ABCMeta

    def __init__(self, **kwargs):
        self._classes = kwargs.get('classes', None)

        if self._classes is None:
            class_file = kwargs.get('class_file', None)
            if class_file is not None and os.path.exists(class_file):
                with open(class_file, 'r') as infile:
                    self._classes = yaml.load(infile)

        if self._classes is None:
            raise ValueError("no valid 'class_file' or 'classes' parameter specified")

        self.load_model(**kwargs)

    @property
    def classes(self):
        return self._classes

    @abstractmethod
    def load_model(self, **kwargs):
        pass

    @abstractmethod
    def detect(self, image_messages):
        pass


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


class ImageDetectionService(object):
    def __init__(self, service_name, detection_class, class_annotation_file, kwargs_file):
        if not issubclass(detection_class, ImageDetector):
            raise ValueError('detection class is not of ImageDetector type')

        if kwargs_file is None:
            kwargs = {}
        else:
            with open(kwargs_file, 'r') as infile:
                kwargs = yaml.load(infile)

        self._detector = detection_class(class_file=class_annotation_file, **kwargs)

        self._recog_service = rospy.Service(service_name, DetectImage, self.handle_detect_images)

    def handle_detect_images(self, request):
        predictions = self._detector.detect(request.images)
        response = DetectImageResponse()
        for boxes in predictions:
            detection = ImageDetection()
            for box in boxes:
                # fill class and confidence
                detection.classes.append(box[ImageDetectionKey.CLASS])
                detection.probabilities.append(box[ImageDetectionKey.CONF])
                # fill bounding box info
                bbox_2d = BoundingBox2D()
                bbox_2d.x_min = box[ImageDetectionKey.X_MIN]
                bbox_2d.y_min = box[ImageDetectionKey.Y_MIN]
                bbox_2d.x_max = box[ImageDetectionKey.X_MAX]
                bbox_2d.y_max = box[ImageDetectionKey.Y_MAX]
                detection.bounding_boxes.append(bbox_2d)

            response.detections.append(detection)

        return response
