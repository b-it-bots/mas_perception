import os
from abc import ABCMeta, abstractmethod

import numpy as np
import rospy
import yaml
from enum import Enum
from mcr_perception_msgs.msg import ImageDetection, BoundingBox2D as BoundingBox2DMsg
from mcr_perception_msgs.srv import DetectImage, DetectImageResponse
from .bounding_box import BoundingBox2D

from .detection_service_proxy import DetectionServiceProxy


class ImageDetectionKey(Enum):
    CLASS = 0
    CONF = 1
    X_MIN = 2
    X_MAX = 3
    Y_MIN = 4
    Y_MAX = 5


class ImageDetector(object):
    __metaclass__ = ABCMeta

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

    @abstractmethod
    def load_model(self, **kwargs):
        pass

    @abstractmethod
    def detect(self, image_messages):
        pass

    @staticmethod
    def prediction_to_detection_msg(predicted_boxes):
        detection_msg = ImageDetection()
        for box in predicted_boxes:
            # fill class and confidence
            detection_msg.classes.append(box[ImageDetectionKey.CLASS])
            detection_msg.probabilities.append(box[ImageDetectionKey.CONF])
            # fill bounding box info
            bbox_2d = BoundingBox2DMsg()
            bbox_2d.x_min = box[ImageDetectionKey.X_MIN]
            bbox_2d.y_min = box[ImageDetectionKey.Y_MIN]
            bbox_2d.x_max = box[ImageDetectionKey.X_MAX]
            bbox_2d.y_max = box[ImageDetectionKey.Y_MAX]
            detection_msg.bounding_boxes.append(bbox_2d)

        return detection_msg

    @staticmethod
    def detection_msg_to_bounding_boxes(detection_msg, color_dict=None):
        boxes = []
        for i, class_name in enumerate(detection_msg.classes):
            box_geometry = (detection_msg.bounding_boxes[i].x_min,
                            detection_msg.bounding_boxes[i].y_min,
                            detection_msg.bounding_boxes[i].x_max - detection_msg.bounding_boxes[i].x_min,
                            detection_msg.bounding_boxes[i].y_max - detection_msg.bounding_boxes[i].y_min)
            label = '{}: {:.2f}'.format(class_name, detection_msg.probabilities[i])
            if color_dict is None:
                color = (0, 0, 255)     # default color: blue
            else:
                color = color_dict[class_name]
            bounding_box = BoundingBox2D(label, color, box_geometry)
            boxes.append(bounding_box)

        return boxes


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
    """
    Interacts with ImageDetector class, only contain 2D bounding boxes
    """
    def __init__(self, service_name, detection_class, class_annotation_file, kwargs_file):
        if not issubclass(detection_class, ImageDetector):
            raise ValueError('detection class is not of ImageDetector type')

        self._detector = detection_class(class_file=class_annotation_file, model_kwargs_file=kwargs_file)
        self._detection_service = rospy.Service(service_name, DetectImage, self.handle_detect_images)

    def handle_detect_images(self, request):
        predictions = self._detector.detect(request.images)
        response = DetectImageResponse()
        for boxes in predictions:
            detection_msg = ImageDetector.prediction_to_detection_msg(boxes)
            response.detections.append(detection_msg)

        return response


class ImageDetectionServiceProxy(DetectionServiceProxy):
    """
    Extends DetectionServiceProxy, use ImageDetector to detect objects in image, using XYZRGB point cloud, returns
    a list of objects
    """
    def __init__(self, service_name, detection_class, class_annotation_file, kwargs_file, point_cloud_topic):
        super(ImageDetectionServiceProxy, self).__init__(service_name, DetectImage)
        if not issubclass(detection_class, ImageDetector):
            raise ValueError('detection class is not of ImageDetector type')

        self._detector = detection_class(class_file=class_annotation_file, model_kwargs_file=kwargs_file)
        self._cloud_topic = point_cloud_topic

    def _get_objects_and_planes_from_response(self, res):
        # visualize detection result
        # get image crops
        # get cloud crops
        # calculate pose
        # fill some fake plane
        # return plane_list
        pass

    def _get_segmentation_req(self):
        # subscribe to point cloud topic
        # wait until cloud is available/timeout
        # extract image from cloud topic, create image message
        # unsubscribe from topic
        # return request with image message
        pass
