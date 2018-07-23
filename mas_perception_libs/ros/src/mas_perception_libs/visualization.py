import numpy as np
import cv2
from cv_bridge import CvBridgeError

from mas_perception_libs._cpp_wrapper import _draw_labeled_boxes, _fit_box_to_image, _crop_image
from .bounding_box import BoundingBox2DWrapper


def draw_labeled_boxes(image, boxes, thickness=2, font_scale=1.0, copy=True):
    if not isinstance(image, np.ndarray):
        raise ValueError('image is not a numpy array')

    if copy:
        drawn_image = image.copy()
    else:
        drawn_image = image

    return _draw_labeled_boxes(drawn_image, boxes, thickness, font_scale)


def draw_labeled_boxes_img_msg(cv_bridge, img_msg, boxes, thickness=2, font_scale=1.0, copy=True):
    try:
        img = cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
    except CvBridgeError as e:
        raise RuntimeError('failed to convert from ROS message to CV image: ' + e.message)

    drawn_image = draw_labeled_boxes(img, boxes, thickness, font_scale, copy)

    try:
        drawn_img_msg = cv_bridge.cv2_to_imgmsg(drawn_image, 'bgr8')
    except CvBridgeError as e:
        raise RuntimeError('failed to convert from CV image to ROS message: ' + e.message)

    return drawn_img_msg


def fit_box_to_image(image_size, bounding_box, offset=0):
    if not isinstance(image_size, tuple) or len(image_size) != 2:
        raise ValueError('image size is not a tuple of length 2')

    if not isinstance(bounding_box, BoundingBox2DWrapper):
        raise ValueError('bounding box object is not of type mas_perception_libs.BoundingBox2D')

    return _fit_box_to_image(image_size, bounding_box, offset)


def crop_image(image, bounding_box, offset=0):
    if not isinstance(image, np.ndarray):
        raise ValueError('image is not a numpy array')

    if not isinstance(bounding_box, BoundingBox2DWrapper):
        raise ValueError('bounding box object is not of type mas_perception_libs.BoundingBox2D')

    return _crop_image(image, bounding_box, offset)


def bgr_dict_from_classes(classes):
    """
    get dictionary of BGR colors span over HSV hue range from a list of classes

    :param classes: list of colors
    :return: dictionary {key=class name, value=color}
    """
    class_num = len(classes)
    np.random.seed(1234)
    np.random.shuffle(classes)

    hue = (np.linspace(0, 1, class_num) * 255).astype(np.uint8).reshape(1, class_num, 1)
    sat = np.ones((1, class_num, 1), dtype=np.uint8) * 127      # saturation 50%
    bright = np.ones((1, class_num, 1), dtype=np.uint8) * 127   # value/brightness 50%

    hsv = np.concatenate((hue, sat, bright), axis=2)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR).astype(int)

    return {class_name: tuple(bgr[0][index]) for index, class_name in enumerate(classes)}