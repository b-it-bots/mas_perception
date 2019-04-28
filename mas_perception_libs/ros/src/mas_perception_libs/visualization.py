import numpy as np
import cv2
from cv_bridge import CvBridgeError
from visualization_msgs.msg import Marker

from mcr_perception_msgs.msg import Plane as PlaneMsg
from mas_perception_libs._cpp_wrapper import _draw_labeled_boxes, _fit_box_to_image, _crop_image, _plane_msg_to_marker
from .bounding_box import BoundingBox2DWrapper
from .ros_message_serialization import from_cpp, to_cpp


def draw_labeled_boxes(image, boxes, thickness=2, font_scale=1.0, copy=True):
    """
    Draw bounding boxes on a CV image using BoundingBox2D objects

    :param image: CV image as ndarray
    :param boxes: list of BoundingBox2D objects
    :param thickness: line thickness
    :param font_scale:
    :param copy: if True copy CV image before drawing; by default image will be drawn over
    :return: image with bounding boxes visualized
    """
    if not isinstance(image, np.ndarray):
        raise ValueError('image is not a numpy array')

    if copy:
        drawn_image = image.copy()
    else:
        drawn_image = image

    return _draw_labeled_boxes(drawn_image, boxes, thickness, font_scale)


def draw_labeled_boxes_img_msg(cv_bridge, img_msg, boxes, thickness=2, font_scale=1.0, copy=True):
    """
    Draw bounding boxes on a sensor_msgs/Image message using BoundingBox2D objects; call draw_labeled_boxes()

    :type cv_bridge: cv_bridge.CvBridge
    :type img_msg: sensor_msgs.msg.Image
    :param boxes: list of BoundingBox2D objects
    :param thickness: line thickness
    :param font_scale:
    :param copy: like with draw_labeled_boxes()
    :return: image message with bounding boxes visualized
    :rtype: sensor_msgs.img.Image
    """
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
    """
    Adjust bounding box to image size

    :param image_size: 2-tuple (width, height)
    :param bounding_box: BoundingBox2D object
    :param offset: used to expand bounding box's dimensions
    :return: adjusted BoundingBox2D object
    """
    if not isinstance(image_size, tuple) or len(image_size) != 2:
        raise ValueError('image size is not a tuple of length 2')

    if not isinstance(bounding_box, BoundingBox2DWrapper):
        raise ValueError('bounding box object is not of type mas_perception_libs.BoundingBox2D')

    return _fit_box_to_image(image_size, bounding_box, offset)


def crop_image(image, bounding_box, offset=0):
    """
    Crop image using BoundingBox2D object

    :param image: CV image as ndarray
    :param bounding_box: BoundingBox2D object
    :param offset: used to expand bounding box's dimensions
    :return: cropped image
    """
    if not isinstance(image, np.ndarray):
        raise ValueError('image is not a numpy array')

    if not isinstance(bounding_box, BoundingBox2DWrapper):
        raise ValueError('bounding box object is not of type mas_perception_libs.BoundingBox2D')

    return _crop_image(image, bounding_box, offset)


def bgr_dict_from_classes(classes):
    """
    get dictionary of BGR colors span over HSV hue range from a list of classes

    :param classes: list of strings containing class names
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


def plane_msg_to_marker(plane_msg, namespace):
    """
    :type plane_msg: PlaneMsg
    :type namespace: str
    :rtype: Marker
    """
    serialized_plane = to_cpp(plane_msg)
    serialized_marker = _plane_msg_to_marker(serialized_plane, namespace)
    return from_cpp(serialized_marker, Marker)
