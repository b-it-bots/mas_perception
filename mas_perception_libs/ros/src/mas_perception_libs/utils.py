import os
import glob
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridgeError
from sensor_msgs.msg import PointCloud2, Image as ImageMsg
from mas_perception_libs._cpp_wrapper import PlaneSegmenterWrapper, _cloud_msg_to_cv_image, _cloud_msg_to_image_msg,\
    _crop_organized_cloud_msg, _crop_cloud_to_xyz, _transform_point_cloud
from .bounding_box import BoundingBox2D
from .ros_message_serialization import to_cpp, from_cpp


class PlaneSegmenter(PlaneSegmenterWrapper):
    """
    Wrapper for plane segmentation code in C++
    """
    def filter_cloud(self, cloud_msg):
        """
        :type cloud_msg: PointCloud2
        :return: filtered cloud message
        :rtype: PointCloud2
        """
        serial_cloud = to_cpp(cloud_msg)
        filtered_serial_cloud = super(PlaneSegmenter, self).filter_cloud(serial_cloud)
        deserialized = from_cpp(filtered_serial_cloud, PointCloud2)
        return deserialized

    def set_params(self, param_dict):
        """
        :param param_dict: parameter dictionary returned from dynarmic reconfiguration server for PlaneFitting.cfg
        :type param_dict: dict
        :return: None
        """
        super(PlaneSegmenter, self).set_params(param_dict)


def get_classes_in_data_dir(data_dir):
    """
    :type data_dir: str
    :return: list of classes as names of top level directories
    """
    classes = []
    for subdir in sorted(os.listdir(data_dir)):
        if os.path.isdir(os.path.join(data_dir, subdir)):
            classes.append(subdir)

    return classes


def process_image_message(image_msg, cv_bridge, target_size=None, func_preprocess_img=None):
    """
    perform common image processing steps on a ROS image message

    :type image_msg: ImageMsg
    :param cv_bridge: bridge for converting sensor_msgs/Image to CV image
    :param target_size: used for resizing image
    :type target_size: tuple
    :param func_preprocess_img: optional preprocess function to call on the image
    :return: processed image as CV image
    """
    np_image = None
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        if target_size is not None:
            cv_image = cv2.resize(cv_image, target_size)
        np_image = np.asarray(cv_image)
        np_image = np_image.astype(float)                   # preprocess needs float64 and img is uint8
        if func_preprocess_img is not None:
            np_image = func_preprocess_img(np_image)        # Normalize the data
    except CvBridgeError as e:
        rospy.logerr('error converting to CV image: ' + str(e))
    return np_image


def case_insensitive_glob(pattern):
    """
    glob certain file types ignoring case

    :param pattern: file types (i.e. '*.jpg')
    :return: list of files matching given pattern
    """
    def either(c):
        return '[%s%s]' % (c.lower(), c.upper()) if c.isalpha() else c
    return glob.glob(''.join(map(either, pattern)))


def cloud_msg_to_cv_image(cloud_msg):
    """
    extract CV image from a sensor_msgs/PointCloud2 message

    :type cloud_msg: PointCloud2
    :return: extracted image as numpy array
    """
    if not isinstance(cloud_msg, PointCloud2):
        raise ValueError('cloud_msg is not a sensor_msgs/PointCloud2')

    serial_cloud = to_cpp(cloud_msg)
    return _cloud_msg_to_cv_image(serial_cloud)


def cloud_msg_to_image_msg(cloud_msg):
    """
    extract sensor_msgs/Image from a sensor_msgs/PointCloud2 message

    :type cloud_msg: PointCloud2
    :return: extracted image message
    :rtype: ImageMsg
    """
    if not isinstance(cloud_msg, PointCloud2):
        raise ValueError('cloud_msg is not a sensor_msgs/PointCloud2')

    serial_cloud = to_cpp(cloud_msg)
    serial_img_msg = _cloud_msg_to_image_msg(serial_cloud)
    return from_cpp(serial_img_msg, ImageMsg)


def crop_organized_cloud_msg(cloud_msg, bounding_box):
    """
    crop a sensor_msgs/PointCloud2 message using info from a BoundingBox2D object

    :type cloud_msg: PointCloud2
    :type bounding_box: BoundingBox2D
    :return: cropped cloud
    :rtype: PointCloud2
    """
    if not isinstance(cloud_msg, PointCloud2):
        raise ValueError('cloud_msg is not a sensor_msgs/PointCloud2 instance')

    if not isinstance(bounding_box, BoundingBox2D):
        raise ValueError('bounding_box is not a BoundingBox2D instance')

    serial_cloud = to_cpp(cloud_msg)
    serial_cropped = _crop_organized_cloud_msg(serial_cloud, bounding_box)
    return from_cpp(serial_cropped, PointCloud2)


def crop_cloud_to_xyz(cloud_msg, bounding_box):
    """
    extract a sensor_msgs/PointCloud2 message for 3D coordinates using info from a BoundingBox2D object

    :type cloud_msg: PointCloud2
    :type bounding_box: BoundingBox2D
    :return: (x, y, z) coordinates within the bounding box
    :rtype: ndarray
    """
    if not isinstance(cloud_msg, PointCloud2):
        raise ValueError('cloud_msg is not a sensor_msgs/PointCloud2 instance')

    if not isinstance(bounding_box, BoundingBox2D):
        raise ValueError('bounding_box is not a BoundingBox2D instance')

    serial_cloud = to_cpp(cloud_msg)
    return _crop_cloud_to_xyz(serial_cloud, bounding_box)


def transform_point_cloud(cloud_msg, tf_matrix, target_frame):
    """
    transform a sensor_msgs/PointCloud2 message using a transformation matrix

    :type cloud_msg: PointCloud2
    :param tf_matrix: transformation matrix
    :type tf_matrix: ndarray
    :param target_frame: frame to be transformed to
    :type target_frame: str
    :return transformed cloud
    :rtype PointCloud2
    """
    if not isinstance(cloud_msg, PointCloud2):
        raise ValueError('cloud_msg is not a sensor_msgs/PointCloud2 instance')
    transformed_cloud = from_cpp(_transform_point_cloud(to_cpp(cloud_msg), tf_matrix), PointCloud2)
    transformed_cloud.header.frame_id = target_frame
    return transformed_cloud
