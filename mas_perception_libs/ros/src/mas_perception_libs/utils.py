import os
import glob
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridgeError
from sensor_msgs.msg import PointCloud2, Image as ImageMsg
from mas_perception_libs._cpp_wrapper import _cloud_msg_to_cv_image, _cloud_msg_to_image_msg,\
    _crop_organized_cloud_msg, _crop_cloud_to_xyz, _transform_point_cloud
from .bounding_box import BoundingBox2D
from .ros_message_serialization import to_cpp, from_cpp


def get_classes_in_data_dir(data_dir):
    classes = []
    for subdir in sorted(os.listdir(data_dir)):
        if os.path.isdir(os.path.join(data_dir, subdir)):
            classes.append(subdir)

    return classes


def process_image_message(image_msg, cv_bridge, target_size=None, func_preprocess_img=None):
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
    def either(c):
        return '[%s%s]' % (c.lower(), c.upper()) if c.isalpha() else c
    return glob.glob(''.join(map(either, pattern)))


def cloud_msg_to_cv_image(cloud_msg):
    if not isinstance(cloud_msg, PointCloud2):
        raise ValueError('cloud_msg is not a sensor_msgs/PointCloud2')

    serial_cloud = to_cpp(cloud_msg)
    return _cloud_msg_to_cv_image(serial_cloud)


def cloud_msg_to_image_msg(cloud_msg):
    if not isinstance(cloud_msg, PointCloud2):
        raise ValueError('cloud_msg is not a sensor_msgs/PointCloud2')

    serial_cloud = to_cpp(cloud_msg)
    serial_img_msg = _cloud_msg_to_image_msg(serial_cloud)
    return from_cpp(serial_img_msg, ImageMsg)


def crop_organized_cloud_msg(cloud_msg, bounding_box):
    if not isinstance(cloud_msg, PointCloud2):
        raise ValueError('cloud_msg is not a sensor_msgs/PointCloud2 instance')

    if not isinstance(bounding_box, BoundingBox2D):
        raise ValueError('bounding_box is not a BoundingBox2D instance')

    serial_cloud = to_cpp(cloud_msg)
    serial_cropped = _crop_organized_cloud_msg(serial_cloud, bounding_box)
    return from_cpp(serial_cropped, PointCloud2)


def crop_cloud_to_xyz(cloud_msg, bounding_box):
    if not isinstance(cloud_msg, PointCloud2):
        raise ValueError('cloud_msg is not a sensor_msgs/PointCloud2 instance')

    if not isinstance(bounding_box, BoundingBox2D):
        raise ValueError('bounding_box is not a BoundingBox2D instance')

    serial_cloud = to_cpp(cloud_msg)
    return _crop_cloud_to_xyz(serial_cloud, bounding_box)


def transform_point_cloud(cloud_msg, tf_matrix):
    if not isinstance(cloud_msg, PointCloud2):
        raise ValueError('cloud_msg is not a sensor_msgs/PointCloud2 instance')
    return from_cpp(_transform_point_cloud(to_cpp(cloud_msg), tf_matrix))
