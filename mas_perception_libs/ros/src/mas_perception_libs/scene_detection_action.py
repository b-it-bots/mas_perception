from abc import ABCMeta, abstractmethod
import os
import numpy as np
import rospy
import tf
from dynamic_reconfigure.server import Server as ParamServer
from actionlib import SimpleActionServer
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from mcr_perception_msgs.msg import DetectSceneAction, DetectSceneResult, PlaneList, Object
from mas_perception_libs.cfg import PlaneFittingConfig
from .bounding_box import BoundingBox, BoundingBox2D
from .image_detector import ImageDetectorBase, SingleImageDetectionHandler
from .utils import PlaneSegmenter, cloud_msg_to_image_msg, transform_cloud_with_listener,\
    crop_organized_cloud_msg, crop_cloud_to_xyz
from .visualization import plane_msg_to_marker


class SceneDetectionActionServer(object):
    __metaclass__ = ABCMeta

    def __init__(self, action_name, **kwargs):
        rospy.loginfo('broadcasting action server: ' + action_name)
        # won't use default auto_start=True as recommended here: https://github.com/ros/actionlib/pull/60
        self._action_server = SimpleActionServer(action_name, DetectSceneAction,
                                                 execute_cb=self._execute_cb, auto_start=False)
        self._initialize(**kwargs)
        self._action_server.start()

    @abstractmethod
    def _initialize(self, **kwargs):
        pass

    @abstractmethod
    def _execute_cb(self, goal):
        pass

    @staticmethod
    def is_object_on_plane(plane_msg, obj_box_msg, z_tolerance=0.05):
        """
        :param z_tolerance: how much can an object be above the plane (meter)
        :type plane_msg: mcr_perception_msgs.msg.Plane
        :type obj_box_msg: mcr_perception_msgs.msg.BoundingBox
        :rtype: bool
        """
        obj_plane_z_diff = obj_box_msg.center.z - obj_box_msg.dimensions.z / 2 - plane_msg.plane_point.z
        if obj_box_msg.center.z < plane_msg.plane_point.z or obj_plane_z_diff > z_tolerance:
            return False
        # assuming positive x is far away from the robot TODO(minhnh): may not work for Jenny
        if obj_box_msg.center.x > plane_msg.limits.max_x:
            return False
        if obj_box_msg.center.y < plane_msg.limits.min_y or obj_box_msg.center.y > plane_msg.limits.max_y:
            return False
        return True


class ImageDetectionActionServer(SceneDetectionActionServer):
    _detector_handler = None            # type: SingleImageDetectionHandler
    _plane_segmenter = None             # type: PlaneSegmenter
    _plane_fitting_param_server = None  # type: ParamServer
    _cloud_topic = None                 # type: str
    _cloud_sub = None                   # type: rospy.Subscriber
    _filtered_cloud_pub = None          # type: rospy.Publisher
    _plane_marker_pub = None            # type: rospy.Publisher
    _cloud_msg = None                   # type: PointCloud2
    _tf_listener = None                 # type: tf.TransformListener
    _target_frame = None                # type: str
    _cv_bridge = None                   # type: CvBridge

    def __init__(self, action_name, **kwargs):
        super(ImageDetectionActionServer, self).__init__(action_name, **kwargs)

    def _initialize(self, **kwargs):
        detection_class = kwargs.get('detection_class', None)
        if not issubclass(detection_class, ImageDetectorBase):
            raise ValueError('"detection_class" is not of ImageDetectorBase type')

        class_annotation_file = kwargs.get('class_annotation_file', None)
        if not class_annotation_file or not os.path.exists(class_annotation_file):
            raise ValueError('invalid value for "class_annotation_file": ' + class_annotation_file)

        kwargs_file = kwargs.get('kwargs_file', None)
        if not kwargs_file or not os.path.exists(kwargs_file):
            raise ValueError('invalid value for "kwargs_file": ' + kwargs_file)

        # Plane segmentation
        self._plane_segmenter = PlaneSegmenter()
        self._plane_fitting_param_server = ParamServer(PlaneFittingConfig, self._plane_fitting_config_cb)

        # image detection
        self._detector_handler = SingleImageDetectionHandler(detection_class, class_annotation_file, kwargs_file,
                                                             '/mas_perception/detection_result')

        self._cloud_topic = kwargs.get('cloud_topic', None)
        if not self._cloud_topic:
            raise ValueError('no cloud topic specified')

        self._filtered_cloud_pub = rospy.Publisher('filtered_cloud', PointCloud2, queue_size=1)
        self._plane_marker_pub = rospy.Publisher('plane_convex_hull', Marker, queue_size=1)

        # TODO(minhnh): target_frame could be the action goal
        self._tf_listener = tf.TransformListener()
        self._target_frame = kwargs.get('target_frame', '/base_link')
        rospy.loginfo('will transform all poses to frame: ' + self._target_frame)

    def _execute_cb(self, _):
        # subscribe and wait for cloud message TODO(minhnh) add timeout
        self._cloud_sub = rospy.Subscriber(self._cloud_topic, PointCloud2, self._cloud_callback)
        while self._cloud_msg is None:
            continue

        # stop subscribing to cloud topic to avoid overhead
        self._cloud_sub.unregister()
        # reset class field for next goal request
        cloud_msg = self._cloud_msg
        self._cloud_msg = None

        rospy.loginfo('detecting objects')
        img_msg = cloud_msg_to_image_msg(cloud_msg)
        try:
            bounding_boxes, classes, confidences = self._detector_handler.process_image_msg(img_msg)
        except RuntimeError as e:
            self._action_server.set_aborted(text=e.message)
            return

        rospy.loginfo('transforming cloud to frame: ' + self._target_frame)
        try:
            transformed_cloud_msg = transform_cloud_with_listener(cloud_msg, self._target_frame, self._tf_listener)
        except RuntimeError as e:
            self._action_server.set_aborted(text=e.message)
            return

        rospy.loginfo('fitting planes')
        try:
            plane_list, filtered_cloud = self._plane_segmenter.find_planes(transformed_cloud_msg)
        except RuntimeError as e:
            self._action_server.set_aborted(text=e.message)
            return

        if self._filtered_cloud_pub.get_num_connections() > 0:
            self._filtered_cloud_pub.publish(filtered_cloud)
        if self._plane_marker_pub.get_num_connections() > 0 and len(plane_list.planes) > 0:
            marker = plane_msg_to_marker(plane_list.planes[0], 'plane_convex')
            self._plane_marker_pub.publish(marker)

        rospy.loginfo('creating action result and setting success')
        result = ImageDetectionActionServer._get_action_result(transformed_cloud_msg, plane_list, bounding_boxes,
                                                               classes, confidences)
        self._action_server.set_succeeded(result)

    def _cloud_callback(self, cloud_msg):
        self._cloud_msg = cloud_msg

    def _plane_fitting_config_cb(self, config, _):
        self._plane_segmenter.set_params(config)
        return config

    @staticmethod
    def _get_action_result(cloud_msg, plane_list, bounding_boxes, classes, confidences):
        """
        TODO(minhnh) adapt to multiple planes
        :type cloud_msg: PointCloud2
        :type plane_list: PlaneList
        :type bounding_boxes: list
        :type classes: list
        :type confidences: list
        :rtype: DetectSceneResult
        """
        result = DetectSceneResult()
        plane = plane_list.planes[0]
        for index, box in enumerate(bounding_boxes):
            # TODO(minhnh): BoundingBox code returns bad positions and dimensions
            # check if object in on plane
            cropped_cloud = crop_organized_cloud_msg(cloud_msg, box)
            normal = [plane.coefficients[0], plane.coefficients[1], plane.coefficients[2]]
            box_3d = BoundingBox(cropped_cloud, normal)
            box_msg = box_3d.get_ros_message()

            obj_coords = crop_cloud_to_xyz(cropped_cloud, BoundingBox2D(box_geometry=(0, 0, cropped_cloud.width,
                                                                                      cropped_cloud.height)))
            obj_coords = np.reshape(obj_coords, (-1, 3))
            min_coord = np.nanmin(obj_coords, axis=0)
            box_msg.center.x = min_coord[0]

            if not SceneDetectionActionServer.is_object_on_plane(plane, box_msg):
                rospy.loginfo("skipping object '%s', position (%.3f, %.3f, %.3f)"
                              % (classes[index], box_msg.center.x, box_msg.center.y, box_msg.center.z))
                continue

            rospy.loginfo("adding object '%s', position (%.3f, %.3f, %.3f), dimensions (%.3f, %.3f, %.3f)"
                          % (classes[index], box_msg.center.x, box_msg.center.y, box_msg.center.z,
                             box_msg.dimensions.x, box_msg.dimensions.y, box_msg.dimensions.z))
            # fill object fields
            detected_obj = Object()
            detected_obj.name = classes[index]
            detected_obj.probability = confidences[index]
            detected_obj.pointcloud = cropped_cloud
            detected_obj.rgb_image = cloud_msg_to_image_msg(cropped_cloud)
            detected_obj.pose = box_3d.get_pose()
            detected_obj.pose.pose.position = box_msg.center
            detected_obj.bounding_box = box_msg

            plane.object_list.objects.append(detected_obj)
        result.planes.append(plane)
        return result
