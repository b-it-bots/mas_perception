from abc import ABCMeta, abstractmethod
import os
import rospy
import tf
from actionlib import SimpleActionServer
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from mcr_perception_msgs.msg import DetectSceneAction, DetectSceneResult, Plane, Object
from .image_detector import ImageDetectorBase, SingleImageDetectionHandler
from .utils import cloud_msg_to_image_msg, transform_point_cloud, crop_organized_cloud_msg


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


class ImageDetectionActionServer(SceneDetectionActionServer):
    _detector_handler = None    # type: SingleImageDetectionHandler
    _cloud_topic = None         # type: str
    _cloud_sub = None           # type: rospy.Subscriber
    _cloud_msg = None           # type: PointCloud2
    _tf_listener = None         # type: tf.TransformListener
    _target_frame = None        # type: str
    _cv_bridge = None           # type: CvBridge

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

        self._detector_handler = SingleImageDetectionHandler(detection_class, class_annotation_file, kwargs_file,
                                                             '/mas_perception/detection_result')

        self._cloud_topic = kwargs.get('cloud_topic', None)
        if not self._cloud_topic:
            raise ValueError('no cloud topic specified')

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
            transformed_cloud_msg = self._transform_cloud(cloud_msg)
        except RuntimeError as e:
            self._action_server.set_aborted(text=e.message)
            return

        rospy.loginfo('creating action result and setting success')
        result = ImageDetectionActionServer._get_action_result(transformed_cloud_msg, bounding_boxes,
                                                               classes, confidences)
        self._action_server.set_succeeded(result)

    def _cloud_callback(self, cloud_msg):
        self._cloud_msg = cloud_msg

    def _transform_cloud(self, cloud_msg):
        """ TODO(minhnh) change this with transform_cloud_with_listener()
        """
        try:
            common_time = self._tf_listener.getLatestCommonTime(self._target_frame, cloud_msg.header.frame_id)
            cloud_msg.header.stamp = common_time
            self._tf_listener.waitForTransform(self._target_frame, cloud_msg.header.frame_id,
                                               cloud_msg.header.stamp, rospy.Duration(1))
            tf_matrix = self._tf_listener.asMatrix(self._target_frame, cloud_msg.header)
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            raise RuntimeError('Unable to transform {0} -> {1}'.format(cloud_msg.header.frame_id, self._target_frame))

        return transform_point_cloud(cloud_msg, tf_matrix, self._target_frame)

    @staticmethod
    def _get_action_result(cloud_msg, bounding_boxes, classes, confidences):
        """
        :type cloud_msg: PointCloud2
        :type bounding_boxes: list
        :type classes: list
        :type confidences: list
        :rtype: DetectSceneResult
        """
        result = DetectSceneResult()
        plane = Plane()
        for index, box in enumerate(bounding_boxes):
            detected_obj = Object()
            detected_obj.name = classes[index]
            detected_obj.probability = confidences[index]
            cropped_cloud = crop_organized_cloud_msg(cloud_msg, box)
            detected_obj.pointcloud = cropped_cloud
            detected_obj.rgb_image = cloud_msg_to_image_msg(cropped_cloud)
            plane.object_list.objects.append(detected_obj)
        result.planes.append(plane)
        return result
