from abc import ABCMeta, abstractmethod
import rospy
from actionlib import SimpleActionServer
from mcr_perception_msgs.msg import DetectSceneAction, DetectSceneResult, Plane, Object


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


class SceneDetectionTestActionServer(SceneDetectionActionServer):
    _num_objects = None  # type: int

    def __init__(self, action_name, num_objects=3, **kwargs):
        super(SceneDetectionTestActionServer, self).__init__(action_name, **kwargs)
        self._num_objects = num_objects

    def _initialize(self, **kwargs):
        pass

    def _execute_cb(self, _):
        # generate fake results
        result = DetectSceneResult()
        result.planes.append(Plane())
        for _ in range(self._num_objects):
            result.planes[0].object_list.objects.append(Object())

        self._action_server.set_succeeded(result)
