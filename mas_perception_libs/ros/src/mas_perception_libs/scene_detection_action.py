from abc import ABCMeta, abstractmethod
import rospy
from actionlib import SimpleActionServer
from mcr_perception_msgs.msg import DetectSceneAction, DetectSceneResult, Plane


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


class SceneDetectionActionServerTest(SceneDetectionActionServer):
    def __init__(self, action_name, **kwargs):
        super(SceneDetectionActionServerTest, self).__init__(action_name, **kwargs)

    def _initialize(self, **kwargs):
        pass

    def _execute_cb(self, _):
        # generate fake results
        result = DetectSceneResult()
        result.planes.append(Plane())
        self._action_server.set_succeeded(result)
