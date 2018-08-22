from actionlib import SimpleActionServer
from mcr_perception_msgs.msg import DetectSceneAction, DetectSceneResult, Plane, Object


class SceneDetectionTestActionServer(object):
    def __init__(self, node_name, num_objects=3):
        self._num_objects = num_objects
        # won't use default auto_start=True as recommended here: https://github.com/ros/actionlib/pull/60
        self._action_server = SimpleActionServer(node_name, DetectSceneAction,
                                                 execute_cb=self._execute_cb, auto_start=False)
        self._action_server.start()

    def _execute_cb(self, _):
        # generate fake results
        result = DetectSceneResult()
        result.planes.append(Plane())
        for _ in range(self._num_objects):
            result.planes[0].object_list.objects.append(Object())

        self._action_server.set_succeeded(result)
