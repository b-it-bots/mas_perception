import rospy
import tf
from actionlib import SimpleActionClient
from geometry_msgs.msg import PointStamped
from mcr_perception_msgs.msg import PlaneList, DetectSceneGoal, DetectSceneAction
from .bounding_box import BoundingBox


class ObjectDetector(object):
    """
    Interacts with a DetectScene action server to detect objects and process them
    """
    _detection_client = None    # type: SimpleActionClient
    _plane_list = PlaneList()   # type: PlaneList
    _timeout = None             # type: int

    def __init__(self, detection_action_name, timeout=20):
        """
        :param detection_action_name: name of action server
        :param timeout: maximum to wait for the action to start or process the goal
        """
        self._timeout = timeout
        self._detection_client = SimpleActionClient(detection_action_name, DetectSceneAction)
        if not self._detection_client.wait_for_server(timeout=rospy.Duration(self._timeout)):
            raise RuntimeError('failed to wait for detection action server after {0} seconds: {1}'
                               .format(self._timeout, detection_action_name))

        self._tf_listener = tf.TransformListener()

    def start_detect_objects(self, plane_frame_prefix, done_callback, target_frame=None, group_planes=True):
        """
        Detect and process objects
        TODO(minhnh) cleanup pose calculation as should be handled in the action server

        :param plane_frame_prefix: str to prepend to plane name
        :param done_callback: function to execute at the end of this method before returning
        :param target_frame: frame to transform object poses to
        :param group_planes: if True group planes close to each other into a single one containing all the objects
        :return: None
        """
        goal = DetectSceneGoal()
        self._detection_client.send_goal(goal)
        if not self._detection_client.wait_for_result(rospy.Duration(self._timeout)):
            rospy.logwarn('action server did not respond after {0} seconds, returning'.format(self._timeout))
            return

        self._plane_list.planes = self._detection_client.get_result().planes

        # transform if target_frame is specified (i.e. not None or empty string)
        if target_frame:
            rospy.loginfo('will transform objects and plane to target frame: ' + str(target_frame))
            for plane in self._plane_list.planes:
                # assuming only mcr_perception_msgs/Plane.pose is used
                plane.pose = self._transform_plane(plane.pose, target_frame)

        if group_planes:
            planes = ObjectDetector.group_planes_by_height(self._plane_list.planes)
        else:
            planes = self._plane_list.planes

        plane_index = 0
        for plane in planes:
            # write plane frame as name
            plane_frame = '{0}_{1}'.format(plane_frame_prefix, plane_index)
            plane.name = plane_frame
            plane_index += 1
            plane_quart = plane.pose.pose.orientation

            # make bounding boxes
            normal = [plane_quart.x, plane_quart.y, plane_quart.z]
            for detected_obj in plane.object_list.objects:
                bounding_box = BoundingBox(detected_obj.pointcloud, normal)
                obj_pose = bounding_box.get_pose()
                bounding_box_msg = bounding_box.get_ros_message()
                if target_frame:
                    bounding_box_msg, obj_pose = self._transform_object(bounding_box_msg, obj_pose, target_frame)

                detected_obj.pose = obj_pose
                detected_obj.bounding_box = bounding_box_msg

            rospy.loginfo('found plane "{0}", height {1} in frame_id {2}, with {3} objects'
                          .format(plane_frame, plane.pose.pose.position.z, plane.pose.header.frame_id,
                                  len(plane.object_list.objects)))

        done_callback()
        return

    def _transform_plane(self, plane_pose, target_frame):
        """ Transform plane pose to a target frame """
        try:
            common_time = self._tf_listener.getLatestCommonTime(target_frame, plane_pose.header.frame_id)
            plane_pose.header.stamp = common_time
            self._tf_listener.waitForTransform(target_frame, plane_pose.header.frame_id,
                                               plane_pose.header.stamp, rospy.Duration(1))

            plane_pose = self._tf_listener.transformPose(target_frame, plane_pose)
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Unable to transform %s -> %s' % (plane_pose.header.frame_id, target_frame))

        return plane_pose

    def _transform_object(self, box_msg, obj_pose, target_frame):
        """ Transform object poses and box message to a target frame """
        try:
            common_time = self._tf_listener.getLatestCommonTime(target_frame, obj_pose.header.frame_id)
            obj_pose.header.stamp = common_time
            self._tf_listener.waitForTransform(target_frame, obj_pose.header.frame_id,
                                               obj_pose.header.stamp, rospy.Duration(1))
            # transform object pose
            old_header = obj_pose.header
            obj_pose = self._tf_listener.transformPose(target_frame, obj_pose)
            # box center is the object position as defined in bounding_box_wrapper.cpp
            box_msg.center = obj_pose.pose.position

            # transform box vertices
            for vertex in box_msg.vertices:
                stamped = PointStamped()
                stamped.header = old_header
                stamped.point = vertex
                transformed = self._tf_listener.transformPoint(target_frame, stamped)
                vertex.x = transformed.point.x
                vertex.y = transformed.point.y
                vertex.z = transformed.point.z

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Unable to transform %s -> %s' % (obj_pose.header.frame_id, target_frame))

        return box_msg, obj_pose

    @property
    def plane_list(self):
        """ list of planes containing objects """
        return self._plane_list

    @staticmethod
    def group_planes_by_height(planes, group_threshold=0.1):
        """ group planes close together into one """
        plane_index = 0
        plane_group_heights = {}    # index: height
        plane_groups = {}           # index: list of indices

        for plane in planes:
            if len(plane_group_heights) == 0:
                plane_group_heights[0] = [plane.pose.pose.position.z]
                plane_groups[0] = [plane_index]
            else:
                grouped = False
                for index in plane_group_heights:
                    avg_height = sum(plane_group_heights[index])/len(plane_group_heights[index])
                    if abs(plane.pose.pose.position.z - avg_height) < group_threshold:
                        rospy.loginfo('grouping plane {0} with planes {1}'
                                      .format(plane_index, plane_groups[index]))
                        plane_groups[index].append(plane_index)
                        plane_group_heights[index].append(plane.pose.pose.position.z)
                        grouped = True
                        break
                    pass

                if not grouped:
                    new_index = max(plane_group_heights.keys()) + 1
                    plane_group_heights[new_index] = [plane.pose.pose.position.z]
                    plane_groups[new_index] = [plane_index]
                    pass
                pass
            plane_index += 1
            pass

        grouped_planes = []
        for index in plane_groups:
            plane = planes[plane_groups[index][0]]
            avg_height = sum(plane_group_heights[index])/len(plane_group_heights[index])
            plane.pose.pose.position.z = avg_height
            for plane_index in plane_groups[index][1:]:
                plane.object_list.objects.extend(planes[plane_index].object_list.objects)

            grouped_planes.append(plane)

        return grouped_planes
