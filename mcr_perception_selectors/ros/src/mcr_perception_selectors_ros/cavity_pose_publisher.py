#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This module contains a component that monitors stores the poses of the
cavities and publishes the pose of cavity in which the object can be 
put in, on trigger.

**Input(s):**
  * cavity: cavity message for the identified cavities.
  * object_name: Name of the object for which the cavity pose id needed.
  * event : To trigger the run state of the node. (it assumes that teh object pose
    is already published.)

**Output(s):**
  * `cavity_pose`: The pose of the cavity.

**Relevant parameter(s):**
  * loop_rate : Loop rate of the node.
"""

import rospy
import mcr_perception_msgs.msg as mpm
import geometry_msgs.msg
import std_msgs.msg

__author__ = 'padmaja'


class CavityPosePublisher(object):
    """
    Publishes the pose of the cavity in which the requested object can be dropped.

    """
    def __init__(self):
        # params
        self.event = None
        self.cavity_msg_array = []
        self.object_name = None

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        #publishers
        self.cavity_pose_pub = rospy.Publisher("~cavity_pose", geometry_msgs.msg.PoseStamped,\
            queue_size=10)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~cavity", mpm.Cavity, self.cavity_cb)
        rospy.Subscriber("~object_name", std_msgs.msg.String, self.object_name_cb)

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def cavity_cb(self, msg):
        """
        Obtains the first pose.

        """
        self.cavity_msg_array.append(msg)

    def object_name_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.object_name = msg.data

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.object_name is not None:
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start' :
            return 'RUNNING'
        elif self.event == 'e_stop':
            self.object_name = None
            self.event = None
            return 'INIT'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.object_name = None
            self.event = None
            return 'INIT'
        else:
            self.event = None
            for idx, cavity in enumerate(self.cavity_msg_array):
                if cavity.object_name == self.object_name:
                    self.cavity_pose_pub.publish(cavity.pose)
                    break
            return 'IDLE'

def main():
    rospy.init_node('cavity_pose_publisher', anonymous=True)
    cavity_pose_publisher = CavityPosePublisher()
    cavity_pose_publisher.start()