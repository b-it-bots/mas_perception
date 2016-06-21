#!/usr/bin/env python

import rospy
import mcr_perception_msgs.msg as mpm
import std_msgs.msg
import numpy as np

def talker():
    pub = rospy.Publisher('/mcr_perception/cavity_pose_selector/cavity',\
     mpm.Cavity, queue_size=10, latch=True)
    rospy.init_node('simple_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0
    msg = mpm.Cavity()
    msg2 = mpm.Cavity()
    ###Filling the message
    msg.name ='F20_20'
    #msg.object_name = 'F20_20_B'
    msg.pose.pose.position.x = 10
    msg2.name ='S40_40'
    #msg2.object_name = 'S40_40_B'
    msg2.pose.pose.position.x = 20
    while not rospy.is_shutdown():
        if i ==0:
            pub.publish(msg2)
        if i ==30:
            pub.publish(msg)
        i = i+1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
