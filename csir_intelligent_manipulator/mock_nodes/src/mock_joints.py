#! /usr/bin/env python

# Copyright (c) 2014 csir_intelligent_manipulator authors. All rights reserved.
# Use of this source code is governed by the MIT license that can be found in
# the LICENSE file.
# Author: Jimmy Kizito

""" mock_wam_joints publishes mock joint states for the WAM.

Classes:
    MockWamJoints: publishes WAM joint states.
"""

import roslib
roslib.load_manifest("mock_nodes")
import rospy
import sensor_msgs.msg
from threading import Thread
from random import random

class MockWamJoints:
    _WAM_DOF = 7
    _HAND_DOF = 7
    _WAM_RATE = 20
    _HAND_RATE = 10
    def __init__(self):
        self._wam_joint_pub = rospy.Publisher("wam/joint_states", sensor_msgs.msg.JointState)
        self._hand_joint_pub = rospy.Publisher("bhand/joint_states", sensor_msgs.msg.JointState)
        self._pub_wam_thread = Thread(target = self.pub_wam)
        self._pub_hand_thread = Thread(target = self.pub_hand)
        self._wam_pub_rate = rospy.Rate(MockWamJoints._WAM_RATE)
        self._hand_pub_rate = rospy.Rate(MockWamJoints._HAND_RATE)
        self._pub_wam_thread.start()
        self._pub_hand_thread.start()

    def pub_wam(self):
        while not rospy.is_shutdown():
            joint_state_msg = sensor_msgs.msg.JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ["" for i in range(MockWamJoints._WAM_DOF)]
            joint_state_msg.position = [0 for i in range(MockWamJoints._WAM_DOF)]
            joint_state_msg.velocity = [0 for i in range(MockWamJoints._WAM_DOF)]
            joint_state_msg.effort = [0 for i in range(MockWamJoints._WAM_DOF)]
            self._wam_joint_pub.publish(joint_state_msg)
            self._wam_pub_rate.sleep()

    def pub_hand(self):
        while not rospy.is_shutdown():
            joint_state_msg = sensor_msgs.msg.JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ["" for i in range(MockWamJoints._HAND_DOF)]
            joint_state_msg.position = [0 for i in range(MockWamJoints._HAND_DOF)]
            joint_state_msg.velocity = [0 for i in range(MockWamJoints._HAND_DOF)]
            joint_state_msg.effort = [0 for i in range(MockWamJoints._HAND_DOF)]
            self._hand_joint_pub.publish(joint_state_msg)
            self._hand_pub_rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("mock_joints_node")
        mock_wam_joints = MockWamJoints()
        #mock_wam_joints.pub_joints()
    except rospy.ROSInterruptException:
        pass

