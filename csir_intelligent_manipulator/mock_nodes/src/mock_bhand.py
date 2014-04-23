#! /usr/bin/env python

# Copyright (c) 2014 csir_intelligent_manipulator authors. All rights reserved.
# Use of this source code is governed by the MIT license that can be found in
# the LICENSE file.
# Author: Jimmy Kizito

""" mock_bhand mocks Barrett Hand

Classes:
    MockBHand: mock hand
"""

import roslib
roslib.load_manifest("mock_nodes")
import rospy
import sensor_msgs.msg
import wam_srvs.srv
from random import random

class MockBHand:
    _HAND_DOF = 7
    _HAND_RATE = 10
    _PROXIMAL_JOINTS = [0, 1, 2]
    def __init__(self):
        self._bhand_joint_states = [0 for i in range(MockBHand._HAND_DOF)]
        self._hand_joint_pub = rospy.Publisher("bhand/joint_states", sensor_msgs.msg.JointState)
        self._hand_pub_rate = rospy.Rate(MockBHand._HAND_RATE)
        self._bhand_spread_srv = rospy.Service("bhand/spread_pos",
                                               wam_srvs.srv.BHandSpreadPos,
                                               self.mock_bhand_spread)
        self._bhand_grasp_srv = rospy.Service("bhand/grasp_pos",
                                               wam_srvs.srv.BHandGraspPos,
                                               self.mock_bhand_grasp)

    def pub_hand(self):
        while not rospy.is_shutdown():
            joint_state_msg = sensor_msgs.msg.JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ["" for i in range(MockBHand._HAND_DOF)]
            joint_state_msg.position = self._bhand_joint_states
            joint_state_msg.velocity = [0 for i in range(MockBHand._HAND_DOF)]
            joint_state_msg.effort = [0 for i in range(MockBHand._HAND_DOF)]
            self._hand_joint_pub.publish(joint_state_msg)
            self._hand_pub_rate.sleep()

    def mock_bhand_spread(self, req):
        rospy.loginfo("Mock spreading: " + str(req.radians))
        self._bhand_joint_states[3] = req.radians # 3 is index of spread angle
        return wam_srvs.srv.BHandSpreadPosResponse()

    def mock_bhand_grasp(self, req):
        rospy.loginfo("Mock grasping: " + str(req.radians))
        for i in MockBHand._PROXIMAL_JOINTS:
            self._bhand_joint_states[i] = req.radians
        rospy.loginfo("grasp: " + str(self._bhand_joint_states))
        return wam_srvs.srv.BHandGraspPosResponse()

if __name__ == "__main__":
    try:
        rospy.init_node("mock_bhand_node")
        mock_bhand = MockBHand()
        mock_bhand.pub_hand()
    except rospy.ROSInterruptException:
        pass

