#! /usr/bin/env python

# Copyright (c) 2014 csir_intelligent_manipulator authors. All rights reserved.
# Use of this source code is governed by the MIT license that can be found in
# the LICENSE file.
# Author: Jimmy Kizito

""" mock_wam mocks WAM

Classes:
    MockWam mock WAM
"""

import roslib
roslib.load_manifest("mock_nodes")
import rospy
import sensor_msgs.msg
import wam_srvs.srv

class MockWam:
    _WAM_DOF = 7
    _WAM_RATE = 20
    def __init__(self):
        self._wam_joint_states = [0 for i in range(MockWam._WAM_DOF)]
        self._wam_joint_pub = rospy.Publisher("wam/joint_states", sensor_msgs.msg.JointState)
        self._wam_pub_rate = rospy.Rate(MockWam._WAM_RATE)
        self._joint_move_srv = rospy.Service("wam/joint_move", wam_srvs.srv.JointMove, self.mock_joint_move)
        self._hold_joint_srv = rospy.Service("wam/hold_joint_pos", wam_srvs.srv.Hold, self.mock_hold_joint_pos)

    def pub_wam(self):
        while not rospy.is_shutdown():
            joint_state_msg = sensor_msgs.msg.JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ["" for i in range(MockWam._WAM_DOF)]
            joint_state_msg.position = self._wam_joint_states
            joint_state_msg.velocity = [0 for i in range(MockWam._WAM_DOF)]
            joint_state_msg.effort = [0 for i in range(MockWam._WAM_DOF)]
            self._wam_joint_pub.publish(joint_state_msg)
            self._wam_pub_rate.sleep()

    def mock_joint_move(self, req):
        self._wam_joint_states = req.joints
        return wam_srvs.srv.JointMoveResponse()

    def mock_hold_joint_pos(self, req):
        return wam_srvs.srv.Hold()

if __name__ == "__main__":
    try:
        rospy.init_node("mock_wam")
        mock_wam = MockWam()
        mock_wam.pub_wam()
    except rospy.ROSInterruptException:
        pass

