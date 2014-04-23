#! /usr/bin/env python

# Copyright (c) 2014 CSIR. All rights reserved.
# Use of this source code is governed by the MIT license that can be found in
# the LICENSE file.
# Author: Jimmy Kizito

""" Unit tests for bhand_grasp_controller

Classes:
    BHandGraspControllerUTest: unit test fixture
"""

PKG = "bhand_grasp_controller"
import roslib
roslib.load_manifest(PKG)
import unittest
from bhand_grasp_controller import *

class BHandGraspControllerUTest(unittest.TestCase):
    """ Unit test fixture for testing bhand_grasp_controller.

    Attributes:
        _bgc: BHandGraspController class under test
        _bhand_joints: list of state (in radians) of 3 proximal finger joints.
            Varies from 0 to 140 degrees.
    """

    def setUp(self):
        """ Initialise class members for test.
        """
        self._bgc = BHandGraspController()
        self._bhand_joints = [0.0] * BHandGraspController._DOF

    def test_convert_joints_to_grasp(self):
        """ Test convert_joints_to_grasp function.

        Test that input values yield expected values and that and that "small"
        changes in the input are ignored.
        """
        # Expect disturbance invariant output
        expected_outputs = [0.0, 0.5, 1.0]
        for i in expected_outputs:
            theta = BHandGraspController._MAX_RANGE * i;
            # [-tol, 0, tol]
            for tol in [j * BHandGraspController._JOINT_TOL for j in range(-1, 2)]:
                self._bhand_joints =  [theta + tol] * len(self._bhand_joints)
                grasp_metric =\
                    self._bgc.convert_joints_to_grasp(self._bhand_joints)
                self.assertAlmostEqual(grasp_metric, i, self._bgc._decimal_places)
        # Check deviations around mid-point
        theta = 0.5 * BHandGraspController._MAX_RANGE
        self._bhand_joints =  [theta] * len(self._bhand_joints);
        self._bhand_joints[1] = 0.25 * BHandGraspController._MAX_RANGE
        grasp_metric = self._bgc.convert_joints_to_grasp(self._bhand_joints)
        self.assertLess(grasp_metric, 0.5)
        self.assertGreater(grasp_metric, 0.0)
        self._bhand_joints[1] = 0.75 * BHandGraspController._MAX_RANGE
        grasp_metric = self._bgc.convert_joints_to_grasp(self._bhand_joints)
        self.assertLess(grasp_metric, 1.0)
        self.assertGreater(grasp_metric, 0.5)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, "utest_bhand_grasp_controller",
                    BHandGraspControllerUTest)

