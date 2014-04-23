#! /usr/bin/env python

# Copyright (c) 2014 CSIR. All rights reserved.
# Use of this source code is governed by the MIT license that can be found in
# the LICENSE file.

""" Node tests for bhand_grasp_controller

    Classes:
        BHandGraspControllerNTest: test fixture
"""

PKG = "bhand_grasp_controller"
import roslib
roslib.load_manifest(PKG)
import rospy
import unittest
import sensor_msgs.msg
import object_manipulation_msgs.msg
import wam_srvs.srv
import actionlib
import actionlib_msgs
from threading import Thread
from bhand_grasp_controller import *

class BHandGraspControllerNTest(unittest.TestCase):
    """ Node test fixture for testing bhand_grasp_controller.

    Attributes:
        _bhand_pub: Publishes synthetic joint states of the Barrett Hand.
        _bhand_thread: Thread that publishes hand joint states.
        _bhand_names: Names of joints in Barrett Hand used in joint state
            messages.
        _bhand_pos: List of hand joint values in radians.
        _bhand_pub_freq: The rate in Hz at which the joint state is published.
    """

    @classmethod
    def setUpClass(cls):
        """ Initialise class members once per test suite.
        """
        cls._bhand_dof = BHandGraspController._DOF
        cls._bhand_names = BHandGraspController._BHAND_NAMES
        cls._bhand_pub_freq = BHandGraspController._BHAND_PUB_FREQ
        cls._bhand_pos = [0.0 for i in range(cls._bhand_dof)]
        cls._proximal_finger_indeces =\
            BHandGraspController._PROXIMAL_JOINTS
        rospy.init_node("ntest_bhand_grasp_controller") # to __main__
        cls._bhand_pub = rospy.Publisher("bhand/joint_states",
                                          sensor_msgs.msg.JointState)
        cls._grasp_action_client = actionlib.SimpleActionClient(
            "posture_action_name",
            object_manipulation_msgs.msg.GraspHandPostureExecutionAction)
        cls._grasp_action_client.wait_for_server() 
        cls._bhand_spread_srv = rospy.Service("bhand/spread_pos",
                                               wam_srvs.srv.BHandSpreadPos,
                                               cls.sim_bhand_spread)
        cls._bhand_grasp_srv = rospy.Service("bhand/grasp_pos",
                                              wam_srvs.srv.BHandGraspPos,
                                              cls.sim_bhand_grasp)
        cls._bhand_thread = Thread(target = cls.pub_bhand)
        cls._bhand_thread.start()

    def setUp(self):
        """ Initialise class members for test.
        """
        self._bhand_dof = BHandGraspController._DOF
        self._bhand_pub_freq = BHandGraspController._BHAND_PUB_FREQ
        self._grasp_goal =\
            object_manipulation_msgs.msg.GraspHandPostureExecutionGoal()
        #self._bhand_pos = [0.0 for i in range(self._bhand_dof)]
        #self._proximal_finger_indeces =\
            #BHandGraspController._PROXIMAL_JOINTS

    def tearDown(self):
        """ Release resources in preparation for next test.
        """
        #self._bhand_spread_srv.shutdown("Test complete")
        #self._bhand_grasp_srv.shutdown("Test complete")
        pass

    @classmethod
    def pub_bhand(cls):
        """ Publish the joint state in _joint_pos on topic bhand/joint_states.
        """
        rospy.logdebug("Publishing BHand")
        pub_rate = rospy.Rate(cls._bhand_pub_freq)
        while not rospy.is_shutdown():
            bhand_msg = sensor_msgs.msg.JointState()
            bhand_msg.header.stamp = rospy.Time.now()
            bhand_msg.name = cls._bhand_names
            bhand_msg.position = cls._bhand_pos
            bhand_msg.velocity = [0.0 for i in range(cls._bhand_dof)]
            bhand_msg.effort = [0.0 for i in range(cls._bhand_dof)]
            cls._bhand_pub.publish(bhand_msg)
            pub_rate.sleep()

    def query_grasp_service(self):
        """ Query the GraspStatus service.

        Returns:
            True if hand occupied, False otherwise.
        """
        rospy.logdebug("Querying grasp status")
        rospy.wait_for_service('grasp_query_name')
        try:
            grasp_query_client = rospy.ServiceProxy("grasp_query_name",
                                                    object_manipulation_msgs.srv.GraspStatus)
            res = grasp_query_client(object_manipulation_msgs.msg.Grasp())
            rospy.logdebug("Received grasp status")
            return res.is_hand_occupied
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def send_goal(self, goal):
        """ Send grasp goal to grasp actionserver.

        Arguments:
            goal (int): PRE_GRASP=1, GRASP=2, RELEASE=3
        """
        rospy.logdebug("Goal")
        self._grasp_goal.goal = goal
        self._grasp_action_client.send_goal(self._grasp_goal)
        self._grasp_action_client.wait_for_result()

    @classmethod
    def sim_bhand_spread(cls, bhand_spread_req):
        """ Simulate the BHandSpreadPos service in the Barrett wam_node.

        Changes the finger spread angle in the hand joint state.
        Arguments:
            bhand_spread_req (BHandSpreadPosRequest): The requested spread angle
                in radians
        """
        rospy.logdebug("Spreading")
        cls._bhand_pos[3] = bhand_spread_req.radians # 3 is the index of the spread angle
        return wam_srvs.srv.BHandSpreadPosResponse()

    @classmethod
    def sim_bhand_grasp(cls, bhand_grasp_req):
        """ Simulate the BHandGraspPos service in the Barrett wam_node.

        The three proximal joint angles of the Barrett hand are set to the
        requested angle.
        Arguments:
            bhand_grasp_req (BHandGraspPosRequest): The requested proximal joint
                angles in radians.
        """
        rospy.logdebug("Grasping")
        for i in cls._proximal_finger_indeces:
            cls._bhand_pos[i] = bhand_grasp_req.radians
        return wam_srvs.srv.BHandGraspPosResponse()

    def test_grasp_query(self):
        """ Test GraspStatus service.

        # Verify that initially open
        # Simulate close - change self._bhand_pos directly
        # Verify occupied
        # Simulate partially open
        # Verify occupied
        # Simulate open
        # Verify unoccupied
        """
        rospy.sleep(10.0 / self._bhand_pub_freq) # Wait for hand initialisation
        is_occupied = self.query_grasp_service()
        self.assertFalse(is_occupied) # Verify unoccupied
        for i in self.__class__._proximal_finger_indeces: # Sim close
            self.__class__._bhand_pos[i] = BHandGraspController._MAX_RANGE
        rospy.sleep(1.5 / self._bhand_pub_freq) # Wait for new joints to be published
        is_occupied = self.query_grasp_service()
        self.assertTrue(is_occupied) # Verify occupied
        for i in self.__class__._proximal_finger_indeces: # Sim partially open
            self.__class__._bhand_pos[i] = BHandGraspController._MAX_RANGE * 0.5
        rospy.sleep(1.5 / self._bhand_pub_freq) # Wait for new joints to be published
        is_occupied = self.query_grasp_service()
        self.assertTrue(is_occupied) # Verify occupied
        for i in self.__class__._proximal_finger_indeces: # Sim open
            self.__class__._bhand_pos[i] = 0.0
        rospy.sleep(1.5 / self._bhand_pub_freq) # Wait for new joints to be published
        is_occupied = self.query_grasp_service()
        self.assertFalse(is_occupied) # Verify unoccupied

    def test_grasp_execution(self):
        """ Test GraspHandPostureExecution action.

        Do twice
        # Send pre-grasp goal
        # Verify open
        # Send grasp goal
        # Verify closed
        # Send release goal
        # Verify open
        """
        rospy.sleep(10.0 / self._bhand_pub_freq) # Wait for hand initialisation
        for i in range(2):
            self.send_goal(
                object_manipulation_msgs.msg.GraspHandPostureExecutionGoal.PRE_GRASP)
            rospy.sleep(2.5 / self._bhand_pub_freq) # Wait for new joints to be published
            self.assertFalse(self.query_grasp_service()) # Verify open
            self.send_goal(
                object_manipulation_msgs.msg.GraspHandPostureExecutionGoal.GRASP)
            rospy.sleep(2.5 / self._bhand_pub_freq) # Wait for new joints to be published
            self.assertTrue(self.query_grasp_service()) # Verify occupied
            self.send_goal(
                object_manipulation_msgs.msg.GraspHandPostureExecutionGoal.RELEASE)
            rospy.sleep(2.5 / self._bhand_pub_freq) # Wait for new joints to be published
            self.assertFalse(self.query_grasp_service()) # Verify open

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "ntest_bhand_grasp_controller",
                   BHandGraspControllerNTest)

