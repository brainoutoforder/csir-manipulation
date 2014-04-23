#! /usr/bin/env python

# Copyright (c) 2014 csir_intelligent_manipulator authors. All rights reserved.
# Use of this source code is governed by the MIT license that can be found in
# the LICENSE file.
# Author: Jimmy Kizito

""" bhand_grasp_controller node interfaces with ROS fuerte manipulation stack.

Classes:
    BHandGraspController: controls Barrett Hand.
"""

import roslib
roslib.load_manifest("bhand_grasp_controller")
import rospy
import actionlib
import wam_srvs.srv
import sensor_msgs.msg
import object_manipulation_msgs.msg
import object_manipulation_msgs.srv
import geometry_msgs.msg
from math import pi, log

class BHandGraspController(object):
    """ Controls Barrett Hand.

    Attributes:
        _DOF: Total number of joints in Barrett Hand.
        _GRASP_DOF: Number of joints controlled for grasping.
        _JOINT_TOL: Rounding for joint readings in radians
        _MAX_RANGE: Max range of grasping joints (i.e. proximal finger joints)
            in radians.
        _PROXIMAL_JOINTS: Indeces of proximal joints in JointState message.
        _BHAND_PUB_FREQ: Pubhlishing frequency in Hz for JointState messages.
        _BHAND_NAMES: Names of joints in JointState message.
        _BHAND_OBJECT_PRESENCE_THRESHOLD: grasp metric threshold
    """
    # "Constants"
    _DOF = 7
    _GRASP_DOF = 3
    _JOINT_TOL = 0.1
    _MAX_RANGE = 7.0 / 9.0 * pi
    _PROXIMAL_JOINTS = [0, 1, 2]
    _BHAND_PUB_FREQ = 10.0
    _BHAND_NAMES = ["inner_f1", "inner_f2", "inner_f3", "spread","outer_f1",
                    "outer_f2", "outer_f3"]
    _BHAND_OBJECT_PRESENCE_THRESHOLD = 0.1

    def __init__(self):
        rospy.loginfo("Init class")
        self._bhand_joints = []
        self._wam_pose = []
        # Calculate no. of decimal places to round to from JOINT_TOL
        self._decimal_places = int(-log(BHandGraspController._JOINT_TOL /\
                                        BHandGraspController._MAX_RANGE , 10))
        self._grasp_status_srv = rospy.Service("grasp_query_name",
            object_manipulation_msgs.srv.GraspStatus,
            self.srv_grasp_status)
        rospy.loginfo("Barrett Hand posture controller started")

    def start(self):
        """ Start JointState subscriber and action server.

        Separate from __init__ to facilitate testing of class methods.
        """
        rospy.logdebug("Starting class")
        self._object_threshold = rospy.get_param(
            "bhand_object_presence_threshold",
            BHandGraspController._BHAND_OBJECT_PRESENCE_THRESHOLD)
        rospy.Subscriber("bhand/joint_states", sensor_msgs.msg.JointState,
                         self.joint_state_callback)
        rospy.Subscriber("wam/pose", geometry_msgs.msg.PoseStamped,
                         self.wam_pose_callback)
        self._action_server = actionlib.SimpleActionServer(
            "posture_action_name",
            object_manipulation_msgs.msg.GraspHandPostureExecutionAction,
            execute_cb = self.execute_callback, auto_start = False)
        self._action_server.start()
        self.open_bhand()

    def joint_state_callback(self, joint_state_msg):
        """ Save joint position
        """
        assert(len(joint_state_msg.position) == BHandGraspController._DOF)
        self._bhand_joints = joint_state_msg.position
        rospy.logdebug("Joint state received" + ": " + str(self._bhand_joints))

    def wam_pose_callback(self, wam_pose_msg):
        """ Save WAM cartesian position
        """
        self._wam_pose = wam_pose_msg.pose
        rospy.logdebug("WAM pose received" + ": " + str(self._wam_pose))

    def srv_grasp_status(self, req):
        """ Returns true if hand occupied, false otherwise based on joint state.
        """
        # Wait for hand to move and update
        rospy.sleep(1.0)
        # Give hand time to close and updated joint state to be published
        #rospy.sleep(2.5/BHandGraspController._BHAND_PUB_FREQ)
        grasp_metric = self.convert_joints_to_grasp(self._bhand_joints)
        rospy.loginfo("Serving grasp status for "+ str(self._bhand_joints) + ": " + str(grasp_metric))
        rep = object_manipulation_msgs.srv.GraspStatusResponse()
        if grasp_metric > self._object_threshold:
            rep.is_hand_occupied = True
        else:
            rep.is_hand_occupied = False
        return rep

    def execute_callback(self, goal):
        """ Open or close hand depending on goal
        """
        rospy.loginfo("Executing hand posture goal")
        if goal.goal == object_manipulation_msgs.msg.GraspHandPostureExecutionGoal.PRE_GRASP:
            self.open_bhand()
        elif goal.goal == object_manipulation_msgs.msg.GraspHandPostureExecutionGoal.GRASP:
            self.grasp_bhand()
        elif goal.goal == object_manipulation_msgs.msg.GraspHandPostureExecutionGoal.RELEASE:
            self.release_bhand()
        else:
            rospy.logerr("Grasp goal not recognised")
            self._action_server.set_aborted()
            return
        # Assume successful completion once this point reached
        self._action_server.set_succeeded()

    def convert_joints_to_grasp(self, bhand_joints):
        """ Converts joint values into a single grasp measurement.

        This implementation only examines the 3 proximal joints of the hand.
        Arguments:
            bhand_joints: list of Barrett Hand joint values in radians. Varies
                from 0 (open) to 140 degrees (closed)
        Returns:
            grasp_metric: Varies from 0.0 (open) to 1.0 (closed)
        """
        assert(len(bhand_joints) == BHandGraspController._DOF)
        joint_sum = 0
        for i in BHandGraspController._PROXIMAL_JOINTS:
            joint_sum += bhand_joints[i]
        grasp_metric = joint_sum / BHandGraspController._MAX_RANGE / 3.0
        grasp_metric = round(grasp_metric, self._decimal_places)
        return grasp_metric

    def open_bhand(self):
        """ Spread and open Barrett Hand
        """
        rospy.loginfo("Waiting to open fingers")
        rospy.wait_for_service("/bhand/spread_pos")
        try:
            spread_hand = rospy.ServiceProxy("/bhand/spread_pos",
                                             wam_srvs.srv.BHandSpreadPos)
            spread_hand(0.5) # ~60 degrees between outer fingers
            rospy.loginfo("Spreading fingers")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call spread service " + str(e))
        rospy.wait_for_service("/bhand/grasp_pos")
        try:
            grasp_hand = rospy.ServiceProxy("/bhand/grasp_pos",
                                            wam_srvs.srv.BHandGraspPos)
            grasp_hand(0.0)
            rospy.loginfo("Opening fingers")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call grasp service " + str(e))

    def release_bhand(self):
        """ Release with Barrett Hand
        """
        self.open_bhand()
        rospy.loginfo("Releasing")
        rospy.wait_for_service("/wam/cart_move")
        try:
            cart_move = rospy.ServiceProxy("/wam/cart_move",
                                             wam_srvs.srv.CartPosMove)
            move_down = [self._wam_pose.position.x, self._wam_pose.position.y,\
                       self._wam_pose.position.z - 0.2]
            move_up = [self._wam_pose.position.x, self._wam_pose.position.y,\
                       self._wam_pose.position.z + 0.2]
            cart_move(move_down)
            self.open_bhand()
            cart_move(move_up)
            rospy.loginfo("Lifting WAM")
            rospy.sleep(2.0)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call cart service " + str(e))

    def close_bhand(self):
        """ Close Barrett fingers
        """
        rospy.loginfo("Waiting to close fingers")
        rospy.wait_for_service("/bhand/grasp_pos")
        try:
            grasp_hand = rospy.ServiceProxy("/bhand/grasp_pos",
                                            wam_srvs.srv.BHandGraspPos)
            grasp_hand(BHandGraspController._MAX_RANGE)
            rospy.loginfo("Closing fingers")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call grasp service " + str(e))

    def grasp_bhand(self):
        """ Grasp with Barrett fingers
        """
        rospy.loginfo("Lowering to grasp")
        rospy.wait_for_service("/wam/cart_move")
        try:
            cart_move = rospy.ServiceProxy("/wam/cart_move",
                                             wam_srvs.srv.CartPosMove)
            move_down = [self._wam_pose.position.x, self._wam_pose.position.y,\
                       self._wam_pose.position.z - 0.2]
            cart_move(move_down)
            rospy.loginfo("Lowering WAM")
            rospy.sleep(2.0)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call cart service " + str(e))
        self.close_bhand()

if __name__ == "__main__":
    rospy.init_node("bhand_grasp_controller")
    bgc = BHandGraspController()
    bgc.start()
    rospy.spin()

