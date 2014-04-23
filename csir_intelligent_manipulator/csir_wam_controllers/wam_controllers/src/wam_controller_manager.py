#! /usr/bin/env python

# Copyright (c) 2014 csir_intelligent_manipulator authors. All rights reserved.
# Use of this source code is governed by the MIT license that can be found in
# the LICENSE file.
# Author: Jimmy Kizito

import roslib
roslib.load_manifest("wam_controllers")
import rospy
import pr2_mechanism_msgs.srv

# def class WamControllerManager(object):
class WamControllerManager(object):
    """ Does nothing useful except satisfy ROS API requirements for object
    manipulation stack.
    """
    def __init__(self):
        self._has_joint_started = False
        self._has_cart_started = False
        self._switch_srv = rospy.Service("wam_switch_controller",
            pr2_mechanism_msgs.srv.SwitchController,
            self.switch_controller)
        self._list_srv = rospy.Service("wam_list_controllers",
            pr2_mechanism_msgs.srv.ListControllers,
            self.list_controllers)

    def switch_controller(self, req):
        rep = pr2_mechanism_msgs.srv.SwitchControllerResponse()
        rospy.loginfo("start: " + ", ".join(req.start_controllers))
        rospy.loginfo("stop: " + ", ".join(req.stop_controllers))
        if "wam_joint_controller" in req.start_controllers:
            rospy.loginfo("joint_controller started")
            self._has_joint_started = True
            self._has_cart_started = False
        else:
            rospy.loginfo("cart_controller started")
            self._has_cart_started = True
            self._has_joint_started = False
        rep.ok = True
        return rep

    def list_controllers(self, req):
        rep = pr2_mechanism_msgs.srv.ListControllersResponse()
        rep.controllers.extend(["wam_joint_controller", "wam_cart_controller"])
        if self._has_joint_started:
            rospy.loginfo("joint_controller running")
            rep.state.extend(["running", "stopped"])
        else:
            rospy.loginfo("cart_controller running")
            rep.state.extend(["stopped", "running"])
        return rep

if __name__ == "__main__":
    rospy.init_node("wam_controller_manager")
    wcm = WamControllerManager()
    rospy.spin()

