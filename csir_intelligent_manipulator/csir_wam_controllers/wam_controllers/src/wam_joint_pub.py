#! /usr/bin/env python

import roslib; roslib.load_manifest("wam_controllers")
import rospy
import sensor_msgs.msg

class WamJointPub:
    def __init__(self):
        self._joint_state_pub = rospy.Publisher("joint_states", sensor_msgs.msg.JointState)
        self._pub_rate = rospy.Rate(250)
        self._wam_joint_sub = rospy.Subscriber("wam/joint_states", sensor_msgs.msg.JointState, self.wam_joint_state_cb)
        self._bhand_joint_sub = rospy.Subscriber("bhand/joint_states", sensor_msgs.msg.JointState, self.bhand_joint_state_cb)
        self._wam_joints_pos = [0 for i in range(7)]#7 DOF of Arm
        self._wam_joints_vel = [0 for i in range(7)]
        self._wam_joints_eff = [0 for i in range(7)]
        self._bhand_joints_pos = [0 for i in range(8)]#8 DOF of Hand
        self._bhand_joints_vel = [0 for i in range(8)]
        self._bhand_joints_eff = [0 for i in range(8)]
        self._joint_names = ["wam_j1_joint", "wam_j2_joint", "wam_j3_joint", "wam_j4_joint", "wam_j5_joint", "wam_j6_joint", "wam_j7_joint", "bhand_j11_joint", "bhand_j12_joint", "bhand_j13_joint", "bhand_j21_joint", "bhand_j22_joint", "bhand_j23_joint", "bhand_j32_joint", "bhand_j33_joint"]

    def wam_joint_state_cb(self, wam_joint_msg):
        assert len(wam_joint_msg.name) == 7
        self._wam_joints_pos = list(wam_joint_msg.position)
        self._wam_joints_vel = list(wam_joint_msg.velocity)
        self._wam_joints_eff = list(wam_joint_msg.effort)

    def bhand_joint_state_cb(self, bhand_joint_msg):
        assert len(bhand_joint_msg.name) == 7
        self._bhand_joints_pos = list(bhand_joint_msg.position)
        #self._bhand_joints_vel = [0 for i in range(8)]#bhand_joint_msg.velocity
        #self._bhand_joints_eff = [0 for i in range(8)]#bhand_joint_msg.effort
        # wam_node publishes joint state in different format, some modifications necessary for arm_nav stack
        spread = self._bhand_joints_pos[3]
        self._bhand_joints_pos.append(self._bhand_joints_pos[3])
        self._bhand_joints_pos[3] = 3.14159265 - spread
        self._bhand_joints_pos[7] = spread - 3.14159265
        new_order = [3, 0, 4, 7, 1, 5, 2, 6]
        self._bhand_joints_pos = [self._bhand_joints_pos[i] for i in new_order]

    def pub_joints(self):
        while not rospy.is_shutdown():
            joint_state_msg = sensor_msgs.msg.JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = self._joint_names
            joint_state_msg.position.extend(self._wam_joints_pos)
            joint_state_msg.position.extend(self._bhand_joints_pos)
            # Rounding to account for encoder inaccurracy
            joint_state_msg.position = [round(pos, 2) for pos in joint_state_msg.position]
            joint_state_msg.velocity.extend(self._wam_joints_vel)
            joint_state_msg.velocity.extend(self._bhand_joints_vel)
            joint_state_msg.velocity = [round(vel, 2) for vel in joint_state_msg.velocity]
            joint_state_msg.effort.extend(self._wam_joints_eff)
            joint_state_msg.effort.extend(self._bhand_joints_eff)
            joint_state_msg.effort = [round(eff, 2) for eff in joint_state_msg.effort]
            self._joint_state_pub.publish(joint_state_msg)
            self._pub_rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("wam_joint_pub")
        wjp = WamJointPub()
        wjp.pub_joints()
    except rospy.ROSInterruptException:
        pass

