#! /usr/bin/env python

import roslib; roslib.load_manifest("wam_controllers")
import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import sensor_msgs.msg
import wam_srvs.srv

# Interface between ros arm navigation stack's action client and Barrett WAM
class WamFollowJointTrajectoryController(object):
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self):
        self._action_name = "arm_controller/follow_joint_trajectory"
        #TODO read joint names from urdf or param server
        self._joint_names = ["wam_j1_joint", "wam_j2_joint", "wam_j3_joint",\
          "wam_j4_joint", "wam_j5_joint", "wam_j6_joint", "wam_j7_joint",\
          "bhand_j11_joint", "bhand_j12_joint", "bhand_j13_joint",\
          "bhand_j21_joint", "bhand_j22_joint", "bhand_j23_joint",\
          "bhand_j32_joint", "bhand_j33_joint"]
        num_joints = len(self._joint_names)
        # Actual joint positions
        self._joint_position_actual = [0.0 for i in range(num_joints)]
        self._joint_velocity_actual = [0.0 for i in range(num_joints)]

    # Start subscriber and action server
    def start(self):
        rospy.Subscriber("joint_states", sensor_msgs.msg.JointState,\
                         self.joint_state_callback)
        self._action_server = actionlib.SimpleActionServer(self._action_name,\
          control_msgs.msg.FollowJointTrajectoryAction,\
          execute_cb = self.execute_callback, auto_start = False)
        self._action_server.start()

    # Have arm follow trajectory specified in goal
    def execute_callback(self,  goal):
        rospy.loginfo("Follow joint trajectory goal received")
        self.follow_trajectory(goal.trajectory)

    def follow_trajectory(self, trajectory):
        rospy.loginfo("Waiting for service /wam/joint_traj_move")
        rospy.wait_for_service("/wam/joint_traj_move")
        try:
            rospy.loginfo("Following joint trajectory")
            rospy.logdebug("joint_names: " + str(trajectory.joint_names))
            traj_move = rospy.ServiceProxy("/wam/joint_traj_move",\
              wam_srvs.srv.JointTrajectoryMove)
            success = traj_move(trajectory)
            # Wait till trajectory complete
            time = trajectory.points[-1].time_from_start.to_sec() + 1.0
            rospy.sleep(time)
            if success:
                rospy.loginfo("Joint trajectory succeeded")
                self._result.error_code =\
                  control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
                rospy.logdebug("Succeeded: " + str(self._result.error_code))
                rospy.logdebug("Goal: " + str(trajectory.points[-1].positions))
                rospy.logdebug("Actual: " + str(self._joint_position_actual))
                # wait till joint state updated before setting success
                #TODO
                self._action_server.set_succeeded(self._result)
            else:
                rospy.loginfo("Joint trajectory failed")
                self._action_server.set_aborted()
        except rospy.ServiceException, e:
            rospy.logerr("Failed to call joint_traj_move service: " + str(e))
            self._action_server.set_aborted()

    # Continually update actual joint state
    def joint_state_callback(self, joint_state_msg):
        self._joint_position_actual = joint_state_msg.position
        self._joint_velocity_actual = joint_state_msg.velocity

if __name__ == "__main__":
    rospy.init_node("wam_follow_joint_trajectory_controller")
    wfjtc = WamFollowJointTrajectoryController()
    wfjtc.start()
    rospy.spin()

