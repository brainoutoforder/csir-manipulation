#! /usr/bin/env python

import roslib; roslib.load_manifest("csir_joint_trajectory_action_interface")
import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import pr2_controllers_msgs.msg
import sensor_msgs.msg
import wam_srvs.srv

# Interface between ros arm navigation stack's action client and Barrett WAM
class WamSimpleActionServer(object):
    _dof = 7 # WAM degrees of freedom

    def __init__(self, name):
        self._action_name = "arm/joint_trajectory"
        self._joint_command_rate = 1.0
        # Initialise joint_map containing joint_names, upper and lower limits and current values
        #TODO read joint names from urdf or param server
        self._joint_names = ["wam_j1_joint", "wam_j2_joint", "wam_j3_joint", "wam_j4_joint", "wam_j5_joint", "wam_j6_joint", "wam_j7_joint", "bhand_j11_joint", "bhand_j12_joint", "bhand_j13_joint", "bhand_j21_joint", "bhand_j22_joint", "bhand_j23_joint", "bhand_j32_joint", "bhand_j33_joint"]
        self._lower_limits = [-2.6, -2.0, -2.8, -0.9, -4.76, -1.6, -3.0, 0.0, 0.0, 0.698131701, -3.14159265, 0.0, 0.698131701, 0.0, 0.698131701]
        self._upper_limits = [ 2.6,  2.0,  2.8,  3.1,  1.24,  1.6,  3.0, 3.14159265, 2.44346095, 2.44346095, 0.0, 2.44346095, 2.44346095, 2.44346095, 2.44346095]
        self._num_free_joints = len(self._joint_names)
        self._joint_map = {}
        for i in range(self._num_free_joints):
            self._joint_map[self._joint_names[i]] = {"lower":self._lower_limits[i], "upper":self._upper_limits[i], "pos":0.0, "vel":0.0, "eff":0.0}
        # Actual joint positions
        self._joint_position_actual = [0.0 for i in range(self._num_free_joints)]
        self._joint_velocity_actual = [0.0 for i in range(self._num_free_joints)]
        # Home position
        self._joint_position_home = [0.0 for i in range(self._num_free_joints)]
        self._joint_position_home[1] = self._joint_map["wam_j2_joint"]["lower"]
        self._joint_position_home[3] = self._joint_map["wam_j4_joint"]["upper"]
        self._joint_position_home[5] = self._joint_map["wam_j6_joint"]["lower"]

    # Start subscriber and action server
    def start(self):
        rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_state_callback)
        self._action_server = actionlib.SimpleActionServer(self._action_name, pr2_controllers_msgs.msg.JointTrajectoryAction, execute_cb = self.execute_callback, auto_start = False)
        self._action_server.start()

    # Have arm follow trajectory specified in goal
    def execute_callback(self,  goal):
        rospy.loginfo("pr2_controllers_msgs:JointTrajectoryGoal received")
        #self._trajectory = self.extract_free_joint_info(goal.trajectory)
        #self.execute_trajectory(self._trajectory)
        #self.see_traj(goal.trajectory)
        self.move_traj(goal.trajectory)

    def see_traj(self, traj):
        rospy.loginfo("Time stamps")
        for point in traj.points:
            rospy.loginfo(point.time_from_start)

    def move_traj(self, traj):
        rospy.loginfo("Waiting for service /wam/joint_traj_move")
        rospy.wait_for_service("/wam/joint_traj_move")
        try:
            rospy.loginfo("Executing joint trajectory")
            rospy.loginfo("joint_names: " + str(traj.joint_names))
            traj_move = rospy.ServiceProxy("/wam/joint_traj_move", wam_srvs.srv.JointTrajectoryMove)
            success = traj_move(traj)
            if success:
                rospy.loginfo("Joint trajectory succeeded")
                rospy.loginfo("Succeeded: ")
                rospy.loginfo("Goal: " + str(traj.points[-1].positions))
                rospy.loginfo("Actual: " + str(self._joint_position_actual))
                self._action_server.set_succeeded()
            else:
                rospy.loginfo("Joint trajectory failed")
                self._action_server.set_aborted()
        except rospy.ServiceException, e:
            rospy.logerr("Failed to call joint_traj_move service: " + str(e))
            self._action_server.set_aborted()

    # Extract joint positions, velocities and timing information from goal message
    def extract_free_joint_info(self, trajectory):
        num_points = len(trajectory.points)
        num_joints = self._dof
        extracted_trajectory = [[0 for j in range(num_joints)] for i in range(num_points)]
        extracted_times = [0 for i in range(num_points)]
        for i in range(num_points):
            assert len(trajectory.points[i].positions) == num_joints, "Incorrect number of joints in goal->positions: " + str(len(trajectory.points[i].positions))
            #for j in range(num_joints):
                #extracted_trajectory[i][j] = trajectory.points[i].positions[j]
                #extracted_velocities[i][j] = trajectory.points[i].velocities[j]
            for name in self._joint_names[:self._dof]:
                j_extr = self._joint_names.index(name)
                j_goal = trajectory.joint_names.index(name)
                extracted_trajectory[i][j_extr] = trajectory.points[i].positions[j_goal]
        return extracted_trajectory

    # Send move joint commands to Barrett WAM according to trajectory
    def execute_trajectory(self, trajectory):
        tol = 1e-1
        success = True
        command_rate = rospy.Rate(10000)
        num_points = len(trajectory)
        for i in range(num_points):
            #command_rate = rospy.Rate(times[i])
            if self._action_server.is_preempt_requested():
                rospy.loginfo("Preempted. Holding position.")
                self.hold_joint_position()
                self._action_server.set_preempted()
                success = False
                break
            pos_reached = False
            rospy.loginfo("Moving:\n\t[from]: " + str(self._joint_position_actual) + "\n\t  [to]: " + str(trajectory[i]))
            self.move_to_joint_position(trajectory[i])
            while not pos_reached: #TODO and not timed_out
                #rospy.loginfo("Moving:\n\t" + str(trajectory[i]) + "\n\t" + str(self._joint_position_actual))
                if self._action_server.is_preempt_requested():
                    rospy.loginfo("Preempted. Holding position.")
                    self.hold_joint_position()
                    self._action_server.set_preempted()
                    success = False
                    break
                command_rate.sleep()
                for j in range(self._dof):
                    pos_reached = True
                    if abs(trajectory[i][j] - self._joint_position_actual[j]) > tol:
                        pos_reached = False
                        break
                #TODO check if timed out
        if success and pos_reached: #TODO and not timed_out
            rospy.loginfo("wam_joint_controller succeeded")
            self._action_server.set_succeeded()
        else:
            rospy.loginfo("wam_joint_controller failed to execute trajectory")

    def move_to_joint_position(self, joint_position):
        #assert len(joint_position) == self._dof, "Incorrect number of joints"
        rospy.loginfo("Waiting for service /wam/joint_move")
        rospy.wait_for_service("/wam/joint_move")
        try:
            joint_move = rospy.ServiceProxy("/wam/joint_move", wam_srvs.srv.JointMove)
            # Get required position from joint_position list
            joint_move(joint_position[:self._dof])
            joint_string = ""
            for i in range(self._dof):
                joint_string += " j" + str(i+1) + ":" + str(joint_position[i])
            rospy.loginfo("WAM moving to" + joint_string)
        except rospy.ServiceException, e:
            rospy.logerr("Failed to call joint_move service: " + str(e))

    # Continually update actual joint state
    def joint_state_callback(self, joint_state_msg):
        #assert len(joint_state_msg.position) == self._dof, "Incorrect number of joints: " + str(len(joint_state_msg.position))
        #assert len(joint_state_msg.velocity) == self._dof, "Incorrect number of joints"
        #rospy.loginfo("Joint state received:\n\t" + str(joint_state_msg.name) + "\n\t" + str(joint_state_msg.position))
        self._joint_position_actual = joint_state_msg.position
        self._joint_velocity_actual = joint_state_msg.velocity
        for i in range(len(joint_state_msg.position)):
            self._joint_map[self._joint_names[i]]["pos"] = joint_state_msg.position[i]
            self._joint_map[self._joint_names[i]]["vel"] = joint_state_msg.velocity[i]

    # Have WAM stop moving
    def hold_joint_position(self):
        rospy.loginfo("Waiting for service /wam/hold_joint_pos")
        rospy.wait_for_service("/wam/hold_joint_pos")
        try:
            hold_joint_pos = rospy.ServiceProxy("/wam/hold_joint_pos", wam_srvs.srv.Hold)
            hold_joint_pos(True)
            rospy.loginfo("WAM position held.")
        except rosp.ServiceException, e:
            rospy.logerr("Failed to call hold_joint_pos service.")

    # Move WAM to home position set in __init__ of this class
    def move_home(self):
        self.move_to_joint_position(self._joint_position_home)

if __name__ == "__main__":
    try:
        rospy.init_node("wam_joint_controller")
        wsas = WamSimpleActionServer(rospy.get_name())
        wsas.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

