#! /usr/bin/env python

import roslib
roslib.load_manifest("wam_grasp_segmented_cluster_planner")
import rospy
import actionlib
import tf
import sensor_msgs.msg
import object_manipulation_msgs.msg
import object_manipulation_msgs.srv
import visualization_msgs.msg

class WamGraspSegmentedClusterPlanner(object):

    def __init__(self):
        self.tf_listener = tf.TransformListener()
#self._marker_sub = rospy.Subscriber("/planning_scene_markers_array", visualization_msgs.msg.MarkerArray, self.marker_cb)

    def marker_cb(self, msg):
        for marker in msg.markers:
            if "graspable_object" in marker.ns: 
                self._frame = marker.header.frame_id
                self._type = marker.type
                self._pose = marker.pose
                self._scale = marker.scale

    def start(self):
        rospy.loginfo("Starting wam_cluster_planner")
        self._as = actionlib.SimpleActionServer("wam_cluster_planner", object_manipulation_msgs.msg.GraspPlanningAction, execute_cb = self.execute_callback, auto_start = False)
        self._as.start()

    def execute_callback(self, goal):
        rospy.loginfo("cluster planner ex_cb")
        box_srv = rospy.ServiceProxy("/find_cluster_bounding_box", object_manipulation_msgs.srv.FindClusterBoundingBox) 
        req = object_manipulation_msgs.srv.FindClusterBoundingBoxRequest()
        req.cluster = goal.target.cluster
        rep = box_srv(req)
        rospy.loginfo("arm: " + goal.arm_name)
        rospy.loginfo("collision_object_name: " + goal.collision_object_name)
#rospy.loginfo("frame: " + str(self._frame))
#rospy.loginfo("type (CUBE=1): " + str(self._type))
        rospy.loginfo("pose: " + str(rep.pose))
        rospy.loginfo("pose: " + str(rep.box_dims))
        rospy.loginfo("error: " + str(rep.error_code))
        result = object_manipulation_msgs.msg.GraspPlanningResult()
        (grasps, error_code) = self.plan_grasps(rep.pose.pose, rep.box_dims)
        result.grasps = grasps
        result.error_code = error_code
        self._as.set_succeeded(result)

    def plan_grasps(self, pose, box_dims):
#rospy.loginfo("target frame: " + target.reference_frame_id)
#rospy.loginfo("collision_name: " + target.collision_name)
        grasps = []#object_manipulation_msgs.msg.Grasp()
        grasp = object_manipulation_msgs.msg.Grasp()
        pose.position.y = -0.01
        pose.position.z = pose.position.z + pose.position.z/2.0 + 10.0
        pose.orientation.x = -0.36
        pose.orientation.y = 0.6
        pose.orientation.z = -0.458 
        pose.orientation.w = 0.548
        grasp.grasp_pose = pose
        grasp.desired_approach_distance = 0.015
        grasp.min_approach_distance = 0.015
#grasp.pre_grasp_posture = 
        rospy.loginfo("grasp: " + str(grasp))
        grasps.append(grasp)
        error_code = object_manipulation_msgs.msg.GraspPlanningErrorCode()
        error_code.value = object_manipulation_msgs.msg.GraspPlanningErrorCode.SUCCESS 
        #.OTHER_ERROR, .SUCCESS, .TF_ERROR
        return (grasps, error_code)

if __name__ == "__main__":
    rospy.init_node("wam_cluster_planner_node")
    wgscp = WamGraspSegmentedClusterPlanner()
    wgscp.start()
    rospy.spin()

