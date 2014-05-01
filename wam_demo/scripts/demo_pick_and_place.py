#! /usr/bin/env python

# Reads object_position topic and uses compute_ik and joint_move services to perform a single pick and place task.

import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import wam_srvs.srv
import moveit_msgs.srv
import sys

def object_position_callback(object_position):
    # x, y, z <- object_position.x, y, z

    # Use the position to send a request to the "/compute_ik" service in moveit for pre-grasp joint configuration of the WAM.
    # Requre service clients for compute_ik, joint_move, bhand_close, bhand_open (go_home)
    # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
    # wam_pose <- x, y, z, qx, qy, qz, qw
    # pre_grasp_pose <- compute_ik(wam_pose)

    # use wam_srvs.joint_move to move to computed pose

    # use wam_srvs...close... to grasp object

    # use compute_ik, joint_move to raise the object.

    # use compute_ik to translate object - this can be to a fixed location on the table which you are certain has an IK solution

    # use compute_ik, joint_move to lower the object.

    # use wam_srvs...open... to release object

    # use wam_srvs..go_home (or compute_ik and joint_move) to move the WAM out of the sensor's field of view. 
  
    # Shut down node after completing task
    sys.exit() # or raise rospy.ROSInterruptException 

if __name__ == "__main__:
    try:
        rospy.init_node("demo_pick_and_place")
        rospy.Subscriber("object_position", geometry_msgs.Point, object_position_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
