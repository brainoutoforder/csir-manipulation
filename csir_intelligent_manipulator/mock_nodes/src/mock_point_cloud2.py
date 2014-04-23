#! /usr/bin/env python

# Copyright (c) 2014 csir_intelligent_manipulator authors. All rights reserved.
# Use of this source is governed by the MIT license that can be found in
# the LICENSE file.
# Author: Jimmy Kizito

""" mock_point_cloud2 publishes mock data to collider_node

Classes:
    MockPointCloud2: publishes PointCloud2 data
"""

import roslib
roslib.load_manifest("mock_nodes")
import rospy
import sensor_msgs.msg
import rosbag

class MockPointCloud2:
    _PUB_RATE = 20
    def __init__(self):
        self._point_cloud2_pub = rospy.Publisher("/wam_base_kinect/depth_registered/points", sensor_msgs.msg.PointCloud2)
        self._pub_rate = rospy.Rate(MockPointCloud2._PUB_RATE)
        self._bag = rosbag.Bag("/home/jkizito/fuerte_workspace/mock_nodes/rosbags/kinect_can.bag")
#self._bag = rosbag.Bag("../bags/kinect_can.bag")

    def pub_point_cloud2(self):
        self._msg = sensor_msgs.msg.PointCloud2()
        for topic, msg, t in self._bag.read_messages():
            self._msg = msg
        while not rospy.is_shutdown():
            self._msg.header.stamp = rospy.Time.now()
            self._point_cloud2_pub.publish(msg)
            self._pub_rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("mock_point_cloud2_node")
        mock_point_cloud2 = MockPointCloud2()
        mock_point_cloud2.pub_point_cloud2() 
    except rospy.ROSInterruptException:
        pass

