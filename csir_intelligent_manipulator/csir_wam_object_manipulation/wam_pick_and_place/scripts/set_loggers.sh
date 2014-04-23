#! /bin/sh

# Set up logger level for debugging

#rosservice call /kinect_self_filter/set_logger_level ros ERROR
rosservice call /object_manipulator/set_logger_level ros.arm_kinematics_constraint_aware DEBUG
rosservice call /object_manipulator/set_logger_level ros.object_manipulator.manipulation DEBUG

