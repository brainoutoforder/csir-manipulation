/*
 * Copyright (c) 2014 csir_intelligent_manipulator authors. All rights reserved.
 * Use of this source code is governed by the MIT license that can be found in
 * the LICENSE file.
 * Author: Jimmy Kizito
 */

#ifndef BHAND_GRASP_CONTROLLER_BHAND_GRASP_CONTROLLER_H
#define BHAND_GRASP_CONTROLLER_BHAND_GRASP_CONTROLLER_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <object_manipulation_msgs/GraspStatus.h>

/*!
 * @brief Controls Barrett Hand.
 */
class BHandGraspController
{
 public:
  BHandGraspController();
  virtual ~BHandGraspController();
  /*! Number of joints controlled for grasping. */
  static const unsigned int DOF_;
  /*! Number of joints controlled for grasping. */
  static const int GRASP_DOF_;
  /*! Rounding for joint readings in radians. */
  static const double JOINT_TOL_;
  /*! Max range of grasping joints (i.e. proximal finger joints) in radians. */
  static const double MAX_RANGE_;
  /*! Indeces of proximal joints in JointState message. */
  static const std::vector<int> PROXIMAL_JOINTS_;
  /*! Names of joints in JointState message. */
  static const std::vector<std::string> BHAND_NAMES_;
  /*! Threshold of grasp metric returned by [convertJointsToGrasp] at which 
   * object detected. */
  static const double DEFAULT_BHAND_OBJECT_PRESENCE_THRESHOLD;
  /*! Rate in Hz at which hand joint states published */
  /*! @todo make this a parameter */
  static const double PUB_FREQ_;
  /*!
   * @brief Converts joint values into a single grasp metric.
   * @details This implementation only examines the 3 proximal joints of the
   * hand.
   * @param  [in] bHandJoints Vector of Barrett Hand joint values in radians.
   * Varies from 0 (open) to 140 degrees (closed).
   * @returns graspMetric Varies from 0.0 (open) to 1.0 (closed)
   */
  double convertJointsToGrasp(const std::vector<double> &bHandJoints);
  /*! @brief Helper function to set value of constant vector PROXIMAL_JOINTS_ */
  static std::vector<int> getProximalJoints();
  /*! @brief Helper function to set value of constant vector PROXIMAL_JOINTS_ */
  static std::vector<std::string> getBHandNames();
  /*!
   * @brief Start JointState subscriber and action server.
   * @details Separate from constructor to facilitate testing of class methods.
   */
  void start();
 private:
  double decimalPlaces; // number of decimal places to round grasp metric to
  std::vector<double> bHandJoints_;
  ros::Subscriber jointStateSub_;
  ros::NodeHandle node_;
  actionlib::SimpleActionServer<object_manipulation_msgs::GraspHandPostureExecutionAction> actionServer_;
  ros::ServiceServer graspStatusSrv_;
  double objectPresenceThreshold_;
  ros::ServiceClient bHandSpreadClient_;
  ros::ServiceClient bHandGraspClient_;
  /*!
   * @brief Saves joint position in bHandJoints_
   * @param [in] msg JointState message.
   */
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
  /*!
   * @brief Returns true if hand occupied and false otherwise based on joint
   * state.
   * @param [in] req GraspStatus request
   * @param [out] res is_hand_occupied
   */
  bool serveGraspStatus(object_manipulation_msgs::GraspStatus::Request &req,
                        object_manipulation_msgs::GraspStatus::Response &res);  
  /*!
   * @brief Open or close hand depending on goal
   */
  void executeCallback(const object_manipulation_msgs::GraspHandPostureExecutionGoalConstPtr &goal);
  /*!
   * @brief Spread and open Barrett Hand
   */
  void openBHand();
  /*!
   * @brief Close Barrett fingers
   */
  void closeBHand();
};

#endif // BHAND_GRASP_CONTROLLER_BHAND_GRASP_CONTROLLER_H

