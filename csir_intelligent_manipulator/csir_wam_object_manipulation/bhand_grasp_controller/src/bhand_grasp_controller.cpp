/*
 * Copyright (c) 2014 csir_intelligent_manipulator authors. All rights reserved.
 * Use of this source code is governed by the MIT license that can be found in
 * the LICENSE file.
 * Author: Jimmy Kizito
 */

#include "bhand_grasp_controller/bhand_grasp_controller.h"
#define _USE_MATH_DEFINES
#include <cmath> // for M_PI
#include <boost/math/special_functions/round.hpp>
#include <ros/assert.h>
#include <wam_srvs/BHandSpreadPos.h>
#include <wam_srvs/BHandGraspPos.h>

// constant initialisation
const unsigned int BHandGraspController::DOF_(7);
const int BHandGraspController::GRASP_DOF_(3);
const double BHandGraspController::JOINT_TOL_(0.01);
const double BHandGraspController::MAX_RANGE_(7.0 / 9.0 * M_PI); //140 degrees
const std::vector<int> BHandGraspController::PROXIMAL_JOINTS_(
    BHandGraspController::getProximalJoints());
const std::vector<std::string> BHandGraspController::BHAND_NAMES_(
    BHandGraspController::getBHandNames());
const double BHandGraspController::DEFAULT_BHAND_OBJECT_PRESENCE_THRESHOLD(0.1);
const double BHandGraspController::PUB_FREQ_(10.0);

BHandGraspController::BHandGraspController() :
    bHandJoints_(BHandGraspController::DOF_, 0.0),
    actionServer_(node_, "posture_action_name", boost::bind(&BHandGraspController::executeCallback, this, _1), false)
{
  decimalPlaces = static_cast<int>(-log10(BHandGraspController::JOINT_TOL_ /
                                          BHandGraspController::MAX_RANGE_));
  graspStatusSrv_ = node_.advertiseService(
      "grasp_query_name", &BHandGraspController::serveGraspStatus, this);
  bHandSpreadClient_ = node_.serviceClient<wam_srvs::BHandSpreadPos>("/bhand/spread_pos");
  bHandGraspClient_ = node_.serviceClient<wam_srvs::BHandGraspPos>("/bhand/grasp_pos");
  ROS_INFO("Barrett Hand posture controller started");
}

BHandGraspController::~BHandGraspController()
{
}

void BHandGraspController::start()
{
  node_.param<double>(
      "bhand_object_presence_threshold",
      objectPresenceThreshold_,
      BHandGraspController::DEFAULT_BHAND_OBJECT_PRESENCE_THRESHOLD);
  jointStateSub_ = node_.subscribe("bhand/joint_states", 1,
                                   &BHandGraspController::jointStateCallback,
                                   this);
  actionServer_.start(); 
  bHandSpreadClient_.waitForExistence();
  bHandGraspClient_.waitForExistence();
  openBHand();
}

void BHandGraspController::jointStateCallback(const sensor_msgs::JointState::ConstPtr
                                         &msg)
{
  ROS_ASSERT(msg->position.size() == BHandGraspController::DOF_);
  //bHandJoints_ = msg->position;
  bHandJoints_.assign(msg->position.begin(), msg->position.end()); 
}

bool BHandGraspController::serveGraspStatus(
    object_manipulation_msgs::GraspStatus::Request &req,
    object_manipulation_msgs::GraspStatus::Response &res)
{
  if (convertJointsToGrasp(bHandJoints_) > objectPresenceThreshold_)
    res.is_hand_occupied = true;
  else
    res.is_hand_occupied = false;
  return true;
}

void BHandGraspController::executeCallback(const object_manipulation_msgs::GraspHandPostureExecutionGoalConstPtr &goal)
{
  switch (goal->goal)
  {
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP:
      openBHand();
      break;
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP:
      closeBHand();
      break;
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE:
      openBHand();
      break;
    default:
      ROS_ERROR("Unknown goal code");
      actionServer_.setAborted();
      return;
  }
  actionServer_.setSucceeded();
}

double BHandGraspController::convertJointsToGrasp(
    const std::vector<double> &bHandJoints)
{
  double jointSum = 0.0;
  std::vector<int>::const_iterator it;
  for (it = BHandGraspController::PROXIMAL_JOINTS_.begin();
       it != BHandGraspController::PROXIMAL_JOINTS_.end(); ++it)
  {
    jointSum += bHandJoints.at(*it);
    ROS_DEBUG("it: %d", *it);
    ROS_DEBUG("bhj: %f", bHandJoints.at(*it));
  }
  ROS_DEBUG("jointSum: %f", jointSum);
  double graspMetric = jointSum / BHandGraspController::MAX_RANGE_ / 3.0;
  ROS_DEBUG("Unrounded graspMetric is: %f", graspMetric);
  double tens = pow(10.0, static_cast<double>(decimalPlaces));
  ROS_DEBUG("tens: %f", tens);
  // round to decimalPlaces
  graspMetric = boost::math::round(graspMetric * tens) / tens; 
  ROS_DEBUG("Rounded graspMetric is: %f", graspMetric);
  return graspMetric;
}

void BHandGraspController::openBHand()
{
  ROS_DEBUG("Waiting to open");
  wam_srvs::BHandSpreadPos spreadSrv;
  spreadSrv.request.radians = M_PI;
  if (bHandSpreadClient_.call(spreadSrv))
    ROS_INFO("Spreading fingers");
  else
    ROS_ERROR("Failed to call spread service");
  wam_srvs::BHandGraspPos graspSrv;
  graspSrv.request.radians = 0.0;
  if (bHandGraspClient_.call(graspSrv))
    ROS_INFO("Opening fingers");
  else
    ROS_ERROR("Failed to call grasp service");
}

void BHandGraspController::closeBHand()
{
  ROS_DEBUG("Waiting to close");
  wam_srvs::BHandGraspPos graspSrv;
  graspSrv.request.radians = BHandGraspController::MAX_RANGE_;
  if (bHandGraspClient_.call(graspSrv))
    ROS_INFO("Closing fingers");
  else
    ROS_ERROR("Failed to call grasp service");
}

std::vector<int> BHandGraspController::getProximalJoints()
{
  std::vector<int> proximalJoints;
  proximalJoints.push_back(0);
  proximalJoints.push_back(1);
  proximalJoints.push_back(2);
  return proximalJoints;
}

std::vector<std::string> BHandGraspController::getBHandNames()
{
  std::vector<std::string> bHandNames;
  bHandNames.push_back("inner_f1");
  bHandNames.push_back("inner_f2");
  bHandNames.push_back("inner_f3");
  bHandNames.push_back("spread");
  bHandNames.push_back("outer_f1");
  bHandNames.push_back("outer_f2");
  bHandNames.push_back("outer_f3");
  return bHandNames;
}

