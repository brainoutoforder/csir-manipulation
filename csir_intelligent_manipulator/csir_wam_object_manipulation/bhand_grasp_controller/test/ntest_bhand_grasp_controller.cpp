/*
 * Copyright (c) 2014 csir_intelligent_manipulator authors. All rights reserved.
 * Use of this source code is governed by the MIT license that can be found in
 * the LICENSE file.
 * Author: Jimmy Kizito
 */

#include "bhand_grasp_controller/bhand_grasp_controller.h"
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <wam_srvs/BHandSpreadPos.h>
#include <wam_srvs/BHandGraspPos.h>
#include <boost/thread.hpp>
#include <vector>
#include <ros/ros.h>
#include <string>
#include <gtest/gtest.h>

/*!
 * @brief Node test fixture for testing bhand_grasp_controller.
 */
class BHandGraspControllerNTest : public ::testing::Test
{
 public:
  /*!
   * @brief Initialise class members before test
   */
  BHandGraspControllerNTest() :
      bHandPos_(BHandGraspController::DOF_, 0.0),
      bHandNames_(BHandGraspController::BHAND_NAMES_),
      spinner_(1),
      graspActionClient_("posture_action_name", true)
  {
    bool hasActionServerStarted =
        graspActionClient_.waitForServer(ros::Duration(10.0));
    EXPECT_TRUE(hasActionServerStarted) << "Action server did not start";
    bHandPub_ = node_.advertise<sensor_msgs::JointState>("bhand/joint_states", 1);
    bHandSpreadServiceServer_ =
        node_.advertiseService("bhand/spread_pos",
                               &BHandGraspControllerNTest::simBHandSpread,
                               this);
    bHandGraspServiceServer_ =
        node_.advertiseService("bhand/grasp_pos",
                               &BHandGraspControllerNTest::simBHandGrasp,
                               this);
    graspQueryClient_ =
        node_.serviceClient<object_manipulation_msgs::GraspStatus>(
            "grasp_query_name");
    spinner_.start();
    pubBHandThread_ = boost::thread(&BHandGraspControllerNTest::pubBHand, this);
  }
  /*!
   * @brief Stop thread before destroying
   */
  virtual ~BHandGraspControllerNTest()
  {
    pubBHandThread_.interrupt();
    pubBHandThread_.join();
  }
  /*!
   * @brief Puclish the joint state in bHandPos_ on topic bhand/joint_states.
   */
  void pubBHand()
  {
    ros::Rate pubRate(BHandGraspController::PUB_FREQ_);
    while (ros::ok())
    {
      try
      {
        boost::this_thread::interruption_point();
        sensor_msgs::JointState bHandMsg;
        bHandMsg.header.stamp = ros::Time::now();
        bHandMsg.name = bHandNames_;
        bHandMsg.position = bHandPos_;
        bHandMsg.velocity = std::vector<double>(BHandGraspController::DOF_, 0.0);
        bHandMsg.effort = std::vector<double>(BHandGraspController::DOF_, 0.0);
        bHandPub_.publish(bHandMsg);
        pubRate.sleep();
      }
      catch (boost::thread_interrupted &e)
      {
        break;
      }
    }
  }
  /*!
   * @brief Query the GraspStatus service.
   * @return true if hand occupied, false otherwise
   */
  bool queryGraspService()
  {
    ROS_DEBUG("Querying grasp status");
    object_manipulation_msgs::GraspStatus serviceRequest;
    if (graspQueryClient_.call(serviceRequest))
    {
      ROS_DEBUG("Received grasp status");
      return serviceRequest.response.is_hand_occupied;
    }
    else
    {
      //can only use assertions in void returning functions
      EXPECT_TRUE(false) << "Failed to call GraspStatus service";
      return false;
    }
  }
  /*!
   * @brief Send grasp goal to grasp action server.
   * @param [in] goal PRE_GRASP=1, GRASP=2, RELEASE=3
   */
  void sendGoal(const int &goal)
  {
    graspGoal_.goal = goal;
    graspActionClient_.sendGoal(graspGoal_);
    graspActionClient_.waitForResult();
  }
  /*!
   * @brief Simulate the BHandSpreadPos service in the Barrett wam_node.
   * @details Changes the finger spread angle in the hand joint state.
   * @param [in] req The requested spread angle in radians.
   * @param [out] res Unused
   */
  bool simBHandSpread(wam_srvs::BHandSpreadPos::Request &req,
                      wam_srvs::BHandSpreadPos::Response &res)
  {
    ROS_DEBUG("Spreading");
    bHandPos_.at(3) = req.radians;
    return true;
  }
  /*!
   * @brief Simulate the BHandGraspPos service in the Barrett wam_node.
   * @details The three proximal joints of the Barrett hand are set to the
   * requsted angle.
   * @param [in] req The requested proximal joint angles in radians.
   * @param [out] res Unused
   */
  bool simBHandGrasp(wam_srvs::BHandSpreadPos::Request &req,
                     wam_srvs::BHandSpreadPos::Response &res)
  {
    ROS_DEBUG("Grasping");
    for (std::vector<int>::const_iterator it =
         BHandGraspController::PROXIMAL_JOINTS_.begin(); it !=
         BHandGraspController::PROXIMAL_JOINTS_.end(); ++it)
    {
      bHandPos_.at(*it) = req.radians;
    }
    return true;
  }
  /*!
   * @brief Sleep for a fracional number of hand publisher periods rounded to
   * nearest millisecond.
   * @param [in] numPeriods Number of periods to sleep for.
   */
  void sleepFor(const double &numPeriods)
  {
    boost::this_thread::sleep(
        boost::posix_time::milliseconds(numPeriods /
                                        BHandGraspController::PUB_FREQ_*1000));
  }
  /*! Listo of hand joint values in radians*/
  std::vector<double> bHandPos_;

 protected:
  virtual void SetUp()
  {
  }

  virtual void TearDown()
  {
  }

 private:
  /*! Names of joints in Barrett Hand used in joint state messages*/
  std::vector<std::string> bHandNames_;
  ros::AsyncSpinner spinner_; // process callbacks in separate thread
  boost::thread pubBHandThread_;
  ros::NodeHandle node_;
  object_manipulation_msgs::GraspHandPostureExecutionGoal graspGoal_;
  ros::Publisher bHandPub_;
  actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> graspActionClient_;
  ros::ServiceServer bHandSpreadServiceServer_;
  ros::ServiceServer bHandGraspServiceServer_;
  ros::ServiceClient graspQueryClient_;
};

/*!
 * @brief Test GraspStatus service.
 * @details Sleep used to allow for new joint state to be published.
 * # Wait for hand to initialise
 * # Verify that hand initially open
 * # Simulate hand closed
 * # Verify hand occupied
 * # Simulate hand partially open
 * # Verify hand occupied
 * # Simulte hand open
 * # Verify hand unoccupied
 */
TEST_F(BHandGraspControllerNTest, graspQuery)
{
  sleepFor(10.0);
  ROS_DEBUG("Test");
  bool isOccupied = queryGraspService();
  ASSERT_FALSE(isOccupied) << "Hand not initially empty";
  std::vector<int>::const_iterator it; 
  for (it = BHandGraspController::PROXIMAL_JOINTS_.begin(); it !=
       BHandGraspController::PROXIMAL_JOINTS_.end(); ++it)
  { // sim close
    bHandPos_.at(*it) = BHandGraspController::MAX_RANGE_;
  }
  sleepFor(1.5);
  isOccupied = queryGraspService();
  ASSERT_TRUE(isOccupied) << "Hand should be occupied";
  for (it = BHandGraspController::PROXIMAL_JOINTS_.begin(); it !=
       BHandGraspController::PROXIMAL_JOINTS_.end(); ++it)
  { // sim partially open
    bHandPos_.at(*it) = BHandGraspController::MAX_RANGE_ * 0.5;
  }
  sleepFor(1.5);
  isOccupied = queryGraspService();
  ASSERT_TRUE(isOccupied) << "Hand should be occupied";
  for (it = BHandGraspController::PROXIMAL_JOINTS_.begin(); it !=
       BHandGraspController::PROXIMAL_JOINTS_.end(); ++it)
  { // sim open 
    bHandPos_.at(*it) = 0.0;
  }
  sleepFor(1.5);
  isOccupied = queryGraspService();
  ASSERT_FALSE(isOccupied) << "Hand should be empty";
}

/*!
 * @brief Test GraspHandPostureExecution action.
 * @details Do twice:
 * # Send pre-grasp goal
 * # Verify hand open
 * # Send grasp goal
 * # Verify hand closed
 * # Send release goal
 * # Verify hand open
 */
TEST_F(BHandGraspControllerNTest, graspExecution)
{
  //boost::thread bHandThread(BHandGraspControllerNTest::pubBHand, this);
  for (int i = 0; i < 2; ++i)
  {
    sendGoal(object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP);
    sleepFor(2.5);
    ASSERT_FALSE(queryGraspService()) << "Hand should be empty";
    sendGoal(object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP);
    sleepFor(2.5);
    ASSERT_TRUE(queryGraspService()) << "Hand should be occupied";
    sendGoal(object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE);
    sleepFor(2.5);
    ASSERT_FALSE(queryGraspService()) << "Hand should be empty";
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ntest_bhand_grasp_controller");
  return RUN_ALL_TESTS();
}

