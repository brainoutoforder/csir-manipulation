/*
 * Copyright (c) 2014 CSIR. All rights reserved.
 * Use of this source code is governed by the MIT license that can be found in
 * the LICENSE file.
 */

#include "bhand_grasp_controller/bhand_grasp_controller.h"
//#include <boost/scoped_ptr.hpp>
#include <vector>
#include <gtest/gtest.h>

/*!
 * @brief Unit test fixture for BHandGraspController
 */
class BHandGraspControllerUTest : public ::testing::Test
{
 public:
  /*!
   * @brief The constructor serves only to initialise members of the test fixture
   * class.
   */
  BHandGraspControllerUTest() : bHandJoints_(BHandGraspController::DOF_, 0.0)
  {
    graspTol = (BHandGraspController::JOINT_TOL_ /
                BHandGraspController::MAX_RANGE_);
  }

 protected:
  //virtual void SetUp()
  //virtual void TearDown()

  /*! Class under test. */
  BHandGraspController bgc_;
  /*! Vector of state (in radians) of 3 proximal finger joints. */
  std::vector<double> bHandJoints_;
  double graspTol;
};

/*!
 * @brief Test convertJointsToGrasp function.
 * @details Test that input values yield expected values and that "small" changes
 * in the input are ignored.
 */
TEST_F(BHandGraspControllerUTest, convertJointsToGrasp)
{
  double theta = 0.0;
  double graspMetric = 0.0;
  // Expect disturbance invariant output
  for (double expectedOutput = 0.0; expectedOutput < 1.0; expectedOutput += 0.5)
  {
    theta = BHandGraspController::MAX_RANGE_ * expectedOutput;
    ROS_DEBUG("theta: %f", theta);
    for (double tol = -BHandGraspController::JOINT_TOL_; tol < BHandGraspController::JOINT_TOL_ + tol/10.0;
         tol += BHandGraspController::JOINT_TOL_)
    {
      bHandJoints_.assign(bHandJoints_.size(), theta + tol);
      ROS_DEBUG("bhj: %f, %f, %f", bHandJoints_.at(0), bHandJoints_.at(1), bHandJoints_.at(2));
      graspMetric = bgc_.convertJointsToGrasp(bHandJoints_);
      EXPECT_NEAR(graspMetric, expectedOutput, graspTol) <<
          "graspMetric too sensitive to joint variance";
    }
  }
  // Check deviations around range
  theta = 0.5 * BHandGraspController::MAX_RANGE_;
  bHandJoints_.assign(bHandJoints_.size(), theta);
  bHandJoints_.at(1) = 0.25 * BHandGraspController::MAX_RANGE_;
  graspMetric = bgc_.convertJointsToGrasp(bHandJoints_);
  EXPECT_LT(graspMetric, 0.5);
  EXPECT_GT(graspMetric, 0.0);
  bHandJoints_.at(1) = 0.75 * BHandGraspController::MAX_RANGE_;
  graspMetric = bgc_.convertJointsToGrasp(bHandJoints_);
  EXPECT_LT(graspMetric, 1.0);
  EXPECT_GT(graspMetric, 0.5);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "utest_bhand_grasp_controller");
  return RUN_ALL_TESTS();
}
