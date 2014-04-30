#include <barrett/systems/wam.h>
#include <math.h>

#include "wam_node/wam_node.h"

template<size_t DOF>
WamNode<DOF>::WamNode(::barrett::systems::Wam<DOF>& wam) : wam_(wam), traj_ramp_(NULL, 1.0)
{
}

template<size_t DOF>
void WamNode<DOF>::init(::barrett::ProductManager& pm)
{
  wam_.gravityCompensate(true); // Turning on Gravity Compenstation by Default when starting the WAM Node
  pm.getExecutionManager()->startManaging(traj_ramp_); //starting traj_ramp manager
  ros::NodeHandle n_("wam"); // WAM specific nodehandle
  hold_jpos_srv_ = n_.advertiseService("hold_joint_pos", &WamNode::holdJPos, this); // wam/hold_joint_pos
  joint_move_srv_ = n_.advertiseService("joint_move", &WamNode::jointMove, this); // wam/joint_move
  go_home_srv_ = n_.advertiseService("go_home", &WamNode::goHome, this); // wam/go_home
  wam_joint_state_pub_ = n_.advertise <sensor_msgs::JointState > ("joint_states", 1); // wam/joint_states
  joint_traj_move_srv_ = n_.advertiseService("joint_traj_move", &WamNode::jointTrajectoryMove, this); // wam/joint_traj_move
  
  //Setting up WAM joint state publisher
  const char* wam_jnts[] = {"wam_j1", "wam_j2", "wam_j3", "wam_j4", "wam_j5", "wam_j6", "wam_j7"};
  std::vector < std::string > wam_joints(wam_jnts, wam_jnts + 7);
  wam_joint_state_.name = wam_joints;
  wam_joint_state_.name.resize(DOF);
  wam_joint_state_.position.resize(DOF);
  wam_joint_state_.velocity.resize(DOF);
  wam_joint_state_.effort.resize(DOF);
}

template<size_t DOF>
bool WamNode<DOF>::holdJPos(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res)
{
  ROS_INFO("Joint Position Hold request: %s", (req.hold) ? "true" : "false");
  if (req.hold)
    wam_.moveTo(wam_.getJointPositions());
  else
    wam_.idle();
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::jointMove(wam_srvs::JointMove::Request &req, wam_srvs::JointMove::Response &res)
{
  if (req.joints.size() != DOF)
  {
    ROS_INFO("Request Failed: %zu-DOF request received, must be %zu-DOF", req.joints.size(), DOF);
    return false;
  }
  ROS_INFO("Moving Robot to Commanded Joint Pose");
  for (size_t i = 0; i < DOF; i++)
    jp_cmd_[i] = req.joints[i];
  wam_.moveTo(jp_cmd_, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  wam_.moveHome();
  return true;
}

//Function to update the WAM publisher
template<size_t DOF>
void WamNode<DOF>::publishWam(::barrett::ProductManager& pm)
{
  //Current values to be published
  jp_type jp = wam_.getJointPositions();
  jt_type jt = wam_.getJointTorques();
  jv_type jv = wam_.getJointVelocities();
  cp_type cp_pub = wam_.getToolPosition();
  //Eigen::Quaterniond to_pub = wam_.getToolOrientation();

  //publishing sensor_msgs/JointState to wam/joint_states
  for (size_t i = 0; i < DOF; i++)
  {
    wam_joint_state_.position[i] = jp[i];
    wam_joint_state_.velocity[i] = jv[i];
    wam_joint_state_.effort[i] = jt[i];
  }
  wam_joint_state_.header.stamp = ros::Time::now();
  wam_joint_state_pub_.publish(wam_joint_state_);

  /*
  //publishing geometry_msgs/PoseStamed to wam/pose
  wam_pose.header.stamp = ros::Time::now();
  wam_pose.pose.position.x = cp_pub[0];
  wam_pose.pose.position.y = cp_pub[1];
  wam_pose.pose.position.z = cp_pub[2];
  wam_pose.pose.orientation.w = to_pub.w();
  wam_pose.pose.orientation.x = to_pub.x();
  wam_pose.pose.orientation.y = to_pub.y();
  wam_pose.pose.orientation.z = to_pub.z();
  wam_pose_pub.publish(wam_pose);
  */
}

template<size_t DOF>
bool WamNode<DOF>::jointTrajectoryMove(wam_srvs::JointTrajectoryMove::Request &req, wam_srvs::JointTrajectoryMove::Response &res)
{
  wam_.idle();
  traj_ramp_.stop();

  size_t num_points = req.trajectory.points.size();
  if (num_points == 0)
  {
    ROS_INFO("Request failed: no points in trajectory msg");
    res.success = false;
  }

  // get joint pos
  traj_vec_ptr_.reset(new std::vector<jp_traj_type>);
  for (size_t i = 0; i < num_points; i++)
  {
    jp_type jp_req;
    jp_req[0] = req.trajectory.points[i].positions[0];
    jp_req[1] = req.trajectory.points[i].positions[1];
    jp_req[2] = req.trajectory.points[i].positions[2];
    jp_req[3] = req.trajectory.points[i].positions[3];
    jp_req[4] = req.trajectory.points[i].positions[4];
    jp_req[5] = req.trajectory.points[i].positions[5];
    jp_req[6] = req.trajectory.points[i].positions[6];

    // Convert time from start of trajectory from nanoseconds to milliseconds and round to the nearest millisecond
    double time_ms = floor(static_cast<double>(req.trajectory.points[i].time_from_start.nsec) * 1.0e-6 + 0.5);
    // Ensures that time stamp has 2 ms resolution (i.e is even)
    time_ms = (static_cast<int>(floor(time_ms)) % 2 == 0) ? time_ms : time_ms + 1.0;
    // Converts milliseconds to seconds
    double time = time_ms / 1000.0;
    // Include seconds in time because time_from_start.nsec is only the fractional part of the time_from_start
    time += static_cast<double>(req.trajectory.points[i].time_from_start.sec);
    // ROS_INFO_STREAM("time: " << time);
    // if time step less than control period discard oldest entry
    double time_prev = 0;
    if (!traj_vec_ptr_->empty()) 
    {
      //time_prev = boost::tuples::get<0>(traj_vec.back());
      time_prev = boost::tuples::get<0>(traj_vec_ptr_->back());
      if ((time - time_prev) < (1.0 / static_cast<double>(WamNode::CONTROL_FREQ)))
      {
	//traj_vec.pop_back();
	traj_vec_ptr_->pop_back();
      }
    }
    jp_traj_type point(time, jp_req);
    traj_vec_ptr_->push_back(point);
  }

  //math::Spline<jp_type> traj_spline(traj_vec);
  traj_spline_ptr_.reset(new ::barrett::math::Spline<jp_type>(*traj_vec_ptr_));

  wam_.moveTo(traj_spline_ptr_->eval(traj_spline_ptr_->initialS()));

  traj_ramp_.setOutput(traj_spline_ptr_->initialS());

  traj_ptr_.reset(new ::barrett::systems::Callback<double, jp_type>(boost::ref(*traj_spline_ptr_)));
  ::barrett::systems::forceConnect(traj_ramp_.output, traj_ptr_->input);/*! @todo use systems::forceConnect?*/
  wam_.trackReferenceSignal(traj_ptr_->output);

  traj_ramp_.start();

  res.success = true;
  return true;
}

