#ifndef WAM_NODE_H
#define WAM_NODE_H

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/math.h> 

#include "ros/ros.h"
#include "wam_srvs/Hold.h"
#include "wam_srvs/JointMove.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "wam_srvs/JointTrajectoryMove.h"

//WamNode Class
template<size_t DOF>
class WamNode
{
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  typedef boost::tuple<double, jp_type> jp_traj_type;
  ::barrett::systems::Wam<DOF>& wam_;
  ros::ServiceServer hold_jpos_srv_;
  ros::ServiceServer joint_move_srv_;
  ros::ServiceServer go_home_srv_;
  ros::ServiceServer joint_traj_move_srv_;
  jp_type jp_cmd_;
  ros::Publisher wam_joint_state_pub_;
  sensor_msgs::JointState wam_joint_state_;
  ::barrett::systems::Ramp traj_ramp_;
  // Trajectory pointers
  boost::shared_ptr<barrett::systems::Callback<double, jp_type> > traj_ptr_;
  boost::shared_ptr<barrett::math::Spline<jp_type> > traj_spline_ptr_;
  boost::shared_ptr<std::vector<jp_traj_type> > traj_vec_ptr_;
 public:
  WamNode(::barrett::systems::Wam<DOF>& wam);
  void init(::barrett::ProductManager& pm);
  bool holdJPos(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res);
  bool jointMove(wam_srvs::JointMove::Request &req, wam_srvs::JointMove::Response &res);
  bool goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool jointTrajectoryMove(wam_srvs::JointTrajectoryMove::Request &req, wam_srvs::JointTrajectoryMove::Response &res);
  void publishWam(::barrett::ProductManager& pm);
  static const int PUBLISH_FREQ = 250; // Default Control Loop / Publishing Frequency
  static const int CONTROL_FREQ = 500;
};

#include "wam_node/wam_node-inl.h"

#endif