/*
 Copyright 2012 Barrett Technology <support@barrett.com>

 This file is part of barrett-ros-pkg.

 This version of barrett-ros-pkg is free software: you can redistribute it
 and/or modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation, either version 3 of the
 License, or (at your option) any later version.

 This version of barrett-ros-pkg is distributed in the hope that it will be
 useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this version of barrett-ros-pkg.  If not, see
 <http://www.gnu.org/licenses/>.

 Barrett Technology holds all copyrights on barrett-ros-pkg. As the sole
 copyright holder, Barrett reserves the right to release future versions
 of barrett-ros-pkg under a different license.

 File: wam_node.cpp
 Date: 5 June, 2012
 Author: Kyle Maroney
 */

#include <unistd.h>
#include <math.h>

#include <boost/thread.hpp> // BarrettHand threading
#include <boost/bind.hpp>

#include "wam_node/wam_node.h"

#include "ros/ros.h"
//#include "tf/transform_datatypes.h"
//#include <bullet/LinearMath/btQuaternion.h>
//#include <bullet/LinearMath/btMatrix3x3.h>

//#include "wam_msgs/RTJointPos.h"
//#include "wam_msgs/RTJointVel.h"
//#include "wam_msgs/RTCartPos.h"
//#include "wam_msgs/RTCartVel.h"
//#include "wam_msgs/RTOrtnPos.h"
//#include "wam_msgs/RTOrtnVel.h"
//#include "wam_srvs/GravityComp.h"
//#include "wam_srvs/Hold.h"
//#include "wam_srvs/PoseMove.h"
//#include "wam_srvs/CartPosMove.h"
//#include "wam_srvs/OrtnMove.h"
//#include "wam_srvs/BHandFingerPos.h"
//#include "wam_srvs/BHandGraspPos.h"
//#include "wam_srvs/BHandSpreadPos.h"
//#include "wam_srvs/BHandFingerVel.h"
//#include "wam_srvs/BHandGraspVel.h"
//#include "wam_srvs/BHandSpreadVel.h"
//#include "std_srvs/Empty.h"
//#include "sensor_msgs/JointState.h"
//#include "geometry_msgs/PoseStamped.h"

//#include <barrett/math.h> 
//#include <barrett/units.h>
//#include <barrett/systems.h>
//#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
//#include <barrett/systems/wam.h>
//#include <barrett/detail/stl_utils.h>

/*! @TODO */
//static const int PUBLISH_FREQ = 250; // Default Control Loop / Publishing Frequency
//static const int BHAND_PUBLISH_FREQ = 5; // Publishing Frequency for the BarretHand
//static const double SPEED = 0.03; // Default Cartesian Velocity

// The root namespace for libbarrett
//using namespace barrett;

//wam_main Function
template<size_t DOF>
int wam_main(int argc, char** argv, ::barrett::ProductManager& pm, ::barrett::systems::Wam<DOF>& wam)
{
  //BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  ros::init(argc, argv, "wam_node");
  WamNode<DOF> wam_node(wam);
  wam_node.init(pm);
  ros::Rate pub_rate(WamNode<DOF>::PUBLISH_FREQ);
  
  //pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

  while (ros::ok() && pm.getSafetyModule()->getMode() == ::barrett::SafetyModule::ACTIVE)
  {
    ros::spinOnce();
    wam_node.publishWam(pm);
    //wam_node.updateRT(pm);
    pub_rate.sleep();
  }

  return 0;
}
