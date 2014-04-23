/*
 * Copyright (c) 2014 csir_intelligent_manipulator authors. All rights reserved.
 * Use of this source code is governed by the MIT license that can be found in
 * the LICENSE file.
 * Author: Jimmy Kizito
 */

#include "bhand_grasp_controller/bhand_grasp_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bhand_grasp_controller");
  BHandGraspController bgc;
  bgc.start();
  ros::spin();
}

