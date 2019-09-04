/*
 * This file is part of the Robot Learning Lab Robot Playground project
 *
 * Copyright (C) 2019 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
 * Copyright (C) 2019 Mark Weinretuer <uieai@student.kit.edu>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <rll_robot_playground_project/playground_iface.h>

void PlaygroundMoveIfaceBase::startServicesAndRunNode(ros::NodeHandle& nh)
{
  ros::AsyncSpinner spinner(0);
  spinner.start();

  RLLMoveIface* move_iface_ptr = this;

  resetToHome();

  RLLMoveIface::JobServer server_job(nh, RLLMoveIface::RUN_JOB_SRV_NAME,
                                     boost::bind(&RLLMoveIface::runJobAction, this, _1, &server_job), false);
  server_job.start();
  RLLMoveIface::JobServer server_idle(nh, RLLMoveIface::IDLE_JOB_SRV_NAME,
                                      boost::bind(&RLLMoveIface::idleAction, this, _1, &server_idle), false);
  server_idle.start();

  ros::ServiceServer robot_ready =
      nh.advertiseService(RLLMoveIface::ROBOT_READY_SRV_NAME, &RLLMoveIface::robotReadySrv, move_iface_ptr);
  ros::ServiceServer move_random =
      nh.advertiseService(RLLMoveIface::MOVE_RANDOM_SRV_NAME, &RLLMoveIface::moveRandomSrv, move_iface_ptr);
  ros::ServiceServer move_lin =
      nh.advertiseService(RLLMoveIface::MOVE_LIN_SRV_NAME, &RLLMoveIface::moveLinSrv, move_iface_ptr);
  ros::ServiceServer move_ptp =
      nh.advertiseService(RLLMoveIface::MOVE_PTP_SRV_NAME, &RLLMoveIface::movePTPSrv, move_iface_ptr);
  ros::ServiceServer move_joints =
      nh.advertiseService(RLLMoveIface::MOVE_JOINTS_SRV_NAME, &RLLMoveIface::moveJointsSrv, move_iface_ptr);
  ros::ServiceServer get_pose =
      nh.advertiseService(RLLMoveIface::GET_POSE_SRV_NAME, &RLLMoveIface::getCurrentPoseSrv, move_iface_ptr);
  ros::ServiceServer get_joint_values = nh.advertiseService(RLLMoveIface::GET_JOINT_VALUES_SRV_NAME,
                                                            &RLLMoveIface::getCurrentJointValuesSrv, move_iface_ptr);

  ROS_INFO("RLL Robot Playground Interface started");
  ros::waitForShutdown();
}
