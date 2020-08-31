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

void PlaygroundMoveIfaceBase::startServicesAndRunNode(ros::NodeHandle* nh)
{
  ros::AsyncSpinner spinner(0);
  spinner.start();

  RLLMoveIfaceServices* move_iface_ptr = this;
  RLLMoveIfaceBase* move_iface_base_ptr = this;

  resetToHome();

  RLLMoveIfaceBase::JobServer server_job(  // NOLINT clang-analyzer-optin.cplusplus.VirtualCall
      *nh, RLLMoveIfaceBase::RUN_JOB_SRV_NAME, boost::bind(&RLLMoveIfaceBase::runJobAction, this, _1, &server_job),
      false);
  server_job.start();
  RLLMoveIfaceBase::JobServer server_idle(*nh, RLLMoveIfaceBase::IDLE_JOB_SRV_NAME,
                                          boost::bind(&RLLMoveIfaceBase::idleAction, this, _1, &server_idle), false);
  server_idle.start();

  ros::ServiceServer robot_ready = nh->advertiseService(RLLMoveIfaceServices::ROBOT_READY_SRV_NAME,
                                                        &RLLMoveIfaceServices::robotReadySrv, move_iface_ptr);
  ros::ServiceServer job_finished = nh->advertiseService(RLLMoveIfaceBase::JOB_FINISHED_SRV_NAME,
                                                         &RLLMoveIfaceBase::jobFinishedSrv, move_iface_base_ptr);
  ros::ServiceServer move_random = nh->advertiseService(RLLMoveIfaceServices::MOVE_RANDOM_SRV_NAME,
                                                        &RLLMoveIfaceServices::moveRandomSrv, move_iface_ptr);
  ros::ServiceServer move_lin =
      nh->advertiseService(RLLMoveIfaceServices::MOVE_LIN_SRV_NAME, &RLLMoveIfaceServices::moveLinSrv, move_iface_ptr);
  ros::ServiceServer move_lin_arm_angle = nh->advertiseService(
      RLLMoveIfaceServices::MOVE_LIN_ARMANGLE_SRV_NAME, &RLLMoveIfaceServices::moveLinArmangleSrv, move_iface_ptr);
  ros::ServiceServer move_ptp =
      nh->advertiseService(RLLMoveIfaceServices::MOVE_PTP_SRV_NAME, &RLLMoveIfaceServices::movePTPSrv, move_iface_ptr);
  ros::ServiceServer move_ptp_armangle = nh->advertiseService(
      RLLMoveIfaceServices::MOVE_PTP_ARMANGLE_SRV_NAME, &RLLMoveIfaceServices::movePTPArmangleSrv, move_iface_ptr);
  ros::ServiceServer move_joints = nh->advertiseService(RLLMoveIfaceServices::MOVE_JOINTS_SRV_NAME,
                                                        &RLLMoveIfaceServices::moveJointsSrv, move_iface_ptr);
  ros::ServiceServer get_pose = nh->advertiseService(RLLMoveIfaceServices::GET_POSE_SRV_NAME,
                                                     &RLLMoveIfaceServices::getCurrentPoseSrv, move_iface_ptr);
  ros::ServiceServer get_joint_values = nh->advertiseService(
      RLLMoveIfaceServices::GET_JOINT_VALUES_SRV_NAME, &RLLMoveIfaceServices::getCurrentJointValuesSrv, move_iface_ptr);

  ROS_INFO("RLL Robot Playground Interface started");
  ros::waitForShutdown();
}
