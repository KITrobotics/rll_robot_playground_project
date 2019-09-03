/*
 * This file is part of the Robot Learning Lab Robot Playground project
 *
 * Copyright (C) 2019 Mark Weinreuter <uieai@student.kit.edu>
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

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rll_msgs/DefaultMoveIfaceAction.h>
#include <rll_msgs/DefaultMoveIfaceActionGoal.h>
#include <rll_msgs/DefaultMoveIfaceResult.h>

#include <rll_msgs/MoveJoints.h>
#include <rll_msgs/MovePTP.h>
#include <rll_msgs/MoveLin.h>
#include <rll_msgs/MoveRandom.h>
#include <rll_msgs/GetJointValues.h>
#include <rll_msgs/GetPose.h>
#include <rll_move/move_iface.h>

void helloWorld(ros::NodeHandle* const nh)
{
  // initialize the service clients for the available movement services
  // see: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
  ros::ServiceClient move_joints = nh->serviceClient<rll_msgs::MoveJoints>(RLLMoveIface::MOVE_JOINTS_SRV_NAME);
  ros::ServiceClient move_ptp = nh->serviceClient<rll_msgs::MovePTP>(RLLMoveIface::MOVE_PTP_SRV_NAME);
  ros::ServiceClient move_lin = nh->serviceClient<rll_msgs::MoveLin>(RLLMoveIface::MOVE_LIN_SRV_NAME);
  ros::ServiceClient move_random = nh->serviceClient<rll_msgs::MoveRandom>(RLLMoveIface::MOVE_RANDOM_SRV_NAME);

  // A service call always requires a Request and Response object. The request contains information,
  // such as where to move to, where as the response provides feedback e.g. was the call successfull
  rll_msgs::MoveRandom::Request random_req;
  rll_msgs::MoveRandom::Response random_resp;
  rll_msgs::MoveJoints::Request joints_req;
  rll_msgs::MoveJoints::Response joints_resp;
  rll_msgs::MovePTP::Request ptp_req;
  rll_msgs::MovePTP::Response ptp_resp;
  rll_msgs::MoveLin::Request lin_req;
  rll_msgs::MoveLin::Response lin_resp;

  std::cout << "Hello World" << std::endl;  // avoid cout for logging
  ROS_INFO("Hello ROS");                    // better use ROS_INFO, ROS_ERROR...

  // move to a random pose, the Request object requires no arguments
  ROS_INFO("calling move_random service");
  move_random.call(random_req, random_resp);

  // The robot should now be moving (in RViz). The delays in this code
  // are only for illustrative purposes and can be removed
  ros::Duration(2).sleep();

  // move all seven joints into their zero (initial) position
  // set all joint values of the Request to zero (uneccessary, zero is the default)
  joints_req.joint_1 = 0;
  joints_req.joint_2 = 0;
  joints_req.joint_3 = 0;
  joints_req.joint_4 = 0;
  joints_req.joint_5 = 0;
  joints_req.joint_6 = 0;
  joints_req.joint_7 = 0;

  ROS_INFO("calling move_joints with all joints values = 0");
  move_joints.call(joints_req, joints_resp);

  ros::Duration(2).sleep();

  // rotate the fourth joint by 90 degrees (pi/2 since we work with radians)
  // the remaining joint values are still equal to zero
  joints_req.joint_4 = M_PI / 2;
  ROS_INFO("calling move_joints with joint_4 = pi/2");
  move_joints.call(joints_req, joints_resp);

  // previously we neglected to check the response of the service call.
  // You should always check the result of a service call!
  if (!joints_resp.success)
  {
    ROS_ERROR("move_joints service call failed (as expected)");
  }

  // ups, moving the fourth joint by 90 degrees didn't work, we bumped into
  // the workspace boundaries -> try moving 90 degrees in the other direction
  joints_req.joint_4 = -M_PI / 2;
  ROS_INFO("calling move_joints with joint_4 = -pi/2");
  move_joints.call(joints_req, joints_resp);

  if (joints_resp.success)
  {
    ROS_INFO("move_joints service call succeeded!");
  }
  else
  {
    ROS_ERROR("move_joints service call failed (unexpectately)!");
  }

  ros::Duration(2).sleep();

  // moving by specifying joint angle values is not the most intuitive way
  // it's easier to specify the pose of the endeffector we'd like to reach
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = .5;
  goal_pose.position.y = .2;
  goal_pose.position.z = .7;
  goal_pose.orientation.z = 1;  // rotate 180 degrees around z (see below)

  // the move_ptp Request requires a Pose argument
  ptp_req.pose = goal_pose;
  ROS_INFO("calling move_ptp service with:");
  ROS_INFO_STREAM(ptp_req.pose);
  move_ptp.call(ptp_req, ptp_resp);

  // not all poses can be reached, always check the success of the operation
  if (ptp_resp.success)
  {
    ROS_INFO("move_ptp service call succeeded!");
  }
  else
  {
    ROS_ERROR("move_ptp service call failed");
  }

  ros::Duration(1).sleep();

  // The orientation of a pose is stored as a quaternion and usually you
  // don't specify them manually. It's easier to e.g. use euler or RPY angles
  // HINT: display the coordinate systems in RViz to visualize orientations
  // In RViz the XYZ axes are color coded in RGB: X=red, Y=green, Z=blue
  // the end effector is pointing along the blue z-axis

  tf2::Quaternion quaternion;
  quaternion.setRPY(0, M_PI / 2, 0);

  // copy the quaternion values to the request
  goal_pose.orientation.x = quaternion.x();
  goal_pose.orientation.y = quaternion.y();
  goal_pose.orientation.z = quaternion.z();
  goal_pose.orientation.w = quaternion.w();

  // move to same position but different orientation
  ptp_req.pose = goal_pose;

  ROS_INFO("move_ptp to the same position but different orientation:");
  ROS_INFO_STREAM(ptp_req.pose);
  move_ptp.call(ptp_req, ptp_resp);  // (error check omitted)

  ros::Duration(1).sleep();

  // rotate 90deg around the y-axis and 45deg around the x-axis
  quaternion.setRPY(M_PI / 4, M_PI / 2, 0);
  goal_pose.orientation.x = quaternion.x();
  goal_pose.orientation.y = quaternion.y();
  goal_pose.orientation.z = quaternion.z();
  goal_pose.orientation.w = quaternion.w();

  ptp_req.pose = goal_pose;

  ROS_INFO("move_ptp to the same position but different orientation:");
  ROS_INFO_STREAM(ptp_req.pose);
  move_ptp.call(ptp_req, ptp_resp);  // (error check omitted)

  ros::Duration(2).sleep();

  // Next up: move the endeffector on a triangular path
  // while maintaining the same orientation
  ROS_INFO("Next: move the endeffector on a triangluar path");

  // orient the z-axis "forward" (along the base x-axis)
  quaternion.setRPY(0, M_PI / 2, 0);
  goal_pose.orientation.x = quaternion.x();
  goal_pose.orientation.y = quaternion.y();
  goal_pose.orientation.z = quaternion.z();
  goal_pose.orientation.w = quaternion.w();

  // move to the starting position still in a ptp fashion
  goal_pose.position.x = 0.5;
  goal_pose.position.y = -0.6;
  goal_pose.position.z = 0.25;
  ptp_req.pose = goal_pose;

  ROS_INFO("move_ptp to the starting point of the triangle:");
  ROS_INFO_STREAM(ptp_req.pose);
  move_ptp.call(ptp_req, ptp_resp);  // (error check omitted)

  ros::Duration(1).sleep();

  // move up, its a right angled triangle
  goal_pose.position.z = .7;

  // this time we move on a linear trajectory to the specified pose
  lin_req.pose = goal_pose;
  lin_req.cartesian_time_parametrization = true;

  ROS_INFO("move_lin to the tip of the triangle:");
  ROS_INFO_STREAM(lin_req.pose);
  move_lin.call(lin_req, lin_resp);

  ros::Duration(1).sleep();

  // next point is the upper right point of the triangle
  goal_pose.position.y = -0.1;
  lin_req.pose = goal_pose;

  ROS_INFO("move_lin to the upper right point of the triangle:");
  ROS_INFO_STREAM(lin_req.pose);
  move_lin.call(lin_req, lin_resp);

  ros::Duration(1).sleep();

  // close the triangle by moving back diagonally to the start position
  goal_pose.position.y = -0.6;
  goal_pose.position.z = .25;
  lin_req.pose = goal_pose;

  ROS_INFO("move_lin to the start to close the triangle shape:");
  ROS_INFO_STREAM(lin_req.pose);
  move_lin.call(lin_req, lin_resp);

  ros::Duration(1).sleep();

  // note: move_lin is not always successful, even if move_ptp succeeds.
  // This is because moving on a linear trajectory is more constraining
  // Example: move to a positive y-position will fail with move_lin
  goal_pose.position.y = 0.3;
  lin_req.pose = goal_pose;

  ROS_INFO("try to move_lin to:");
  ROS_INFO_STREAM(lin_req.pose);
  move_lin.call(lin_req, lin_resp);

  if (!lin_resp.success)
  {
    ROS_ERROR("move_lin service call failed (as expected)");
  }

  // calling move_ptp with the exact same goal pose succeeds
  ptp_req.pose = goal_pose;

  ROS_INFO("try to move_ptp to:");
  ROS_INFO_STREAM(ptp_req.pose);
  move_ptp.call(ptp_req, ptp_resp);

  if (ptp_resp.success)
  {
    ROS_INFO("move_ptp service call succeeded");
  }
  else
  {
    ROS_ERROR("move_ptp service call failed");
  }

  ros::Duration(2).sleep();

  // the Response object sometimes holds more information than only success
  ROS_INFO("move_random to a new random position");
  move_random.call(random_req, random_resp);  // (error check omitted)

  // we can obtain the chosen random pose from the response
  ROS_INFO("move_random moved to: ");
  ROS_INFO_STREAM(random_resp.pose);

  ros::Duration(2).sleep();
}

void actionCallback(ros::NodeHandle* const nh,
                    actionlib::SimpleActionServer<rll_msgs::DefaultMoveIfaceAction>* const server,
                    const rll_msgs::DefaultMoveIfaceGoalConstPtr& goal)
{
  ROS_INFO("Action invoked");

  // run the actual movement code
  helloWorld(nh);
  server->setSucceeded();

  ROS_INFO("Action completed");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_playground");

  ros::NodeHandle nh;
  // when the 'move_client' action is called  run the 'actionCallback'
  // see: http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
  actionlib::SimpleActionServer<rll_msgs::DefaultMoveIfaceAction> server(
      nh, "move_client", boost::bind(actionCallback, &nh, &server, _1), false);

  server.start();
  ROS_INFO("Action server started");

  ros::spin();
  return 0;
}
