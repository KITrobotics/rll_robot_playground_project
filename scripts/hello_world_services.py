#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab Robot Playground project
#
# Copyright (C) 2019 Mark Weinreuter <uieai@student.kit.edu>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import time
import rospy
from rll_msgs.srv import MoveJoints, MovePTP, MoveRandom, MoveLin
from rll_msgs.msg import DefaultMoveIfaceAction
import actionlib
from math import pi
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


def hello_world():

    # service proxies used to call the available services
    move_joints_service = rospy.ServiceProxy("move_joints", MoveJoints)
    move_ptp_service = rospy.ServiceProxy("move_ptp", MovePTP)
    move_lin_service = rospy.ServiceProxy("move_lin", MoveLin)
    move_random_service = rospy.ServiceProxy("move_random", MoveRandom)

    print("Hello world")  # avoid using print() for logging
    rospy.loginfo("Hello ROS")  # better use rospy.loginfo(), logerror()...

    # move to a random pose, this service call requires no arguments
    rospy.loginfo("calling move_random service")
    move_random_service.call()

    # The robot should now be moving (in RViz). The delays in this code
    # are only for illustrative purposes and can be removed
    time.sleep(2)

    # move all seven joints into their zero position by calling the move_joints
    # service and passing the joint values as arguments
    rospy.loginfo("calling move_joints with all joint values = 0")
    move_joints_service.call(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    time.sleep(2)

    # rotate the fourth joint by 90 degrees (pi/2 since we work with radians)
    # the remaining joint values are still equal to zero
    rospy.loginfo("calling move_joints with joint_4 = pi/2")
    resp = move_joints_service.call(0.0, 0.0, 0.0, pi / 2, 0.0, 0.0, 0.0)

    # previously we neglected to check the response of the service call.
    # You should always check the result of a service call!
    if not resp.success:
        rospy.logerr("move_joints service call failed (as expected)")

    # ups, moving the fourth joint by 90 degrees didn't work, we bumped into
    # the workspace boundaries -> try moving 90 degrees in the other direction
    rospy.loginfo("calling move_joints with joint_4 = -pi/2")
    move_joints_service.call(0.0, 0.0, 0.0, -pi / 2, 0.0, 0.0, 0.0)

    if resp.success:
        rospy.loginfo("move_joints service call succeeded!")
    if resp.success:
        rospy.logerr("move_joints service call failed (unexpectedly)!")

    time.sleep(2)

    # moving by specifying joint angle values is not the most intuitive way
    # it's easier to specify the pose of the end effector we'd like to reach
    goal_pose = Pose()
    goal_pose.position.x = .5
    goal_pose.position.y = .2
    goal_pose.position.z = .7
    goal_pose.orientation.z = 1  # rotate 180 degrees around z (see below)

    # the move_ptp service call requires a Pose argument
    rospy.loginfo("calling move_ptp service with: %s", goal_pose)
    resp = move_ptp_service.call(goal_pose)

    # not all poses can be reached, always check the success of the operation
    if resp.success:
        rospy.loginfo("move_ptp service call succeeded!")
    else:
        rospy.logerr("move_ptp service call failed")

    time.sleep(2)

    # The orientation of a pose is stored as a quaternion and usually you
    # don't specify them manually. It's easier to e.g. use euler or RPY angles
    # HINT: display the coordinate systems in RViz to visualize orientations
    # In RViz the XYZ axes are color coded in RGB: X=red, Y=green, Z=blue
    # the end effector is pointing along the blue z-axis
    
    quaternion_array = quaternion_from_euler(0, pi / 2, 0, 'sxyz')
    goal_pose.orientation.x = quaternion_array[0]
    goal_pose.orientation.y = quaternion_array[1]
    goal_pose.orientation.z = quaternion_array[2]
    goal_pose.orientation.w = quaternion_array[3]

    # move to same position but different orientation
    rospy.loginfo("move_ptp same position but different orientation: %s",
                  goal_pose)
    move_ptp_service.call(goal_pose)  # (error check omitted)

    time.sleep(1)

    # rotate 90deg around the y-axis and 45deg around the x-axis
    quaternion_array = quaternion_from_euler(pi / 2, pi / 4, 0, 'ryxz')
    goal_pose.orientation.x = quaternion_array[0]
    goal_pose.orientation.y = quaternion_array[1]
    goal_pose.orientation.z = quaternion_array[2]
    goal_pose.orientation.w = quaternion_array[3]

    rospy.loginfo("move_ptp same position but different orientation: %s",
                  goal_pose)
    move_ptp_service.call(goal_pose)  # (error check omitted)

    time.sleep(2)

    # Next up: move the end effector on a triangular path
    # while maintaining the same orientation
    rospy.loginfo("Next: move the end effector on a triangular path")

    # orient the z-axis "forward" (along the base x-axis)
    quaternion_array = quaternion_from_euler(0, pi / 2, 0, 'sxyz')
    goal_pose.orientation.x = quaternion_array[0]
    goal_pose.orientation.y = quaternion_array[1]
    goal_pose.orientation.z = quaternion_array[2]
    goal_pose.orientation.w = quaternion_array[3]

    # move to the starting position still in a ptp fashion
    goal_pose.position.x = 0.5
    goal_pose.position.y = -0.6
    goal_pose.position.z = 0.25

    rospy.loginfo("move_ptp to the starting point of the triangle: %s",
                  goal_pose)
    move_ptp_service.call(goal_pose)  # (error check omitted)

    time.sleep(1)

    # move up, its a right angled triangle
    goal_pose.position.z = .7

    # this time we move on a linear trajectory to the specified pose
    rospy.loginfo("move_lin to the tip of the triangle: %s", goal_pose)
    move_lin_service.call(goal_pose, True)  # (error check omitted)

    time.sleep(1)

    # next point is the upper right point of the triangle
    goal_pose.position.y = -0.1
    rospy.loginfo("move_lin to the upper right point of the triangle: %s",
                  goal_pose)
    move_lin_service.call(goal_pose, True)  # (error check omitted)

    # close the triangle by moving back diagonally to the start position
    goal_pose.position.y = -0.6
    goal_pose.position.z = .25
    rospy.loginfo("move_lin to the start to close the triangle shape: %s",
                  goal_pose)
    move_lin_service.call(goal_pose, True)  # (error check omitted)

    time.sleep(1)

    # note: move_lin is not always successful, even if move_ptp succeeds.
    # This is because moving on a linear trajectory is more constraining
    # Example: move to a positive y-position will fail with move_lin
    goal_pose.position.y = 0.3
    rospy.loginfo("try to move_lin to: %s", goal_pose)
    resp = move_lin_service.call(goal_pose, True)

    if not resp.success:
        rospy.logerr("move_lin service call failed (as expected)")

    # calling move_ptp with the exact same goal pose succeeds
    # TODO(uieai): WHY does it fail?
    rospy.loginfo("try to move_ptp to: %s", goal_pose)
    resp = move_ptp_service.call(goal_pose)

    if resp.success:
        rospy.loginfo("move_ptp service call succeeded")
    else:
        rospy.logerr("move_ptp service call failed")

    time.sleep(2)

    # the Response sometimes holds more information than only success
    rospy.loginfo("move_random to a new random position")
    resp = move_random_service.call()  # (error check omitted)

    # we can obtain the chosen random pose from the response
    rospy.loginfo("move_random moved to: %s", resp.pose)

    time.sleep(2)


def action_callback(req):
    rospy.loginfo("Action invoked.")
    # run the actual movement code
    hello_world()
    rospy.loginfo("Action completed!")
    server.set_succeeded(True)


# When the 'move_client' action is called run the 'action_callback'
server = actionlib.SimpleActionServer("move_client", DefaultMoveIfaceAction,
                                      action_callback, False)

if __name__ == '__main__':
    # Initialize the ROS node, start the action server and wait
    rospy.init_node('hello_world')
    rospy.loginfo("Action server started")
    server.start()
    rospy.spin()
