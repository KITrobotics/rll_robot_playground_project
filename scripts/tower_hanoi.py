#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab stack
#
# Copyright (C) 2018 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

# 1->3, 1->2, 3->2, 1->3, 2->1, 2->3, 1->3

import rospy
from rll_moveit_testing.srv import *
from geometry_msgs.msg import Pose

def play_tower_of_hanoi():
    pose_above = Pose()
    pose_grip = Pose()

    # above table: z: 0.12
    # 1->3
    pose_above.position.z = 0.2
    pose_above.position.x = 0.5
    pose_above.position.y = 0.0
    pose_above.orientation.y = 1.0
    pose_grip.position.z = 0.166
    pose_grip.position.x = 0.5
    pose_grip.position.y = 0.0
    pose_grip.orientation.y = 1.0

    pick_place = rospy.ServiceProxy('pick_place', PickPlace)
    resp = pick_place(pose_above, pose_grip, True)
    rospy.loginfo("pick place job finished")

    pose_above.position.y = 0.2
    pose_grip.position.z = 0.122
    pose_grip.position.y = 0.2
    resp = pick_place(pose_above, pose_grip, False)
    rospy.loginfo("pick place job finished")

    # 1->2
    pose_above.position.y = 0.0
    pose_grip.position.z = 0.144
    pose_grip.position.y = 0.0
    resp = pick_place(pose_above, pose_grip, True)
    rospy.loginfo("pick place job finished")

    pose_above.position.y = 0.1
    pose_grip.position.z = 0.122
    pose_grip.position.y = 0.1
    resp = pick_place(pose_above, pose_grip, False)
    rospy.loginfo("pick place job finished")

    # 3->2
    pose_above.position.y = 0.2
    pose_grip.position.z = 0.122
    pose_grip.position.y = 0.2
    resp = pick_place(pose_above, pose_grip, True)
    rospy.loginfo("pick place job finished")

    pose_above.position.y = 0.1
    pose_grip.position.z = 0.144
    pose_grip.position.y = 0.1
    resp = pick_place(pose_above, pose_grip, False)
    rospy.loginfo("pick place job finished")

    # 1->3
    pose_above.position.y = 0.0
    pose_grip.position.z = 0.122
    pose_grip.position.y = 0.0
    resp = pick_place(pose_above, pose_grip, True)
    rospy.loginfo("pick place job finished")

    pose_above.position.y = 0.2
    pose_grip.position.z = 0.122
    pose_grip.position.y = 0.2
    resp = pick_place(pose_above, pose_grip, False)
    rospy.loginfo("pick place job finished")

    # 2->1
    pose_above.position.y = 0.1
    pose_grip.position.z = 0.144
    pose_grip.position.y = 0.1
    resp = pick_place(pose_above, pose_grip, True)
    rospy.loginfo("pick place job finished")

    pose_above.position.y = 0.0
    pose_grip.position.z = 0.122
    pose_grip.position.y = 0.0
    resp = pick_place(pose_above, pose_grip, False)
    rospy.loginfo("pick place job finished")

    # 2->3
    pose_above.position.y = 0.1
    pose_grip.position.z = 0.122
    pose_grip.position.y = 0.1
    resp = pick_place(pose_above, pose_grip, True)
    rospy.loginfo("pick place job finished")

    pose_above.position.y = 0.2
    pose_grip.position.z = 0.144
    pose_grip.position.y = 0.2
    resp = pick_place(pose_above, pose_grip, False)
    rospy.loginfo("pick place job finished")

    # 1->3
    pose_above.position.y = 0.0
    pose_grip.position.z = 0.122
    pose_grip.position.y = 0.0
    resp = pick_place(pose_above, pose_grip, True)
    rospy.loginfo("pick place job finished")

    pose_above.position.y = 0.2
    pose_grip.position.z = 0.166
    pose_grip.position.y = 0.2
    resp = pick_place(pose_above, pose_grip, False)
    rospy.loginfo("pick place job finished")

    # reset
    pose_above.position.y = 0.2
    pose_grip.position.z = 0.122
    pose_grip.position.y = 0.2
    resp = pick_place(pose_above, pose_grip, True)
    rospy.loginfo("pick place job finished")

    pose_above.position.y = 0.0
    # pose_above.position.z = 0.15
    pose_grip.position.z = 0.122
    pose_grip.position.y = 0.0
    resp = pick_place(pose_above, pose_grip, False)
    rospy.loginfo("pick place job finished")

if __name__ == '__main__':
    rospy.init_node('tower_hanoi')
    play_tower_of_hanoi()
