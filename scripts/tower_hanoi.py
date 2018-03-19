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

import rospy
from moveit_testing_sender.srv import *
from geometry_msgs.msg import Pose


def hanoi(n, source, helper, target):
    print source, helper, target
    if n > 0:
        # move tower of size n - 1 to helper:
        hanoi(n - 1, source, target, helper)

        # move disk from source peg to target peg
        if source["disks"]:
            move_disk(source, target)

        # move tower of size n-1 from helper to target
        hanoi(n - 1, helper, source, target)

def move_disk(start, dest):
    pose_above = Pose()
    pose_grip = Pose()

    # pick disk up
    rospy.loginfo("start has %d disks", start["disks"])
    pose_above.position.z = 0.08
    pose_above.position.x = 0.5
    pose_above.position.y = stack_pos[start["pos"]]
    pose_above.orientation.y = 1.0
    pose_grip.position.z = heights[start["disks"]]
    pose_grip.position.x = 0.5
    pose_grip.position.y = stack_pos[start["pos"]]
    pose_grip.orientation.y = 1.0

    pick_place = rospy.ServiceProxy('pick_place', PickPlace)
    resp = pick_place(pose_above, pose_grip, True)
    rospy.loginfo("pick up job finished")
    start["disks"] -= 1

    # place disk
    rospy.loginfo("dest has %d disks", dest["disks"])
    pose_above.position.y = stack_pos[dest["pos"]]
    pose_grip.position.z = heights[dest["disks"]+1]
    pose_grip.position.y = stack_pos[dest["pos"]]
    resp = pick_place(pose_above, pose_grip, False)
    rospy.loginfo("place job finished")
    dest["disks"] += 1

def cleanup():
    pose_above = Pose()
    pose_grip = Pose()

    # pick tower up
    pose_above.position.z = 0.03
    pose_above.position.x = 0.5
    pose_above.position.y = stack_pos[target["pos"]]
    pose_above.orientation.y = 1.0
    pose_grip.position.z = heights[1]
    pose_grip.position.x = 0.5
    pose_grip.position.y = stack_pos[target["pos"]]
    pose_grip.orientation.y = 1.0

    pick_place = rospy.ServiceProxy('pick_place', PickPlace)
    resp = pick_place(pose_above, pose_grip, True)
    rospy.loginfo("pick up job finished")

    # place disk
    pose_above.position.y = stack_pos[source["pos"]]
    pose_grip.position.y = stack_pos[source["pos"]]
    pick_place = rospy.ServiceProxy('pick_place', PickPlace)
    resp = pick_place(pose_above, pose_grip, False)
    rospy.loginfo("place job finished")

    # move a little higher before homing
    pose_above.position.z = 0.1
    pose_grip.position.z = 0.1
    resp = pick_place(pose_above, pose_grip, False)
    rospy.loginfo("ready to home")


source = {"pos": 1, "disks": 3}
target = {"pos": 3, "disks": 0}
helper = {"pos": 2, "disks": 0}

heights = {3: 0.048, 2: 0.026, 1: 0.004, 0: 0.004}
stack_pos = {1: -0.2, 2: 0.0, 3: 0.2}

if __name__ == '__main__':
    rospy.init_node('tower_hanoi')

    hanoi(source["disks"], source, helper, target)
    cleanup()
