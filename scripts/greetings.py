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
import actionlib
from rll_msgs.srv import *
from rll_msgs.msg import *

def greetings():
    joint_1 = 0.6
    joint_2 = 1.832
    joint_3 = -0.3
    joint_4 = 2.01
    joint_5 = -2
    joint_6 = 0.0
    joint_7 = 1.4835

    move_joints = rospy.ServiceProxy('move_joints', MoveJoints)
    resp = move_joints(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7)
    if resp.success == False:
        rospy.logerr("moving to first joint pos failed")
        return False

    joint_3 = 0.3
    joint_5 = -0.7
    resp = move_joints(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7)
    if resp.success == False:
        rospy.logerr("moving to second joint pos failed")
        return False


    rospy.loginfo("one greetings movement finished")

def greet_full():
    for x in range(0, 5):
        greetings()

class MoveClient:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("move_client", DefaultMoveIfaceAction, self.execute, False)
        self.server.start()

    def execute(self, req):
        greet_full()
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('tower_hanoi')

    server = MoveClient()

    rospy.spin()
