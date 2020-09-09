#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab Move Client
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

import rospy
import rosunit

from rll_move_client.tests_demo_launcher import create_test_class


def main():
    rospy.init_node("playground_demo")

    to_import = [("rll_robot_playground_project", "scripts/"),
                 ("rll_move_client", "src")]
    klass = create_test_class(to_import, "playground", "hello_world",
                              "rll_move_client.client", "RLLDefaultMoveClient")

    rosunit.unitrun("rll_robot_playground_project", "playground_demo", klass)


if __name__ == "__main__":
    main()
