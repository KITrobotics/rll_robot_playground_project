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

#include <rll_move/move_iface_simulation.h>
#include <rll_robot_playground_project/playground_iface.h>

using PlaygroundIface = RLLCombinedMoveIface<PlaygroundMoveIfaceBase, RLLSimulationMoveIface>;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "playground_iface");
  ros::NodeHandle nh;

  if (waitForMoveGroupAction())
  {
    PlaygroundIface iface(nh);
    iface.startServicesAndRunNode(&nh);
  }

  return 0;
}
