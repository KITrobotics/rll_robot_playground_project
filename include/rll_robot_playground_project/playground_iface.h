/*
 * This file is part of the Robot Learning Lab Robot Playground project
 *
 * Copyright (C) 2019 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#ifndef RLL_ROBOT_PLAYGROUND_PROJECT_PLAYGROUND_IFACE_H
#define RLL_ROBOT_PLAYGROUND_PROJECT_PLAYGROUND_IFACE_H

#include <rll_move/move_iface_base.h>

class PlaygroundMoveIfaceBase : public RLLMoveIfaceBase
{
public:
  explicit PlaygroundMoveIfaceBase(const ros::NodeHandle& nh) : RLLMoveIfaceBase(nh)
  {
  }
  void startServicesAndRunNode(ros::NodeHandle* nh) override;
};

#endif  // RLL_ROBOT_PLAYGROUND_PROJECT_PLAYGROUND_IFACE_H
