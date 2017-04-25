/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H





#include <draw.h>

#include <parallel_node.h>
#include <fallback_node.h>
#include <sequence_node.h>

#include <sequence_node_with_memory.h>
#include <fallback_node_with_memory.h>


#include <actions/action_test_node.h>
#include <conditions/condition_test_node.h>
#include <actions/ros_action.h>
#include <conditions/ros_condition.h>


#include <exceptions.h>

#include <string>
#include <map>

#include <typeinfo>
#include <math.h>       /* pow */

#include "ros/ros.h"
#include "std_msgs/UInt8.h"

void Execute(BT::ControlNode* root, int TickPeriod_milliseconds);


#endif  // BEHAVIOR_TREE_H
