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


#include <conditions/ros_condition.h>
#include <string>

enum Status {RUNNING, SUCCESS, FAILURE};


BT::ROSCondition::ROSCondition(std::string name) :
  ConditionNode::ConditionNode(name),
  action_client_(name, true)
{
    ROS_INFO("Waiting For the Acutator named %s to start", get_name().c_str());
    action_client_.waitForServer();  // will wait for infinite time until the server starts
    ROS_INFO("Actuator %s Started", get_name().c_str());
}

BT::ROSCondition::~ROSCondition() {}

BT::ReturnStatus BT::ROSCondition::Tick()
{
    ROS_INFO("I am running the request");

    // Condition checking and state update
    action_client_.sendGoal(goal);
    action_client_.waitForResult(ros::Duration(30.0));
    node_result = *(action_client_.getResult());
    set_status((ReturnStatus)node_result.status);
    return (ReturnStatus)node_result.status;
}
