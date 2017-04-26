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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behavior_tree_core/BTAction.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_action");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<behavior_tree_core::BTAction> ac("action", true);
    behavior_tree_core::BTResult node_result;
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer();  // will wait for infinite time

    behavior_tree_core::BTGoal goal;


    int command = 0;
    bool isRunning = false;


    while (command != 3)
    {
        ROS_INFO("Send a command: 1:start the action | 2:stop the action | 3:exit the program");
        std::cin >> command;

        switch (command)
        {
        case 1:
            if (!isRunning)
            {
                ROS_INFO("I am running the request");
                ac.sendGoal(goal);
                isRunning = true;
                node_result = *(ac.getResult());
                ROS_INFO("Action finished, status: %d", node_result.status);
            }
            else
            {
                ROS_INFO("I am re-running the request");
                ac.cancelGoal();
                ac.sendGoal(goal);
                ROS_INFO("Action finished, status: %d", node_result.status);
            }
            break;
        case 2:
            ROS_INFO("I am cancelling the request");
            ac.cancelGoal();
            isRunning = false;
            break;
        default:
            break;
        }
    }
    return 0;
}

