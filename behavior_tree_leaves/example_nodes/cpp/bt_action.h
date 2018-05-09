/*
 * Abstract class used to create different BT actions.
 * Usage: Pass the action's name to this class when constructing your action.
 * Nodehandles and other variables should be part of your implementation.
 * Maintainer: Anas Abou Allaban
 */

#ifndef BTACTION_CLASS
#define BTACTION_CLASS

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_core/BTAction.h>

#include <functional>   // std::bind

enum Status {RUNNING, SUCCESS, FAILURE};  // BT return statuses

class BTAction
{
protected:
    // Create action server
    actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
    std::string action_name_;
    behavior_tree_core::BTFeedback feedback_;  // Action feedback (SUCCESS, FAILURE)
    behavior_tree_core::BTResult result_;      // Action feedback (feedback for us)

public:
    BTAction(std::string name);
    ~BTAction(void) {}
    virtual void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal) = 0;
    void set_status(int status);    // Returns the status to the client (Behavior Tree)

};

BTAction::BTAction(std::string name):
        as_(name, std::bind(&BTAction::execute_callback, this, _1), false),
        action_name_(name)
{
    as_.start(); // Starts the action server
}

void BTAction::set_status(int status)
{
    // Set The feedback and result of BT.action
    feedback_.status = status;
    result_.status = feedback_.status;
    as_.publishFeedback(feedback_); // Publish feedback
    // setSucceeded means that it has finished the action (it has returned SUCCESS or FAILURE).
    as_.setSucceeded(result_);

    switch (status)  // Print for convenience
    {
        case SUCCESS:
            ROS_INFO("%s Succeeded", ros::this_node::getName().c_str() );
            break;
        case FAILURE:
            ROS_INFO("%s Failed", ros::this_node::getName().c_str() );
            break;
        default:
            break;
    }
}

#endif  // BTACTION_CLASS
