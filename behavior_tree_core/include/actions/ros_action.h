#ifndef ROSACTION_H
#define ROSACTION_H

#include <action_node.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behavior_tree_core/BTAction.h>
namespace BT
{
class ROSAction : public ActionNode
{
protected:
    actionlib::SimpleActionClient<behavior_tree_core::BTAction> action_client_;
    behavior_tree_core::BTResult node_result;
    behavior_tree_core::BTGoal goal;
public:
    // Constructor
    ROSAction(std::string name);
    ~ROSAction();

    // The method that is going to be executed by the thread
    void WaitForTick();

    // The method used to interrupt the execution of the node
    void Halt();


};
}

#endif
