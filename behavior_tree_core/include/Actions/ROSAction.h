#ifndef ROSACTION_H
#define ROSACTION_H

#include <ActionNode.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behavior_tree_core/BTAction.h>
namespace BT
{
    class ROSAction : public ActionNode
    {
    public:
        // Constructor
        ROSAction(std::string Name);
        ~ROSAction();

        // The method that is going to be executed by the thread
        void Exec();

        // The method used to interrupt the execution of the node
        bool Halt();

  	//actionlib::SimpleActionClient<bt_actions::BTAction> ac();

	  behavior_tree_core::BTResult node_result;
          behavior_tree_core::BTGoal goal;
    };
}

#endif
