#ifndef BEHAVIORTREECORE_ACTIONNODE_H
#define BEHAVIORTREECORE_ACTIONNODE_H

#include "leaf_node.h"

namespace BT
{

    class ActionNode : public LeafNode
    {
    public:
        // Constructor
        ActionNode(std::string name);
        ~ActionNode();

        // The method that is going to be executed by the thread
        virtual void Exec2() = 0;
        virtual BT::NodeState Exec() = 0;


        // The method used to interrupt the execution of the node
        virtual bool Halt() = 0;

        // Methods used to access the node state without the
        // conditional waiting (only mutual access)
        bool WriteState(NodeState new_state);
    int DrawType();
    };
}

#endif
