#ifndef ACTIONTEST_H
#define ACTIONTEST_H

#include <action_node.h>

namespace BT
{
    class ActionTestNode : public ActionNode
    {

    public:

        // Constructor
        ActionTestNode(std::string Name);
        ~ActionTestNode();

        // The method that is going to be executed by the thread
        void Exec2();
        BT::NodeState Exec();
        void set_status(NodeState status);
        void set_time(int time);
	
        // The method used to interrupt the execution of the node
        bool Halt();
    private:
        int time_;
    	NodeState status_;

    };
}

#endif
