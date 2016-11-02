#ifndef SELECTORNODE_H
#define SELECTORNODE_H

#include <control_node.h>

namespace BT
{
    class SelectorNode : public ControlNode
    {
    public:
        // Constructor
        SelectorNode(std::string name);
        ~SelectorNode();
    int DrawType();
        // The method that is going to be executed by the thread
        void Exec();
    };
}

#endif
