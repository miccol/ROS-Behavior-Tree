#include <ActionNode.h>



BT::ActionNode::ActionNode(std::string name) : LeafNode::LeafNode(name)
{
    Type = Action;
}

BT::ActionNode::~ActionNode() {}

bool BT::ActionNode::WriteState(NodeState StateToBeSet)
{

    if(StateToBeSet != Idle)
    {
        SetColorState(StateToBeSet);
    }
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(StateMutex);

    // Check for spourios "Halted"
    if (State == Halted && StateToBeSet != Idle && StateToBeSet != Exit)
    {
        return false;
    }

    State = StateToBeSet;
    return true;
}

int BT::ActionNode::GetType()
{
    // Lock acquistion

    return BT::ACTION;
}
