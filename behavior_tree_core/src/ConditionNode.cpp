#include <ConditionNode.h>


BT::ConditionNode::ConditionNode(std::string name) : LeafNode::LeafNode(name)
{
    type_ = Condition;
}

BT::ConditionNode::~ConditionNode() {}

bool BT::ConditionNode::Halt() { return true;}

bool BT::ConditionNode::WriteState(NodeState StateToBeSet)
{

    if(StateToBeSet != BT::Idle)
    {
        SetColorState(StateToBeSet);
    }
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(StateMutex);

    State = StateToBeSet;
    return true;
}
int BT::ConditionNode::GetType()
{
    // Lock acquistion

    return BT::CONDITION;
}
