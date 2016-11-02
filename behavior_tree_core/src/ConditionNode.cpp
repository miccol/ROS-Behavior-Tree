#include <ConditionNode.h>


BT::ConditionNode::ConditionNode(std::string name) : LeafNode::LeafNode(name)
{
    type_ = BT::CONDITION_NODE;
}

BT::ConditionNode::~ConditionNode() {}

bool BT::ConditionNode::Halt() { return true;}

bool BT::ConditionNode::WriteState(NodeState StateToBeSet)
{

    if(StateToBeSet != BT::IDLE)
    {
        SetColorState(StateToBeSet);
    }
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(state_mutex_);

    state_ = StateToBeSet;
    return true;
}
int BT::ConditionNode::DrawType()
{
    // Lock acquistion

    return BT::CONDITION;
}
