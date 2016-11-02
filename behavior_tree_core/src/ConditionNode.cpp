#include <ConditionNode.h>


BT::ConditionNode::ConditionNode(std::string name) : LeafNode::LeafNode(name)
{
    type_ = BT::CONDITION_NODE;
}

BT::ConditionNode::~ConditionNode() {}

bool BT::ConditionNode::Halt() { return true;}

bool BT::ConditionNode::WriteState(NodeState new_state)
{

    if(new_state != BT::IDLE)
    {
        SetColorState(new_state);
    }
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(state_mutex_);

    state_ = new_state;
    return true;
}
int BT::ConditionNode::DrawType()
{
    // Lock acquistion

    return BT::CONDITION;
}
