#include <ActionNode.h>



BT::ActionNode::ActionNode(std::string name) : LeafNode::LeafNode(name)
{
    type_ = BT::ACTION_NODE;
}

BT::ActionNode::~ActionNode() {}

bool BT::ActionNode::WriteState(NodeState StateToBeSet)
{

    if(StateToBeSet != BT::IDLE)
    {
        SetColorState(StateToBeSet);
    }
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(state_mutex_);

    // Check for spourios "Halted"
    if (state_ == BT::HALTED && StateToBeSet != BT::IDLE && StateToBeSet != BT::EXIT)
    {
        return false;
    }

    state_ = StateToBeSet;
    return true;
}

int BT::ActionNode::DrawType()
{
    // Lock acquistion

    return BT::ACTION;
}
