#include <action_node.h>



BT::ActionNode::ActionNode(std::string name) : LeafNode::LeafNode(name)
{
    type_ = BT::ACTION_NODE;
}

BT::ActionNode::~ActionNode() {}

bool BT::ActionNode::WriteState(NodeState new_state)
{

    if(new_state != BT::IDLE)
    {
        SetColorState(new_state);
    }
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(state_mutex_);

    // Check for spourios "Halted"
    if (state_ == BT::HALTED && new_state != BT::IDLE && new_state != BT::EXIT)
    {
        return false;
    }

    state_ = new_state;
    return true;
}

int BT::ActionNode::DrawType()
{
    // Lock acquistion

    return BT::ACTION;
}
