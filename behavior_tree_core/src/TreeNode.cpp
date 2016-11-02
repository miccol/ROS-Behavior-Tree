#include <TreeNode.h>

BT::TreeNode::TreeNode(std::string name) : tick_engine(0)
{
    // Initialization
    name_ = name;
    is_state_updated_ = false;
    state_ = BT::IDLE;
}

BT::TreeNode::~TreeNode() {}

BT::NodeState BT::TreeNode::GetNodeState()
{
    NodeState ReadState;
    // Lock acquistion
    boost::unique_lock<boost::mutex> UniqueLock(state_mutex_);

    // Wait until the state is updated by the node thread
    while(is_state_updated_ == false)
        state_condition_variable_.wait(UniqueLock);

    // Reset the is_state_updated_ flag
    is_state_updated_ = false;

    // state_ save
    ReadState = state_;

    // Releasing the node thread;
    state_condition_variable_.notify_all();

    // Take the state and unlock the mutex
    return ReadState;
}

void BT::TreeNode::SetNodeState(NodeState new_state)
{

    if(new_state != BT::IDLE)
    {
        SetColorState(new_state);
    }

    // Lock acquistion
    boost::unique_lock<boost::mutex> UniqueLock(state_mutex_);

    // state_ update
    state_ = new_state;
    is_state_updated_ = true;

    // Notification and unlock of the mutex
    state_condition_variable_.notify_all();

    // Waiting for the father to read the state
    state_condition_variable_.wait(UniqueLock);
}

BT::NodeState BT::TreeNode::ReadState()
{
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(state_mutex_);

    return state_;
}


BT::NodeState BT::TreeNode::ReadColorState()
{
    // Lock acquistion

    return color_state_;
}

void BT::TreeNode::SetColorState(NodeState ColorStateToBeSet)
{
    // Lock acquistion

    // state_ update
    color_state_ = ColorStateToBeSet;
}


float BT::TreeNode::get_x_pose()
{

return x_pose_;
}


void BT::TreeNode::set_x_pose(float x_pose)
{

x_pose_ = x_pose;
}



float BT::TreeNode::get_x_shift()
{

return x_shift_;
}


void BT::TreeNode::set_x_shift(float x_shift)
{

x_shift_ = x_shift;
}



std::string BT::TreeNode::get_name()
{

return name_;

}



BT::NodeType BT::TreeNode::get_type()
{

return type_;

}

