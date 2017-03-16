#include <tree_node.h>

BT::TreeNode::TreeNode(std::string name) : tick_engine(0)
{
    // Initialization
    name_ = name;
    is_state_updated_ = false;
    set_status(BT::IDLE);
}

BT::TreeNode::~TreeNode() {}

void BT::TreeNode::set_status(ReturnStatus new_status)
{

    if(new_status != BT::IDLE)
    {
        SetColorState(new_status);
    }

    // Lock acquistion
    std::unique_lock<std::mutex> UniqueLock(state_mutex_);

    // state_ update
    status_ = new_status;
}

BT::ReturnStatus BT::TreeNode::get_status()
{
    // Lock acquistion
    DEBUG_STDOUT(get_name() << " is setting its status to " << status_);

    std::lock_guard<std::mutex> LockGuard(state_mutex_);

    return status_;
}


BT::ReturnStatus BT::TreeNode::ReadColorState()
{
    // Lock acquistion

    return color_state_;
}

void BT::TreeNode::SetColorState(ReturnStatus ColorStateToBeSet)
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



void BT::TreeNode::set_name(std::string new_name)
{

name_ = new_name;

}


std::string BT::TreeNode::get_name()
{

return name_;

}


BT::NodeType BT::TreeNode::get_type()
{

return type_;

}

