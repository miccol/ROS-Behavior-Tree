#include <TreeNode.h>

BT::TreeNode::TreeNode(std::string Name) : Semaphore(0)
{
    // Initialization
    this->Name = Name;
    StateUpdated = false;
    State = BT::Idle;
}

BT::TreeNode::~TreeNode() {}

BT::NodeState BT::TreeNode::GetNodeState()
{
    NodeState ReadState;
    // Lock acquistion
    boost::unique_lock<boost::mutex> UniqueLock(StateMutex);

    // Wait until the state is updated by the node thread
    while(StateUpdated == false)
        StateConditionVariable.wait(UniqueLock);

    // Reset the StateUpdated flag
    StateUpdated = false;

    // State save
    ReadState = State;

    // Releasing the node thread;
    StateConditionVariable.notify_all();

    // Take the state and unlock the mutex
    return ReadState;
}

void BT::TreeNode::SetNodeState(NodeState StateToBeSet)
{

    if(StateToBeSet != BT::Idle)
    {
        SetColorState(StateToBeSet);
    }

    // Lock acquistion
    boost::unique_lock<boost::mutex> UniqueLock(StateMutex);

    // State update
    State = StateToBeSet;
    StateUpdated = true;

    // Notification and unlock of the mutex
    StateConditionVariable.notify_all();

    // Waiting for the father to read the state
    StateConditionVariable.wait(UniqueLock);
}

BT::NodeState BT::TreeNode::ReadState()
{
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(StateMutex);

    return State;
}


BT::NodeState BT::TreeNode::ReadColorState()
{
    // Lock acquistion

    return ColorState;
}

void BT::TreeNode::SetColorState(NodeState ColorStateToBeSet)
{
    // Lock acquistion

    // State update
    ColorState = ColorStateToBeSet;
}


float BT::TreeNode::GetXPose()
{

return x_pose_;
}


void BT::TreeNode::SetXPose(float x_pose)
{

x_pose_ = x_pose;
}



float BT::TreeNode::GetXShift()
{

return x_shift_;
}


void BT::TreeNode::SetXShift(float x_shift)
{

x_shift_ = x_shift;
}



std::string BT::TreeNode::get_name()
{

return Name;

}

