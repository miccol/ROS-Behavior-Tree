#include <ControlNode.h>


BT::ControlNode::ControlNode(std::string name) : TreeNode::TreeNode(name)
{
    type_ = BT::CONDITION_NODE;
}

BT::ControlNode::~ControlNode() {}

void BT::ControlNode::AddChild(TreeNode* Child)
{
    // Checking if the Child is not already present
    for (unsigned int i=0; i<ChildNodes.size(); i++)
    {
        if (ChildNodes[i] == Child)
        {
            throw BehaviorTreeException("'" + Child->get_name() + "' is already a '" + get_name() + "' child.");
        }
    }

    ChildNodes.push_back(Child);
    ChildStates.push_back(BT::IDLE);
}

unsigned int BT::ControlNode::GetChildrenNumber()
{
    return ChildNodes.size();
}

bool BT::ControlNode::Halt()
{
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(state_mutex_);

    state_ = BT::HALTED;
    return true;
}

bool BT::ControlNode::WriteState(NodeState StateToBeSet)
{
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(state_mutex_);

    state_ = StateToBeSet;
    return true;
}


std::vector<BT::TreeNode*> BT::ControlNode::GetChildren()
{
    return ChildNodes;
}


void BT::ControlNode::ResetColorState()
{

    SetColorState(BT::IDLE);
    for(unsigned int i = 0; i < ChildNodes.size(); i++)
    {
        ChildNodes[i]->ResetColorState();
    }
}

void BT::ControlNode::HaltChildren(int i){
    for(unsigned int j=i; j<ChildNodes.size(); j++)
    {
        if (ChildNodes[j]->get_type() != BT::ACTION_NODE && ChildStates[j] == BT::RUNNING)
        {
            // if the control node was running:
            // halting it;
            ChildNodes[j]->Halt();

            // sync with it (it's waiting on the semaphore);
            ChildNodes[j]->tick_engine.tick();

            std::cout << get_name() << " halting child number " << j << "!" << std::endl;
        }
        else if (ChildNodes[j]->get_type() == BT::ACTION_NODE && ChildNodes[j]->ReadState() == BT::RUNNING)
        {
            std::cout << get_name() << " trying halting child number " << j << "..." << std::endl;

            // if it's a action node that hasn't finished its job:
            // trying to halt it:
            if (ChildNodes[j]->Halt() == false)
            {
                // this means that, before this node could set its child state
                // to "Halted", the child had already written the action outcome;
                // sync with him ignoring its state;
                ChildNodes[j]->tick_engine.tick();

                std::cout << get_name() << " halting of child number " << j << " failed!" << std::endl;
            }

            std::cout << get_name() << " halting of child number " << j << " succedeed!" << std::endl;
        }
        else if (ChildNodes[j]->get_type() == BT::ACTION_NODE && ChildNodes[j]->ReadState() != BT::IDLE)
        {
            // if it's a action node that has finished its job:
            // ticking it without saving its returning state;
            ChildNodes[j]->tick_engine.tick();
        }

        // updating its vector cell
        ChildStates[j] = BT::IDLE;
    }

 }


int BT::ControlNode::Depth()
{
        int depMax = 0;
        int dep = 0;
        for (int i = 0; i < ChildNodes.size(); i++)
        {
            dep = (ChildNodes[i]->Depth());
           if (dep > depMax)
           {
               depMax = dep;
           }

        }
      return 1 + depMax;
}

