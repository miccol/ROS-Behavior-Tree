#include <control_node.h>


BT::ControlNode::ControlNode(std::string name) : TreeNode::TreeNode(name)
{
    type_ = BT::CONDITION_NODE;
}

BT::ControlNode::~ControlNode() {}

void BT::ControlNode::AddChild(TreeNode* child)
{
    // Checking if the child is not already present
    for (unsigned int i=0; i<children_nodes_.size(); i++)
    {
        if (children_nodes_[i] == child)
        {
            throw BehaviorTreeException("'" + child->get_name() + "' is already a '" + get_name() + "' child.");
        }
    }

    children_nodes_.push_back(child);
    children_states_.push_back(BT::IDLE);
}

unsigned int BT::ControlNode::GetChildrenNumber()
{
    return children_nodes_.size();
}

bool BT::ControlNode::Halt()
{
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(state_mutex_);

    state_ = BT::HALTED;
    return true;
}

bool BT::ControlNode::WriteState(NodeState new_state)
{
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(state_mutex_);

    state_ = new_state;
    return true;
}


std::vector<BT::TreeNode*> BT::ControlNode::GetChildren()
{
    return children_nodes_;
}


void BT::ControlNode::ResetColorState()
{

    SetColorState(BT::IDLE);
    for(unsigned int i = 0; i < children_nodes_.size(); i++)
    {
        children_nodes_[i]->ResetColorState();
    }
}

void BT::ControlNode::HaltChildren(int i){
    for(unsigned int j=i; j<children_nodes_.size(); j++)
    {
        if (children_nodes_[j]->get_type() != BT::ACTION_NODE && children_states_[j] == BT::RUNNING)
        {
            // if the control node was running:
            // halting it;
            children_nodes_[j]->Halt();

            // sync with it (it's waiting on the semaphore);
            children_nodes_[j]->tick_engine.tick();

            std::cout << get_name() << " halting child number " << j << "!" << std::endl;
        }
        else if (children_nodes_[j]->get_type() == BT::ACTION_NODE && children_nodes_[j]->ReadState() == BT::RUNNING)
        {
            std::cout << get_name() << " trying halting child number " << j << "..." << std::endl;

            // if it's a action node that hasn't finished its job:
            // trying to halt it:
            if (children_nodes_[j]->Halt() == false)
            {
                // this means that, before this node could set its child state
                // to "Halted", the child had already written the action outcome;
                // sync with him ignoring its state;
                children_nodes_[j]->tick_engine.tick();

                std::cout << get_name() << " halting of child number " << j << " failed!" << std::endl;
            }

            std::cout << get_name() << " halting of child number " << j << " succedeed!" << std::endl;
        }
        else if (children_nodes_[j]->get_type() == BT::ACTION_NODE && children_nodes_[j]->ReadState() != BT::IDLE)
        {
            // if it's a action node that has finished its job:
            // ticking it without saving its returning state;
            children_nodes_[j]->tick_engine.tick();
        }

        // updating its vector cell
        children_states_[j] = BT::IDLE;
    }

 }


int BT::ControlNode::Depth()
{
        int depMax = 0;
        int dep = 0;
        for (int i = 0; i < children_nodes_.size(); i++)
        {
            dep = (children_nodes_[i]->Depth());
           if (dep > depMax)
           {
               depMax = dep;
           }

        }
      return 1 + depMax;
}

