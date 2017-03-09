#include <fallback_node.h>


BT::FallbackNode::FallbackNode(std::string name) : ControlNode::ControlNode(name)
{

}

BT::FallbackNode::~FallbackNode() {}

BT::ReturnStatus BT::FallbackNode::Tick()
{
    unsigned int i;
    // Vector size initialization. N_of_children_ could change at runtime if you edit the tree
    N_of_children_ = children_nodes_.size();

    // Routing the tree according to the sequence node's logic:
    for (i = 0; i < N_of_children_; i++)
    {
        if (children_nodes_[i]->get_type() == BT::ACTION_NODE)
        {
            //1) If the child i is an action, read its state.
            //Action nodes runs in another parallel, hence you cannot retrieve the status just by executing it.

            child_i_status_ = children_nodes_[i]->get_status();

            if (child_i_status_ != BT::RUNNING)
            {
                //1.1 If the action status is not running, the sequence node sends a tick to it.
                DEBUG_STDOUT(get_name() << " NEEDS TO TICK " << children_nodes_[i]->get_name());
                children_nodes_[i]->tick_engine.Tick();

                //waits for the tick to arrive to the child
                do
                {
                    child_i_status_ = children_nodes_[i]->get_status();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                }
                while(child_i_status_ != BT::RUNNING && child_i_status_ != BT::SUCCESS && child_i_status_ != BT::FAILURE);

                if(child_i_status_ == BT::RUNNING || child_i_status_ == BT::SUCCESS)
                {
                    //the sequence node's status is equal to ActionState if this is running or failure

                    return child_i_status_;
                }
            }
            else
            {
                //1.2 if the action is running already, let the action run and return success to the parent node
                return BT::RUNNING;
            }

            return BT::EXIT;
        }
        else
        {
            // 2 if it's not an action:
            // Send the tick and wait for the response;
           child_i_status_ = children_nodes_[i]->Tick();
        }

        if(child_i_status_ != BT::FAILURE)
        {
        // 2.1 -  If the  non-action status is not success, halt the nest children
            DEBUG_STDOUT(get_name() << " is HALTING children from " << (i+1));
            HaltChildren(i+1);
            return child_i_status_;
        }
        // 2.2 -  If the  non-action status is success, continue to the next child in the for loop (if any).
        if(i == N_of_children_ - 1)
        {
             return child_i_status_;
        }
    }
}


int BT::FallbackNode::DrawType()
{
    // Lock acquistion

    return BT::SELECTOR;
}
