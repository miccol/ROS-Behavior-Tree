#include "fallback_node_with_memory.h"

BT::FallbackNodeWithMemory::FallbackNodeWithMemory(std::string name) : ControlNode::ControlNode(name)
{
    reset_policy_ = BT::ON_SUCCESS_OR_FAILURE;
    current_child_idx_ = 0;//initialize the current running child

}


BT::FallbackNodeWithMemory::FallbackNodeWithMemory(std::string name, int reset_policy) : ControlNode::ControlNode(name)
{
    DEBUG_STDOUT("*****************************Poly**************************");

    reset_policy_ = reset_policy;
    current_child_idx_ = 0;//initialize the current running child

}


BT::FallbackNodeWithMemory::~FallbackNodeWithMemory() {}


BT::ReturnStatus BT::FallbackNodeWithMemory::Tick()
{

    DEBUG_STDOUT(get_name() << " ticked, memory counter: "<< current_child_idx_);

    // Vector size initialization. N_of_children_ could change at runtime if you edit the tree
    N_of_children_ = children_nodes_.size();

    // Routing the tree according to the sequence node's (with memory) logic:
    while (current_child_idx_ < N_of_children_)
    {
        if (children_nodes_[current_child_idx_]->get_type() == BT::ACTION_NODE )
        {
            //1) If the child i is an action, read its state.
            //Action nodes runs in another parallel, hence you cannot retrieve the status just by executing it.
           // DEBUG_STDOUT(get_name() << "It is an action " << children_nodes_[current_child_]->get_name());

            child_i_status_ = children_nodes_[current_child_idx_]->get_status();
            DEBUG_STDOUT(get_name() << " It is an action " << children_nodes_[current_child_idx_]->get_name() << " with status: " << child_i_status_);


            if (child_i_status_ != BT::RUNNING)
            {
                if(child_i_status_ == BT::IDLE)
                {
                    //1.1 If the action status is IDLE, the sequence node sends a tick to it.
                    DEBUG_STDOUT(get_name() << "NEEDS TO TICK " << children_nodes_[current_child_idx_]->get_name());
                    children_nodes_[current_child_idx_]->tick_engine.Tick();
                }
                //waits for the tick to arrive to the child
                do
                {
                    child_i_status_ = children_nodes_[current_child_idx_]->get_status();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                }
                while(child_i_status_ != BT::RUNNING && child_i_status_ != BT::SUCCESS && child_i_status_ != BT::FAILURE);



                if(child_i_status_ != BT::FAILURE)
                {
                // 1.2 -  If the  action status is not FAILURE, return the status
                    DEBUG_STDOUT("the status of: " << get_name() << " becomes " << child_i_status_);
                    if(child_i_status_ == BT::SUCCESS && (reset_policy_ == BT::ON_SUCCESS || reset_policy_ == BT::ON_SUCCESS_OR_FAILURE))
                    {
                    current_child_idx_ = 0;
                    }
                    set_status(child_i_status_);
                    return child_i_status_;
                }
                else if(current_child_idx_ != N_of_children_ - 1)
                {
                    // 1.3-  If the  non-action status is success, continue to the next child (if any, hence if(current_child_ != N_of_children_ - 1) ) in the for loop (if any).

                    std::cout << get_name() << " increasing the memory counter" << std::endl;

                    current_child_idx_++;

                }else if(current_child_idx_ == N_of_children_ - 1)
                {
                    if(child_i_status_ == BT::FAILURE)
                    {
                        //1.4 if it the last child and it has returned SUCCESS, reset the memory
                        current_child_idx_ = 0;
                    }
                    set_status(child_i_status_);
                    return child_i_status_;
                }




//                if(child_i_status_ == BT::RUNNING || child_i_status_ == BT::FAILURE)
//                {
//                    //the sequence node's status is equal to ActionState if this is running or failure

//                    return child_i_status_;
//                }
            }
            else
            {
                //1.2 if the action is running already, let the action run and return success to the parent node
                set_status(BT::RUNNING);
                return BT::RUNNING;
            }

            //return BT::EXIT;
        }
        else
        {
            // 2 if it's not an action:
            // Send the tick and wait for the response;
            child_i_status_ = children_nodes_[current_child_idx_]->Tick();


            if(child_i_status_ != BT::FAILURE)
            {
                // 2.1 -  If the  non-action status is not failure, return the status
                if(child_i_status_ == BT::SUCCESS && (reset_policy_ == BT::ON_SUCCESS || reset_policy_ == BT::ON_SUCCESS_OR_FAILURE))
                {
                current_child_idx_ = 0;
                }
                std::cout << get_name() << " Returning " << child_i_status_ << std::endl;

                set_status(child_i_status_);
                return child_i_status_;
            }
            else if(current_child_idx_ < N_of_children_ -1)
            {
                // 2.2 -  If the  non-action status is FAILURE, continue to the next child (if any, hence if(current_child_ != N_of_children_ - 1) ) in the for loop (if any).
                std::cout << get_name() << " increasing the memory counter" << std::endl;

                current_child_idx_ = current_child_idx_ +1;
                std::cout << get_name() << "NEW memory counter: " << current_child_idx_<< std::endl;

            }else if(current_child_idx_ == (N_of_children_ - 1))

            {
                if(child_i_status_ == BT::FAILURE)
                {
                    //3 if it the last child and it has returned FAILURE, reset the memory
                    std::cout << get_name() << " resetting the memory counter" << std::endl;
                    current_child_idx_ = 0;
                }
                set_status(child_i_status_);
                return child_i_status_;
            }
        }
    }
}


int BT::FallbackNodeWithMemory::DrawType()
{
    return BT::SELECTORSTAR;
}

void BT::FallbackNodeWithMemory::Halt()
{
    current_child_idx_ = 0;
    BT::ControlNode::Halt();
}


