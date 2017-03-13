#include "parallel_node.h"


BT::ParallelNode::ParallelNode(std::string name, int threshold_M) : ControlNode::ControlNode(name)
{
threshold_M_ = threshold_M;
}

BT::ParallelNode::~ParallelNode() {}

BT::ReturnStatus BT::ParallelNode::Tick()
{

    unsigned int success_childred_num;
    unsigned int failed_childred_num;

    // Vector size initialization. N_of_children_ could change at runtime if you edit the tree
    N_of_children_ = children_nodes_.size();

    // Routing the tree according to the sequence node's logic:
    for (unsigned int i = 0; i < N_of_children_; i++)
    {
        if (children_nodes_[i]->get_type() == BT::ACTION_NODE)
        {
            //1) If the child i is an action, read its state.
            //Action nodes runs in another parallel, hence you cannot retrieve the status just by executing it.
            child_i_status_ = children_nodes_[i]->get_status();
        }
        else
        {
            child_i_status_ = children_nodes_[i]->Tick();

        }
        if (child_i_status_ != BT::RUNNING)
        {
            //1.1 If the action status is not running, the sequence node sends a tick to it.
            DEBUG_STDOUT(get_name() << "NEEDS TO TICK " << children_nodes_[i]->get_name());
            children_nodes_[i]->tick_engine.Tick();

            //waits for the tick to arrive to the child
            do
            {
                child_i_status_ = children_nodes_[i]->get_status();
                boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            }
            while(child_i_status_ != BT::RUNNING && child_i_status_ != BT::SUCCESS && child_i_status_ != BT::FAILURE);
        }

        switch(child_i_status_)
        {
        case BT::SUCCESS:
            if(++success_childred_num >= threshold_M_)
            {
                set_status(child_i_status_);
                success_childred_num = 0;
                failure_childred_num = 0;
                return child_i_status_;
            }
            break;
        case BT::FAILURE:
            if(++failed_childred_num > N_of_children_- threshold_M_)
            {
                set_status(child_i_status_);
                success_childred_num = 0;
                failure_childred_num = 0;
                return child_i_status_;
            }
            break;
        case BT::RUNNING:
            set_status(child_i_status_);
            return child_i_status_;
            break;

        }
    }
}

void BT::ParallelNode::Halt()
{
    success_childred_num = 0;
    failure_childred_num = 0;
    BT::ControlNode::Halt();
}

int BT::ParallelNode::DrawType()
{
    return BT::PARALLEL;
}

