#include <sequence_node.h>


BT::SequenceNode::SequenceNode(std::string name) : ControlNode::ControlNode(name)
{

}

BT::SequenceNode::~SequenceNode() {}

BT::NodeState BT::SequenceNode::Tick()
{

    unsigned int i;
    NodeState ChildState;
    // Vector size initialization. N_of_children_ could change at runtime if you edit the tree
    N_of_children_ = children_nodes_.size();

    // Routing the tree according to the sequence node's logic:
    for (i = 0; i < N_of_children_; i++)
    {
        if (children_nodes_[i]->get_type() == BT::ACTION_NODE)
        {
            //1) If the child i is an action, read its state.
            //Action nodes runs in another parallel, hence you cannot retrieve the status just by executing it.

            ChildState = children_nodes_[i]->get_status();

            if (ChildState != BT::RUNNING)
            {
                //1.1 If the action status is not running, the sequence node sends a tick to it.
                DEBUG_STDOUT("NEED TO TICK " << children_nodes_[i]->get_name());
                children_nodes_[i]->tick_engine.Tick();

                //waits for the tick to arrive to the child
                do
                {
                    ChildState = children_nodes_[i]->get_status();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                }
                while(ChildState != BT::RUNNING && ChildState != BT::SUCCESS && ChildState != BT::FAILURE);

                if(ChildState == BT::RUNNING || ChildState == BT::FAILURE)
                {
                    //the sequence node's status is equal to ActionState if this is running or failure

                    return ChildState;
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

            std::cout << " NOT AN ACTION !" << std::endl;

            // Send the tick and wait for the response;
           ChildState = children_nodes_[i]->Tick();
        }

        if(ChildState != BT::SUCCESS)
        {
        // 2.1 -  If the  non-action status is not success, halt the nest children
            DEBUG_STDOUT("Halting other children");
            HaltChildren(i+1);
            return ChildState;
        }
        // 2.2 -  If the  non-action status is success, continue to the next child in the for loop.
    }
}


int BT::SequenceNode::DrawType()
{
    // Lock acquistion

    return BT::SEQUENCE;
}

