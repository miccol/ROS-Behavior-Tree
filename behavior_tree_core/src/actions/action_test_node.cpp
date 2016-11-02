#include <actions/action_test_node.h>

BT::ActionTestNode::ActionTestNode(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    // thread_ start
    thread_ = boost::thread(&ActionTestNode::Exec, this);
}

BT::ActionTestNode::~ActionTestNode() {}

void BT::ActionTestNode::Exec()
{



    while(true)
    {

        // Waiting for a tick to come
        tick_engine.wait();

        if(ReadState() == BT::EXIT)
        {    //SetColorState(Idle);

            // The behavior tree is going to be destroied
            return;
        }

        // Running state
        SetNodeState(BT::RUNNING);
        std::cout << get_name() << " returning " << BT::RUNNING << "!" << std::endl;

        // Perform action...
        int i = 0;
        while(ReadState() == BT::RUNNING and i++<5)
        {
            std::cout << get_name() << " working!" << std::endl;
            boost::this_thread::sleep(boost::posix_time::milliseconds(800));
        }

        if(ReadState() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return;
        }


        else
        {
            // trying to set the outcome state:
            if (WriteState(status_) != true)
            {
                // meanwhile, my father halted me!
                std::cout << get_name() << " Halted!" << std::endl;

                // Resetting the state
                WriteState(BT::IDLE);

                // Next loop
                continue;
            }

            std::cout << get_name() << " returning " << BT::SUCCESS << "!" << std::endl;
        }

        // Synchronization
        // (my father is telling me that it has read my new state)
        tick_engine.wait();

        if(ReadState() == BT::EXIT)
        {

            // The behavior tree is going to be destroied
            return;
        }

        // Resetting the state
        WriteState(BT::IDLE);
    }
}

bool BT::ActionTestNode::Halt()
{
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(state_mutex_);

    // Checking for "Running" correctness
    if (state_ != BT::RUNNING)
    {
        return false;
    }
    //SetColorState(Idle);

    state_ = BT::HALTED;
    return true;
}


void BT::ActionTestNode::SetBehavior(NodeState status){
    status_ = status;
}
