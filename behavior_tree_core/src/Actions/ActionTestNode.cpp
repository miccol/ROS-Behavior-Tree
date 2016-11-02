#include <Actions/ActionTestNode.h>

BT::ActionTestNode::ActionTestNode(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::Action;
    // Thread start
    Thread = boost::thread(&ActionTestNode::Exec, this);
}

BT::ActionTestNode::~ActionTestNode() {}

void BT::ActionTestNode::Exec()
{



    while(true)
    {

        // Waiting for a tick to come
        Semaphore.Wait();

        if(ReadState() == Exit)
        {    //SetColorState(Idle);

            // The behavior tree is going to be destroied
            return;
        }

        // Running state
        SetNodeState(BT::Running);
        std::cout << get_name() << " returning " << BT::Running << "!" << std::endl;

        // Perform action...
        int i = 0;
        while(ReadState() == BT::Running and i++<5)
        {
            std::cout << get_name() << " working!" << std::endl;
            boost::this_thread::sleep(boost::posix_time::milliseconds(800));
        }

        if(ReadState() == BT::Exit)
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
                WriteState(Idle);

                // Next loop
                continue;
            }

            std::cout << get_name() << " returning " << BT::Success << "!" << std::endl;
        }

        // Synchronization
        // (my father is telling me that it has read my new state)
        Semaphore.Wait();

        if(ReadState() == BT::Exit)
        {

            // The behavior tree is going to be destroied
            return;
        }

        // Resetting the state
        WriteState(BT::Idle);
    }
}

bool BT::ActionTestNode::Halt()
{
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(StateMutex);

    // Checking for "Running" correctness
    if (State != BT::Running)
    {
        return false;
    }
    //SetColorState(Idle);

    State = Halted;
    return true;
}


void BT::ActionTestNode::SetBehavior(NodeState status){
    status_ = status;
}
