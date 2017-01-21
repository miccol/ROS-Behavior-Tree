#include <actions/action_test_node.h>

BT::ActionTestNode::ActionTestNode(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    // thread_ start
    thread_ = boost::thread(&ActionTestNode::Exec2, this);
}

BT::ActionTestNode::~ActionTestNode() {}

BT::NodeState BT::ActionTestNode::Exec(){ return BT::EXIT;}


void BT::ActionTestNode::Exec2()
{


time_ = 100;
    while(true)
    {

        // Waiting for the first tick to come
        tick_engine.wait();
        DEBUG_STDOUT("TICK RECEIVED");
//        if(ReadState() == BT::EXIT)
//        {    //SetColorState(Idle);

//            // The behavior tree is going to be destroied
//            return;
//        }

        // Running state
        SetNodeState(BT::RUNNING);
        // Perform action...
        int i = 0;
        while(ReadState() == BT::RUNNING && i++<time_)
        {
            DEBUG_STDOUT(" Action running!");
            boost::this_thread::sleep(boost::posix_time::milliseconds(800));
        }

        DEBUG_STDOUT(" OUT !");

        if(ReadState() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return;
        }


        else
        {
            // trying to set the outcome state:
           // if (WriteState(status_) != true)
//            {
//                // meanwhile, my father halted me!
//                std::cout << get_name() << " Halted!" << std::endl;

//                // Resetting the state
//                //WriteState(BT::IDLE);
//                SetNodeState(BT::IDLE);

//                // Next loop
//                continue;
//            }

           // std::cout << get_name() << " returning " << BT::SUCCESS << "!" << std::endl;
        }

        // Synchronization
        // (my father is telling me that it has read my new state)
       // tick_engine.wait();

        if(ReadState() == BT::EXIT)
        {

            // The behavior tree is going to be destroied
            return;
        }

        // Resetting the state
       // WriteState(BT::IDLE);
    }
}

bool BT::ActionTestNode::Halt()
{
    // Lock acquistion
    DEBUG_STDOUT(" Waiting for Lock to Halt action!");

    //SetColorState(Idle);
    DEBUG_STDOUT(" Action Halted!");

    SetNodeState(BT::HALTED);
    DEBUG_STDOUT(" HALTED state set!");

    return true;
}


void BT::ActionTestNode::set_status(NodeState status){
    status_ = status;
}

void BT::ActionTestNode::set_time(int time){
    time_ = time;
}
