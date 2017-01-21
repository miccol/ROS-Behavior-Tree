#include <actions/action_test_node.h>

BT::ActionTestNode::ActionTestNode(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    // thread_ start
    thread_ = boost::thread(&ActionTestNode::WaitForTick, this);
}

BT::ActionTestNode::~ActionTestNode() {}

//BT::NodeState BT::ActionTestNode::Tick(){ return BT::EXIT;}


void BT::ActionTestNode::WaitForTick()
{


time_ = 100;
    while(true)
    {

        // Waiting for the first tick to come
        DEBUG_STDOUT("WAIT FOR TICK");

        tick_engine.Wait();
        DEBUG_STDOUT("TICK RECEIVED");


        // Running state
        set_status(BT::RUNNING);
        // Perform action...
        int i = 0;
        while(get_status() == BT::RUNNING && i++<time_)
        {
            DEBUG_STDOUT(" Action running!");
            boost::this_thread::sleep(boost::posix_time::milliseconds(800));
        }

        DEBUG_STDOUT(" OUT !");

    }
}

bool BT::ActionTestNode::Halt()
{
    // Lock acquistion
    DEBUG_STDOUT(" Waiting for Lock to Halt action!");

    //SetColorState(Idle);
    DEBUG_STDOUT(" Action Halted!");

    set_status(BT::HALTED);
    DEBUG_STDOUT(" HALTED state set!");

    return true;
}


//void BT::ActionTestNode::set_status(NodeState status){
//    status_ = status;
//}

void BT::ActionTestNode::set_time(int time){
    time_ = time;
}
