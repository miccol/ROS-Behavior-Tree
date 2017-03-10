#include <actions/action_test_node.h>
#include <thread>

BT::ActionTestNode::ActionTestNode(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    // thread_ start
    boolean_value_ = true;

    thread_ = boost::thread(&ActionTestNode::WaitForTick, this);
}

BT::ActionTestNode::~ActionTestNode() {}

//BT::ReturnStatus BT::ActionTestNode::Tick(){ return BT::EXIT;}


void BT::ActionTestNode::WaitForTick()
{


time_ = 3;
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
        while(get_status() == BT::RUNNING && i++ < time_)
        {
            DEBUG_STDOUT(" Action " << get_name() << "running! Thread id:" << std::this_thread::get_id());
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
        if(get_status() != BT::RUNNING)
        {
            DEBUG_STDOUT(" STATUS of " << get_name() << " NOT RUNNING !");
        }else
        {
            if(boolean_value_)
            {
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");

            }else
            {
                set_status(BT::FAILURE);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
            }

        }
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


//void BT::ActionTestNode::set_status(ReturnStatus status){
//    status_ = status;
//}

void BT::ActionTestNode::set_time(int time){
    time_ = time;
}



void BT::ActionTestNode::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}


