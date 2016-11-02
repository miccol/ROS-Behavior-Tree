#include <Actions/ROSAction.h>




enum Status {ROS_RUNNING,ROS_SUCCESS, ROS_FAILURE};

BT::ROSAction::ROSAction(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    // thread_ start
    thread_ = boost::thread(&ROSAction::Exec, this);

}

BT::ROSAction::~ROSAction() {}

void BT::ROSAction::Exec()
{


    //bool hasReturned = false;
	  // create the action client
	  // true causes the client to spin its own thread
    ROS_INFO("Waiting For the Acutator %s to start", get_name().c_str());

      actionlib::SimpleActionClient<behavior_tree_core::BTAction> ac(get_name(), true);

      ac.waitForServer(); //will wait for infinite time until the server starts
      ROS_INFO("Tthe Acutator %s has started", get_name().c_str());

      behavior_tree_core::BTGoal goal;
      node_result.status = ROS_RUNNING;//
    while(true)
    {
        // Waiting for a tick to come
        tick_engine.wait();

        if(ReadState() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return;
        }

        // Running state
        SetNodeState(BT::RUNNING);
        std::cout << get_name() << " returning " << BT::RUNNING << "!" << std::endl;
        node_result.status = ROS_RUNNING;
        // Perform action...
        ROS_INFO("I am running the request to %s",get_name().c_str());
        ac.sendGoal(goal);
        do
        {
            node_result = *(ac.getResult());//checking the result
        } while(node_result.status == ROS_RUNNING && ReadState() == BT::RUNNING);
            ROS_INFO("The Server Has Replied Or I the node is halted");

        std::cout << get_name() << " RETURNING " << node_result.status << "!" << std::endl;

        if(ReadState() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return;
        }


        if (node_result.status == ROS_SUCCESS)
        {
            // trying to set the outcome state:
            if (WriteState(BT::SUCCESS) != true)
            {
                // meanwhile, my father halted me!
                std::cout << get_name() << " Halted!" << std::endl;
                ROS_INFO("I am cancelling the request");
                ac.cancelGoal();
                // Resetting the state
                WriteState(BT::IDLE);
                continue;
            }

            std::cout << get_name() << " returning Success " << BT::SUCCESS << "!" << std::endl;
        }
        else if( node_result.status == ROS_FAILURE)
        {
            // trying to set the outcome state:
            if (WriteState(BT::FAILURE) != true)
            {
                // meanwhile, my father halted me!
                std::cout << get_name() << " Halted!" << std::endl;
                ROS_INFO("I am cancelling the request");
                ac.cancelGoal();
                // Resetting the state
                WriteState(BT::IDLE);
                continue;
            }

            std::cout << get_name() << " returning Failure" << BT::FAILURE << "!" << std::endl;
        }else{//it means that the parent has halted the node

            std::cout << get_name() << " Halted!" << std::endl;
            ROS_INFO("I am cancelling the request");
            ac.cancelGoal();
            // Resetting the state
            WriteState(BT::IDLE);
            continue;

            std::cout << get_name() << " returning NOTHING (HALTED)" << BT::FAILURE << "!" << std::endl;
        }


            std::cout << get_name() << " returning " << BT::SUCCESS << "!" << std::endl;


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

bool BT::ROSAction::Halt()
{

        ROS_INFO("I am Halting the client");
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(state_mutex_);

    // Checking for "Running" correctness
    if (state_ != BT::RUNNING)
    {
        return false;
    }

    state_ = BT::HALTED;
    return true;
}
