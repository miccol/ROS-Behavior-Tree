#include <Actions/ROSAction.h>



using namespace BT;

enum Status {RUNNING,SUCCESS, FAILURE};

ROSAction::ROSAction(std::string Name) : ActionNode::ActionNode(Name)
{
    Type = Action;
    // Thread start
    Thread = boost::thread(&ROSAction::Exec, this);

}

ROSAction::~ROSAction() {}

void ROSAction::Exec()
{


    //bool hasReturned = false;
	  // create the action client
	  // true causes the client to spin its own thread
    ROS_INFO("Waiting For the Acutator %s to start", Name.c_str());

      actionlib::SimpleActionClient<behavior_tree_core::BTAction> ac(Name, true);

      ac.waitForServer(); //will wait for infinite time until the server starts
      ROS_INFO("Tthe Acutator %s has started", Name.c_str());

      behavior_tree_core::BTGoal goal;
      node_result.status = RUNNING;//
    while(true)
    {
        // Waiting for a tick to come
        Semaphore.Wait();

        if(ReadState() == Exit)
        {
            // The behavior tree is going to be destroied
            return;
        }

        // Running state
        SetNodeState(Running);
        std::cout << Name << " returning " << Running << "!" << std::endl;
        node_result.status = RUNNING;
        // Perform action...
        ROS_INFO("I am running the request to %s",Name.c_str());
        ac.sendGoal(goal);
        do
        {
            node_result = *(ac.getResult());//checking the result
        } while(node_result.status == RUNNING && ReadState() == Running);
            ROS_INFO("The Server Has Replied Or I the node is halted");

        std::cout << Name << " RETURNING " << node_result.status << "!" << std::endl;

        if(ReadState() == Exit)
        {
            // The behavior tree is going to be destroied
            return;
        }


        if (node_result.status == SUCCESS)
        {
            // trying to set the outcome state:
            if (WriteState(Success) != true)
            {
                // meanwhile, my father halted me!
                std::cout << Name << " Halted!" << std::endl;
                ROS_INFO("I am cancelling the request");
                ac.cancelGoal();
                // Resetting the state
                WriteState(Idle);
                continue;
            }

            std::cout << Name << " returning Success " << Success << "!" << std::endl;
        }
        else if( node_result.status == FAILURE)
        {
            // trying to set the outcome state:
            if (WriteState(Failure) != true)
            {
                // meanwhile, my father halted me!
                std::cout << Name << " Halted!" << std::endl;
                ROS_INFO("I am cancelling the request");
                ac.cancelGoal();
                // Resetting the state
                WriteState(Idle);
                continue;
            }

            std::cout << Name << " returning Failure" << Failure << "!" << std::endl;
        }else{//it means that the parent has halted the node

            std::cout << Name << " Halted!" << std::endl;
            ROS_INFO("I am cancelling the request");
            ac.cancelGoal();
            // Resetting the state
            WriteState(Idle);
            continue;

            std::cout << Name << " returning NOTHING (HALTED)" << Failure << "!" << std::endl;
        }


            std::cout << Name << " returning " << Success << "!" << std::endl;


        // Synchronization
        // (my father is telling me that it has read my new state)
        Semaphore.Wait();

        if(ReadState() == Exit)
        {
            // The behavior tree is going to be destroied
            return;
        }

        // Resetting the state
        WriteState(Idle);
    }
}

bool ROSAction::Halt()
{

        ROS_INFO("I am Halting the client");
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(StateMutex);

    // Checking for "Running" correctness
    if (State != Running)
    {
        return false;
    }

    State = Halted;
    return true;
}
