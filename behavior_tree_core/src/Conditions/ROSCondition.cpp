#include <Conditions/ROSCondition.h>


enum Status {RUNNING,ROS_SUCCESS, ROS_FAILURE};


BT::ROSCondition::ROSCondition(std::string name) : ConditionNode::ConditionNode(name)
{
    type_ = BT::CONDITION_NODE;

    // thread_ start
    thread_ = boost::thread(&ROSCondition::Exec, this);
}

BT::ROSCondition::~ROSCondition() {}

void BT::ROSCondition::Exec()
{

   // ROS_INFO("Waiting For the Acutator %s to start", name);

    actionlib::SimpleActionClient<behavior_tree_core::BTAction> ac(get_name(), true);
    ac.waitForServer(); //will wait for infinite time until the server starts
    //ROS_INFO("Actuator %s Started", get_name());

    behavior_tree_core::BTGoal goal;
    while(true)
    {
	
        // Waiting for a tick to come
        tick_engine.wait();
        node_result.status = 0;

        ROS_INFO("I am running the request");


        if(ReadState() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return;
        }

        // Condition checking and state update
       // do{
            ac.sendGoal(goal);
            ac.waitForResult(ros::Duration(30.0));
            node_result = *(ac.getResult());

            std::cout << " Condition Status" << node_result.status << "!" << std::endl;


       // }while(node_result.status != 2 && node_result.status != 1 ); //if it is not halted and has not returned a status

        if (node_result.status == ROS_SUCCESS)
        {
            SetNodeState(BT::SUCCESS);
           std::cout << get_name() << " returning Success" << BT::SUCCESS << "!" << std::endl;
        }
        else if( node_result.status == ROS_FAILURE)
        {
            SetNodeState(BT::FAILURE);
            std::cout << get_name() << " returning Failure" << BT::FAILURE << "!" << std::endl;
        }else{
            SetNodeState(BT::FAILURE);
            std::cout << get_name() << " returning NOTHING" << BT::FAILURE << "!" << std::endl;
        }

	

        // Resetting the state
        WriteState(BT::IDLE);
    }
}
