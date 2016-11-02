#include <conditions/condition_test_node.h>


BT::ConditionTestNode::ConditionTestNode(std::string name) : ConditionNode::ConditionNode(name)
{
    type_ = BT::CONDITION_NODE;

    // thread_ start
    thread_ = boost::thread(&ConditionTestNode::Exec, this);
}

BT::ConditionTestNode::~ConditionTestNode() {}

void BT::ConditionTestNode::Exec()
{
 int i = 0;   
    while(true)
    {
	
        // Waiting for a tick to come
        tick_engine.wait();

        if(ReadState() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return;
        }

        // Condition checking and state update
        i++;
        if (i < 5)
        {
            SetNodeState(BT::SUCCESS);
            std::cout << get_name() << " returning Success" << BT::SUCCESS << "!" << std::endl;
        }
        else if( i<10)
        {
            SetNodeState(BT::FAILURE);
            std::cout << get_name() << " returning Failure" << BT::FAILURE << "!" << std::endl;
        } else
	{
            std::cout << get_name() << " reset i!" << std::endl;
            SetNodeState(BT::FAILURE);
	i=0;
	}
	

        // Resetting the state
        WriteState(BT::IDLE);
    }
}
