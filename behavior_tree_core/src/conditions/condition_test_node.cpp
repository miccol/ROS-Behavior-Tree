#include <conditions/condition_test_node.h>


BT::ConditionTestNode::ConditionTestNode(std::string name) : ConditionNode::ConditionNode(name)
{
    type_ = BT::CONDITION_NODE;
    boolean_value_ = true;

    // thread_ start
    //thread_ = boost::thread(&ConditionTestNode::Exec, this);
}

BT::ConditionTestNode::~ConditionTestNode() {}

BT::NodeState BT::ConditionTestNode::Exec()
{
//    while(true)
    {
	
        // Waiting for a tick to come
     //   tick_engine.wait();

        if(ReadState() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return BT::EXIT;
        }

        // Condition checking and state update

        if (boolean_value_)
        {
          //  SetNodeState(BT::SUCCESS);
            std::cout << get_name() << " returning Success" << BT::SUCCESS << "!" << std::endl;
            return BT::SUCCESS;
        }
        else
        {
//            SetNodeState(BT::FAILURE);
            std::cout << get_name() << " returning Failure" << BT::FAILURE << "!" << std::endl;
            return BT::FAILURE;

        }
	


    }


}




	void BT::ConditionTestNode::set_boolean_value(bool boolean_value)
	{
		boolean_value_ = boolean_value;
	}

