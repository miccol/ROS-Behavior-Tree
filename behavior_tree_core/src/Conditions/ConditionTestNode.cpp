#include <Conditions/ConditionTestNode.h>


BT::ConditionTestNode::ConditionTestNode(std::string name) : ConditionNode::ConditionNode(name)
{
    Type = BT::Condition;

    // Thread start
    Thread = boost::thread(&ConditionTestNode::Exec, this);
}

BT::ConditionTestNode::~ConditionTestNode() {}

void BT::ConditionTestNode::Exec()
{
 int i = 0;   
    while(true)
    {
	
        // Waiting for a tick to come
        Semaphore.Wait();

        if(ReadState() == BT::Exit)
        {
            // The behavior tree is going to be destroied
            return;
        }

        // Condition checking and state update
        i++;
        if (i < 5)
        {
            SetNodeState(BT::Success);
            std::cout << get_name() << " returning Success" << BT::Success << "!" << std::endl;
        }
        else if( i<10)
        {
            SetNodeState(BT::Failure);
            std::cout << get_name() << " returning Failure" << BT::Failure << "!" << std::endl;
        } else
	{
            std::cout << get_name() << " reset i!" << std::endl;
            SetNodeState(BT::Failure);
	i=0;
	}
	

        // Resetting the state
        WriteState(BT::Idle);
    }
}
