#include "behavior_tree.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "BehaviorTree");
    try
    {
        int TickPeriod_milliseconds = 1000;

//        BT::ActionTestNode* action = new BT::ActionTestNode("1234567 89\n77773737377373737373737373737373737");
//        BT::ConditionTestNode* condition = new BT::ConditionTestNode("01234567");
//        condition->set_boolean_value(true);


//        BT:: SequenceNode* sequence1 = new BT::SequenceNode("seq1");

//        sequence1->AddChild(condition);
//        sequence1->AddChild(action);


        std::string text = "";

        BT:: SequenceNode* root = new BT::SequenceNode("seq1");

        for (int i = 0; i < 10; i++)
        {
            BT:: SequenceNode* seq = new BT::SequenceNode("seq");
            text = "Action ";
            text += std::to_string(i);
            BT::ActionTestNode* action = new BT::ActionTestNode(text);
            text = "Condition ";
            text += std::to_string(i);
            BT::ConditionTestNode* condition = new BT::ConditionTestNode(text);
            seq->AddChild(condition);

            seq->AddChild(action);

            root->AddChild(seq);
        }

        Execute(root, TickPeriod_milliseconds);//from BehaviorTree.cpp

}
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

return 0;
}


