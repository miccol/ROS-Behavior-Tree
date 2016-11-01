#include "BehaviorTree.h"

using namespace BT;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BehaviorTree");
    try
    {
        int TickPeriod_milliseconds = 1000;

        ActionTestNode* action = new ActionTestNode("action");
        ActionTestNode* condition = new ActionTestNode("condition");


         SequenceNode* sequence1 = new SequenceNode("seq1");
        //SequenceStarNode* sequence1 = new SequenceStarNode("seq1");

        sequence1->AddChild(condition);
        sequence1->AddChild(action);

        Execute(sequence1, TickPeriod_milliseconds);//from BehaviorTree.cpp

}
    catch (BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

return 0;
}


