#include "BehaviorTree.h"

using namespace BT;

int main(int argc, char **argv)
{
    try
    {
        int TickPeriod_milliseconds = 1000;

        ROSAction* action = new ROSAction("A1");
        ROSCondition* condition = new ROSCondition("C1");


        SequenceNode* sequence1 = new SequenceNode("seq1");

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


