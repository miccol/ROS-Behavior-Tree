#include <gtest/gtest.h>
#include "behavior_tree.h"

struct BehaviorTreeTest : testing::Test
{
    BT:: SequenceNode* root;
    BT::ActionTestNode* action;
     BT::ConditionTestNode* condition;
    BehaviorTreeTest()
    {
        action = new BT::ActionTestNode("action");
        condition = new BT::ConditionTestNode("condition");


        root = new BT::SequenceNode("seq1");

        root->AddChild(condition);
        root->AddChild(action);
    }


};


TEST_F(BehaviorTreeTest, ActionRunning) {


    std::cout << "Ticking the root node !" << std::endl << std::endl;

    // Ticking the root node
    BT::NodeState state = root->Exec();

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    ASSERT_EQ(BT::RUNNING, action->ReadState());

}

TEST_F(BehaviorTreeTest, TreeRunning) {
    // Ticking the root node
    BT::NodeState state = root->Exec();

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    ASSERT_EQ(BT::RUNNING, state);
    root->Halt();

}


TEST_F(BehaviorTreeTest, ActionHalted) {

    BT::NodeState state = root->Exec();

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    condition->set_boolean_value(false);

    root->Exec();


    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    ASSERT_EQ(BT::HALTED,action->ReadState());
    root->Halt();

}

//TEST_F(BehaviorTreeTest, TreeFailure) {
//    // Ticking the root node
//    condition->set_boolean_value(false);

//    BT::NodeState state = root->Exec();
//    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

//    ASSERT_EQ(BT::FAILURE, state);

//}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "BehaviorTree");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


