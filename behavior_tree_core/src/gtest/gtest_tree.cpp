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


struct BehaviorTreeTest2 : testing::Test
{
    BT:: SequenceNode* root;
    BT::ActionTestNode* action_1;
    BT::ConditionTestNode* condition_1;
    BT::ConditionTestNode* condition_2;

     BT:: SequenceNode* seq_conditions;
     BT:: SequenceNode* seq_actions;

    BehaviorTreeTest2()
    {
        action_1 = new BT::ActionTestNode("action 1");
        condition_1 = new BT::ConditionTestNode("condition 1");
        condition_2 = new BT::ConditionTestNode("condition 2");
        seq_conditions = new BT::SequenceNode("sequence_conditions");

        seq_conditions->AddChild(condition_1);
        seq_conditions->AddChild(condition_2);

        root = new BT::SequenceNode("root");
        root->AddChild(seq_conditions);
        root->AddChild(action_1);
    }
};


TEST_F(BehaviorTreeTest, SimpleSequenceConditionTrue) {

    std::cout << "Ticking the root node !" << std::endl << std::endl;
    // Ticking the root node
    BT::NodeState state = root->Tick();

    ASSERT_EQ(BT::RUNNING, action->get_status());
  // ASSERT_EQ(BT::RUNNING, state);
}


TEST_F(BehaviorTreeTest, SimpleSequenceConditionTurnToFalse) {

    BT::NodeState state = root->Tick();


    condition->set_boolean_value(false);

    state = root->Tick();
    ASSERT_EQ(BT::FAILURE, state);
    ASSERT_EQ(BT::HALTED, action->get_status());
    root->Halt();

}


TEST_F(BehaviorTreeTest2, ComplexSequenceConditionsTrue) {

        BT::NodeState state = root->Tick();
        //    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        ASSERT_EQ(BT::RUNNING, action_1->get_status());
        ASSERT_EQ(BT::RUNNING, state);

}




TEST_F(BehaviorTreeTest2, ComplexSequenceConditions1ToFalse) {

        BT::NodeState state = root->Tick();

        condition_1->set_boolean_value(false);

        state = root->Tick();

        ASSERT_EQ(BT::FAILURE, state);
        ASSERT_EQ(BT::HALTED, action_1->get_status());
}

TEST_F(BehaviorTreeTest2, ComplexSequenceConditions2ToFalse) {

        BT::NodeState state = root->Tick();

        condition_2->set_boolean_value(false);

        state = root->Tick();

        ASSERT_EQ(BT::FAILURE, state);
        ASSERT_EQ(BT::HALTED, action_1->get_status());
}











int main(int argc, char **argv)
{
    ros::init(argc, argv, "BehaviorTree");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


