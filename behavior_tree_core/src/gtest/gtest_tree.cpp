#include <gtest/gtest.h>
#include "behavior_tree.h"

struct SimpleSequenceTest : testing::Test
{
    BT:: SequenceNode* root;
    BT::ActionTestNode* action;
     BT::ConditionTestNode* condition;
    SimpleSequenceTest()
    {
        action = new BT::ActionTestNode("action");
        condition = new BT::ConditionTestNode("condition");

        root = new BT::SequenceNode("seq1");

        root->AddChild(condition);
        root->AddChild(action);
    }


};


struct ComplexSequenceTest : testing::Test
{
    BT:: SequenceNode* root;
    BT::ActionTestNode* action_1;
    BT::ConditionTestNode* condition_1;
    BT::ConditionTestNode* condition_2;

     BT:: SequenceNode* seq_conditions;

    ComplexSequenceTest()
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


struct SimpleFallbackTest : testing::Test
{
    BT:: FallbackNode* root;
    BT::ActionTestNode* action;
     BT::ConditionTestNode* condition;
    SimpleFallbackTest()
    {
        action = new BT::ActionTestNode("action");
        condition = new BT::ConditionTestNode("condition");

        root = new BT::FallbackNode("seq1");

        root->AddChild(condition);
        root->AddChild(action);
    }


};


struct ComplexFallbackTest : testing::Test
{
    BT:: FallbackNode* root;
    BT::ActionTestNode* action_1;
    BT::ConditionTestNode* condition_1;
    BT::ConditionTestNode* condition_2;

    BT:: FallbackNode* sel_conditions;

    ComplexFallbackTest()
    {
        action_1 = new BT::ActionTestNode("action 1");
        condition_1 = new BT::ConditionTestNode("condition 1");
        condition_2 = new BT::ConditionTestNode("condition 2");
        sel_conditions = new BT::FallbackNode("fallback_conditions");

        sel_conditions->AddChild(condition_1);
        sel_conditions->AddChild(condition_2);

        root = new BT::FallbackNode("root");
        root->AddChild(sel_conditions);
        root->AddChild(action_1);
    }
};




struct BehaviorTreeTest : testing::Test
{
    BT:: SequenceNode* root;
    BT::ActionTestNode* action_1;
    BT::ConditionTestNode* condition_1;
    BT::ConditionTestNode* condition_2;

     BT:: FallbackNode* sel_conditions;

    BehaviorTreeTest()
    {
        action_1 = new BT::ActionTestNode("action 1");
        condition_1 = new BT::ConditionTestNode("condition 1");
        condition_2 = new BT::ConditionTestNode("condition 2");
        sel_conditions = new BT::FallbackNode("fallback_conditions");

        sel_conditions->AddChild(condition_1);
        sel_conditions->AddChild(condition_2);

        root = new BT::SequenceNode("root");
        root->AddChild(sel_conditions);
        root->AddChild(action_1);
    }
};




struct SimpleSequenceWithMemoryTest : testing::Test
{
    BT:: SequenceNodeWithMemory* root;
    BT::ActionTestNode* action;
     BT::ConditionTestNode* condition;
    SimpleSequenceWithMemoryTest()
    {
        action = new BT::ActionTestNode("action");
        condition = new BT::ConditionTestNode("condition");

        root = new BT::SequenceNodeWithMemory("seq1");

        root->AddChild(condition);
        root->AddChild(action);
    }


};

struct ComplexSequenceWithMemoryTest : testing::Test
{
    BT:: SequenceNodeWithMemory* root;

    BT::ActionTestNode* action_1;
    BT::ActionTestNode* action_2;

    BT::ConditionTestNode* condition_1;
    BT::ConditionTestNode* condition_2;

     BT:: SequenceNodeWithMemory* seq_conditions;
     BT:: SequenceNodeWithMemory* seq_actions;

    ComplexSequenceWithMemoryTest()
    {
        action_1 = new BT::ActionTestNode("action 1");
        action_2 = new BT::ActionTestNode("action 2");


        condition_1 = new BT::ConditionTestNode("condition 1");
        condition_2 = new BT::ConditionTestNode("condition 2");

        seq_conditions = new BT::SequenceNodeWithMemory("sequence_conditions");
        seq_actions = new BT::SequenceNodeWithMemory("sequence_actions");

        seq_actions->AddChild(action_1);
        seq_actions->AddChild(action_2);

        seq_conditions->AddChild(condition_1);
        seq_conditions->AddChild(condition_2);

        root = new BT::SequenceNodeWithMemory("root");
        root->AddChild(seq_conditions);
        root->AddChild(seq_actions);
    }
};

struct SimpleFallbackWithMemoryTest : testing::Test
{
    BT::FallbackNodeWithMemory* root;
    BT::ActionTestNode* action;
     BT::ConditionTestNode* condition;
    SimpleFallbackWithMemoryTest()
    {
        action = new BT::ActionTestNode("action");
        condition = new BT::ConditionTestNode("condition");

        root = new BT::FallbackNodeWithMemory("seq1");

        root->AddChild(condition);
        root->AddChild(action);
    }


};

struct ComplexFallbackWithMemoryTest : testing::Test
{
    BT:: FallbackNodeWithMemory* root;

    BT::ActionTestNode* action_1;
    BT::ActionTestNode* action_2;

    BT::ConditionTestNode* condition_1;
    BT::ConditionTestNode* condition_2;

     BT:: FallbackNodeWithMemory* fal_conditions;
     BT:: FallbackNodeWithMemory* fal_actions;

    ComplexFallbackWithMemoryTest()
    {
        action_1 = new BT::ActionTestNode("action 1");
        action_2 = new BT::ActionTestNode("action 2");
        //action_1->set_boolean_value(false);
        action_1->set_boolean_value(false);
        condition_1 = new BT::ConditionTestNode("condition 1");
        condition_2 = new BT::ConditionTestNode("condition 2");

        fal_conditions = new BT::FallbackNodeWithMemory("fallback_conditions");
        fal_actions = new BT::FallbackNodeWithMemory("fallback_actions");

        fal_actions->AddChild(action_1);
        fal_actions->AddChild(action_2);

        fal_conditions->AddChild(condition_1);
        fal_conditions->AddChild(condition_2);

        root = new BT::FallbackNodeWithMemory("root");
        root->AddChild(fal_conditions);
        root->AddChild(fal_actions);
    }
};


/*************************************************TESTS START HERE**********************************************************************/



//TEST_F(SimpleSequenceTest, ConditionTrue) {

//    std::cout << "Ticking the root node !" << std::endl << std::endl;
//    // Ticking the root node
//    BT::ReturnStatus state = root->Tick();
//    //boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

//    ASSERT_EQ(BT::RUNNING, action->get_status());
//    ASSERT_EQ(BT::RUNNING, state);
//    root->Halt();

//}


//TEST_F(SimpleSequenceTest, ConditionTurnToFalse) {

//    BT::ReturnStatus state = root->Tick();


//    condition->set_boolean_value(false);

//    state = root->Tick();
//    ASSERT_EQ(BT::FAILURE, state);
//    ASSERT_EQ(BT::HALTED, action->get_status());
//    root->Halt();

//}


//TEST_F(ComplexSequenceTest, ComplexSequenceConditionsTrue) {

//        BT::ReturnStatus state = root->Tick();
//        //    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

//        ASSERT_EQ(BT::RUNNING, action_1->get_status());
//        ASSERT_EQ(BT::RUNNING, state);
//        root->Halt();


//}




//TEST_F(ComplexSequenceTest, ComplexSequenceConditions1ToFalse) {

//        BT::ReturnStatus state = root->Tick();

//        condition_1->set_boolean_value(false);

//        state = root->Tick();

//        ASSERT_EQ(BT::FAILURE, state);
//        ASSERT_EQ(BT::HALTED, action_1->get_status());
//        root->Halt();

//}

//TEST_F(ComplexSequenceTest, ComplexSequenceConditions2ToFalse) {

//        BT::ReturnStatus state = root->Tick();

//        condition_2->set_boolean_value(false);

//        state = root->Tick();

//        ASSERT_EQ(BT::FAILURE, state);
//        ASSERT_EQ(BT::HALTED, action_1->get_status());
//        root->Halt();

//}



//TEST_F(SimpleFallbackTest, ConditionTrue) {

//    std::cout << "Ticking the root node !" << std::endl << std::endl;
//    // Ticking the root node
//    condition->set_boolean_value(true);
//    BT::ReturnStatus state = root->Tick();

//    ASSERT_EQ(BT::IDLE, action->get_status());
//    ASSERT_EQ(BT::SUCCESS, state);
//    root->Halt();

//}


//TEST_F(SimpleFallbackTest, ConditionToFalse) {

//    condition->set_boolean_value(false);

//    BT::ReturnStatus state = root->Tick();
//    condition->set_boolean_value(true);


//    state = root->Tick();

//    ASSERT_EQ(BT::SUCCESS, state);
//    ASSERT_EQ(BT::HALTED, action->get_status());
//    root->Halt();

//}


//TEST_F(ComplexFallbackTest, Condition1ToTrue) {

//       condition_1->set_boolean_value(false);
//       condition_2->set_boolean_value(false);

//        BT::ReturnStatus state = root->Tick();

//        condition_1->set_boolean_value(true);

//        state = root->Tick();

//        ASSERT_EQ(BT::SUCCESS, state);

//        ASSERT_EQ(BT::HALTED, action_1->get_status());
//        root->Halt();

//}

//TEST_F(ComplexFallbackTest, Condition2ToTrue) {

//    condition_1->set_boolean_value(false);
//    condition_2->set_boolean_value(false);

//     BT::ReturnStatus state = root->Tick();

//     condition_2->set_boolean_value(true);

//     state = root->Tick();

//     ASSERT_EQ(BT::SUCCESS, state);
//     ASSERT_EQ(BT::HALTED, action_1->get_status());
//     root->Halt();

//}



//TEST_F(BehaviorTreeTest, Condition1ToFalseCondition2True) {

//    condition_1->set_boolean_value(false);
//    condition_2->set_boolean_value(true);

//     BT::ReturnStatus state = root->Tick();

//     ASSERT_EQ(BT::RUNNING, state);
//     ASSERT_EQ(BT::RUNNING, action_1->get_status());
//     root->Halt();

//}

//TEST_F(BehaviorTreeTest, Condition2ToFalseCondition1True) {

//    condition_2->set_boolean_value(false);
//    condition_1->set_boolean_value(true);

//     BT::ReturnStatus state = root->Tick();

//     ASSERT_EQ(BT::RUNNING, state);
//     ASSERT_EQ(BT::RUNNING, action_1->get_status());
//     root->Halt();

//}


//TEST_F(SimpleSequenceWithMemoryTest, ConditionTrue) {

//    std::cout << "Ticking the root node !" << std::endl << std::endl;
//    // Ticking the root node
//    BT::ReturnStatus state = root->Tick();
//    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

//    ASSERT_EQ(BT::RUNNING, action->get_status());
//    ASSERT_EQ(BT::RUNNING, state);
//    root->Halt();

//}


//TEST_F(SimpleSequenceWithMemoryTest, ConditionTurnToFalse) {

//    BT::ReturnStatus state = root->Tick();


//    condition->set_boolean_value(false);

//    state = root->Tick();
//    ASSERT_EQ(BT::RUNNING, state);
//    ASSERT_EQ(BT::RUNNING, action->get_status());
//    root->Halt();

//}





//TEST_F(ComplexSequenceWithMemoryTest, ConditionsTrue) {

//        BT::ReturnStatus state = root->Tick();
//        //    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

//        ASSERT_EQ(BT::RUNNING, action_1->get_status());
//        ASSERT_EQ(BT::IDLE, action_2->get_status());
//        ASSERT_EQ(BT::RUNNING, state);
//        root->Halt();


//}




//TEST_F(ComplexSequenceWithMemoryTest, Conditions1ToFalse) {

//        BT::ReturnStatus state = root->Tick();

//        condition_1->set_boolean_value(false);

//        state = root->Tick();

//        ASSERT_EQ(BT::RUNNING, action_1->get_status());
//        ASSERT_EQ(BT::IDLE, action_2->get_status());
//        ASSERT_EQ(BT::RUNNING, state);
//        root->Halt();


//}

//TEST_F(ComplexSequenceWithMemoryTest, Conditions2ToFalse) {

//        BT::ReturnStatus state = root->Tick();

//        condition_2->set_boolean_value(false);

//        state = root->Tick();

//        ASSERT_EQ(BT::RUNNING, action_1->get_status());
//        ASSERT_EQ(BT::IDLE, action_2->get_status());
//        ASSERT_EQ(BT::RUNNING, state);
//        root->Halt();

//}

//TEST_F(ComplexSequenceWithMemoryTest, Action1Done) {

//        BT::ReturnStatus state = root->Tick();

//        condition_2->set_boolean_value(false);

//        state = root->Tick();
//        boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
//        state = root->Tick();

//        //ASSERT_EQ(BT::IDLE, action_1->get_status());
//        ASSERT_EQ(BT::RUNNING, action_2->get_status());
//       // ASSERT_EQ(BT::RUNNING, state);
//        root->Halt();


//}




//TEST_F(SimpleFallbackWithMemoryTest, ConditionFalse) {

//    std::cout << "Ticking the root node !" << std::endl << std::endl;
//    // Ticking the root node
//    condition->set_boolean_value(false);
//    BT::ReturnStatus state = root->Tick();
//    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

//    ASSERT_EQ(BT::RUNNING, action->get_status());
//    ASSERT_EQ(BT::RUNNING, state);
//    root->Halt();

//}


//TEST_F(SimpleFallbackWithMemoryTest, ConditionTurnToTrue) {

//    condition->set_boolean_value(false);

//    BT::ReturnStatus state = root->Tick();


//    condition->set_boolean_value(true);

//    state = root->Tick();
//    ASSERT_EQ(BT::RUNNING, state);
//    ASSERT_EQ(BT::RUNNING, action->get_status());
//    root->Halt();

//}



TEST_F(ComplexFallbackWithMemoryTest, ConditionsTrue) {


        BT::ReturnStatus state = root->Tick();
        //    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        ASSERT_EQ(BT::IDLE, action_1->get_status());
        ASSERT_EQ(BT::IDLE, action_2->get_status());
        ASSERT_EQ(BT::SUCCESS, state);
        root->Halt();


}

TEST_F(ComplexFallbackWithMemoryTest, Condition1False) {

        condition_1->set_boolean_value(false);
        BT::ReturnStatus state = root->Tick();
        //    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        ASSERT_EQ(BT::IDLE, action_1->get_status());
        ASSERT_EQ(BT::IDLE, action_2->get_status());
        ASSERT_EQ(BT::SUCCESS, state);
        root->Halt();


}


TEST_F(ComplexFallbackWithMemoryTest, ConditionsFalse) {

    condition_1->set_boolean_value(false);
    condition_2->set_boolean_value(false);
    BT::ReturnStatus state = root->Tick();
    //    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::IDLE, action_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);
    root->Halt();


}




TEST_F(ComplexFallbackWithMemoryTest, Conditions1ToFalse) {

    condition_1->set_boolean_value(false);
        BT::ReturnStatus state = root->Tick();
        condition_1->set_boolean_value(true);


        state = root->Tick();

        ASSERT_EQ(BT::RUNNING, action_1->get_status());
        ASSERT_EQ(BT::IDLE, action_2->get_status());
        ASSERT_EQ(BT::RUNNING, state);
        root->Halt();


}

TEST_F(ComplexFallbackWithMemoryTest, Conditions2ToFalse) {
    condition_2->set_boolean_value(false);

        BT::ReturnStatus state = root->Tick();

        condition_2->set_boolean_value(true);

        state = root->Tick();

        ASSERT_EQ(BT::RUNNING, action_1->get_status());
        ASSERT_EQ(BT::IDLE, action_2->get_status());
        ASSERT_EQ(BT::RUNNING, state);
        root->Halt();

}

TEST_F(ComplexFallbackWithMemoryTest, Action1Done) {

        BT::ReturnStatus state = root->Tick();

        condition_2->set_boolean_value(false);

        state = root->Tick();
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
        state = root->Tick();

        //ASSERT_EQ(BT::IDLE, action_1->get_status());
        ASSERT_EQ(BT::RUNNING, action_2->get_status());
       // ASSERT_EQ(BT::RUNNING, state);
        root->Halt();


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "BehaviorTree");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


