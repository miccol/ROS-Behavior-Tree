#include <DecoratorNegationNode.h>


BT::DecoratorNegationNode::DecoratorNegationNode(std::string name) : ControlNode::ControlNode(name)
{
    // thread_ start
    thread_ = boost::thread(&DecoratorNegationNode::Exec, this);
}

BT::DecoratorNegationNode::~DecoratorNegationNode() {}

void BT::DecoratorNegationNode::Exec()
{


    // Waiting for the first tick to come
    tick_engine.wait();

    // Vector size initialization
    M = ChildNodes.size();

    // Simulating a tick for myself
    tick_engine.tick();

    while(true)
    {
        // Waiting for a tick to come
        tick_engine.wait();

        if(ReadState() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return;
        }

        // Checking if i was halted
        if (ReadState() != BT::HALTED)
        {
            // If not, the children can be ticked
            std::cout << get_name() << " ticked, ticking children..." << std::endl;




                if (ChildNodes[0]->get_type() == BT::ACTION_NODE)
                {
                    // 1) if it's an action:
                    // 1.1) read its state;
                    NodeState ActionState = ChildNodes[0]->ReadState();

                    if (ActionState == BT::IDLE)
                    {
                        // 1.2) if it's "Idle":
                        // 1.2.1) ticking it;
                        ChildNodes[0]->tick_engine.tick();

                        // 1.2.2) retrive its state as soon as it is available;
                        ChildStates[0] = ChildNodes[0]->GetNodeState();
                    }
                    else if (ActionState == BT::RUNNING)
                    {
                        // 1.3) if it's "Running":
                        // 1.3.1) saving "Running"
                        ChildStates[0] = BT::RUNNING;
                    }
                    else
                    {
                        // 1.4) if it's "Success" of "Failure" (it can't be "Halted"!):
                        // 1.2.1) ticking it;
                        ChildNodes[0]->tick_engine.tick();

                        // 1.2.2) saving the read state;
                        ChildStates[0] = ActionState;
                    }
                }
                else
                {
                    // 2) if it's not an action:
                    // 2.1) ticking it;
                    ChildNodes[0]->tick_engine.tick();

                    // 2.2) retrive its state as soon as it is available;
                    ChildStates[0] = ChildNodes[0]->GetNodeState();
                }

                // 3) if the child state is a success:
                if(ChildStates[0] == BT::SUCCESS)
                {
                    // 3.1) the node state is equal to failure since I am negating the status
                    SetNodeState(BT::FAILURE);

                    // 3.2) resetting the state;
                    WriteState(BT::IDLE);

                    std::cout << get_name() << " returning " << BT::FAILURE << "!" << std::endl;
                }
                else if(ChildStates[0] == BT::FAILURE)
                {
                    // 4.1) the node state is equal to success since I am negating the status
                    SetNodeState(BT::SUCCESS);

                    // 4.2) state reset;
                    WriteState(BT::IDLE);

                    std::cout << get_name() << " returning " << BT::SUCCESS << "!" << std::endl;

                } else
                // 5) if the child state is  running
                {
                    // 5.1) the node state is equal to running
                    SetNodeState(BT::RUNNING);

                    // 5.2) state reset;
                    WriteState(BT::IDLE);
                }

        }
        else
        {
            // If it was halted, all the "busy" children must be halted too
            std::cout << get_name() << " halted! Halting all the children..." << std::endl;

                if (ChildNodes[0]->get_type() != BT::ACTION_NODE && ChildStates[0] == BT::RUNNING)
                {
                    // if the control node was running:
                    // halting it;
                    ChildNodes[0]->Halt();

                    // sync with it (it's waiting on the semaphore);
                    ChildNodes[0]->tick_engine.tick();

                    std::cout << get_name() << " halting child  "  << "!" << std::endl;
                }
                else if (ChildNodes[0]->get_type() == BT::ACTION_NODE && ChildNodes[0]->ReadState() == BT::RUNNING)
                {
                    std::cout << get_name() << " trying halting child  "  << "..." << std::endl;

                    // if it's a action node that hasn't finished its job:
                    // trying to halt it:
                    if (ChildNodes[0]->Halt() == false)
                    {
                        // this means that, before this node could set its child state
                        // to "Halted", the child had already written the action outcome;
                        // sync with him ignoring its state;
                        ChildNodes[0]->tick_engine.tick();

                        std::cout << get_name() << " halting of child  "  << " failed!" << std::endl;
                    }

                    std::cout << get_name() << " halting of child  "  << " succedeed!" << std::endl;
                }
                else if (ChildNodes[0]->get_type() == BT::ACTION_NODE && ChildNodes[0]->ReadState() != BT::IDLE)
                {
                    // if it's a action node that has finished its job:
                    // ticking it without saving its returning state;
                    ChildNodes[0]->tick_engine.tick();
                }

                // updating its vector cell
                ChildStates[0] = BT::IDLE;


            // Resetting the node state
            WriteState(BT::IDLE);
        }
    }
}

int BT::DecoratorNegationNode::DrawType()
{
    // Lock acquistion

    return BT::DECORATOR;
}


void BT::DecoratorNegationNode::AddChild(BT::TreeNode* Child)
{
    // Checking if the Child is not already present

        if (ChildNodes.size() > 0)
        {
            throw BehaviorTreeException("Decorators can have only one child");
        }


    ChildNodes.push_back(Child);
    ChildStates.push_back(BT::IDLE);
}
