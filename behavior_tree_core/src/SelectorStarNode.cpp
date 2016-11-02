#include <SelectorStarNode.h>


BT::SelectorStarNode::SelectorStarNode(std::string name) : ControlNode::ControlNode(name)
{
    // thread_ start
    thread_ = boost::thread(&SelectorStarNode::Exec, this);
}

BT::SelectorStarNode::~SelectorStarNode() {}

void BT::SelectorStarNode::Exec()
{
    unsigned int i;

    // Waiting for the first tick to come
    tick_engine.wait();

    // Vector size initialization
    M = ChildNodes.size();

    // Simulating a tick for myself
    tick_engine.tick();
    i = 0; //Initialize the index of the child to tick

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

            // For each child:
            while(i < M)
            {
                if (ChildNodes[i]->get_type() == BT::ACTION_NODE)
                {
                    // 1) if it's an action:
                    // 1.1) read its state;
                    NodeState ActionState = ChildNodes[i]->ReadState();

                    if (ActionState == BT::IDLE)
                    {
                        // 1.2) if it's "Idle":
                        // 1.2.1) ticking it;
                        ChildNodes[i]->tick_engine.tick();

                        // 1.2.2) retrive its state as soon as it is available;
                        ChildStates[i] = ChildNodes[i]->GetNodeState();
                    }
                    else if (ActionState == BT::RUNNING)
                    {
                        // 1.3) if it's "Running":
                        // 1.3.1) saving "Running"
                        ChildStates[i] = BT::RUNNING;
                    }
                    else
                    {
                        // 1.4) if it's "Success" of "Failure" (it can't be "Halted"!):
                        // 1.2.1) ticking it;
                        ChildNodes[i]->tick_engine.tick();

                        // 1.2.2) saving the read state;
                        ChildStates[i] = ActionState;
                    }
                }
                else
                {
                    // 2) if it's not an action:
                    // 2.1) ticking it;
                    ChildNodes[i]->tick_engine.tick();

                    // 2.2) retrive its state as soon as it is available;
                    ChildStates[i] = ChildNodes[i]->GetNodeState();
                }

                // 3) if the child state is not a success:
                if(ChildStates[i] != BT::FAILURE)
                {


                    // 3.1) the node state is equal to it;
                    SetNodeState(ChildStates[i]);

                    // 3.2) state reset;
                    WriteState(BT::IDLE);
                    if (ChildStates[i] == BT::SUCCESS)
                    {
                     i = 0; // Final state_ of rhe selector node. Child index reinitialized
                     }


                    std::cout << get_name() << " returning " << ChildStates[i] << "!" << std::endl;

                    // 3.4) the while loop must end here.
                    break;
                } else if(ChildStates[i] == BT::FAILURE)//if child i has failed the selector star node can tick the next child
                {
                    i ++;
                }
            }

            if (i == M)
            {
                // 4) if all of its children return "failure":
                // 4.1) the node state must be "failure";
                SetNodeState(BT::FAILURE);
                i = 0;
                // 4.2) resetting the state;
                WriteState(BT::IDLE);

                std::cout << get_name() << " returning " << BT::SUCCESS << "!" << std::endl;
            }
        }
        else
        {
            // If it was halted, all the "busy" children must be halted too
            std::cout << get_name() << " halted! Halting all the children..." << std::endl;


            HaltChildren(0);
            // Resetting the node state
            WriteState(BT::IDLE);
        }
    }
}


int BT::SelectorStarNode::DrawType()
{
    // Lock acquistion

    return BT::SELECTORSTAR;
}
