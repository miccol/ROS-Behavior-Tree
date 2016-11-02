#include <ParallelNode.h>


BT::ParallelNode::ParallelNode(std::string name) : ControlNode::ControlNode(name)
{
    // Initializations
    N = std::numeric_limits<unsigned int>::max();
    StateUpdate = false;

    // thread_ start
    thread_ = boost::thread(&ParallelNode::Exec, this);
}

BT::ParallelNode::~ParallelNode() {}

void BT::ParallelNode::SetThreshold(unsigned int N)
{
    // Vector size initialization
    M = ChildNodes.size();

    // Checking N correctness
    if (N > M)
    {
        std::stringstream S;
        S << "Wrong N threshold for '" << get_name() << "'. M=" << M << " while N=" << N << ". N should be <= M.";
        throw BehaviorTreeException(S.str());
    }

    this->N = N;

    // First tick for the thread
    tick_engine.tick();
}

void BT::ParallelNode::Exec()
{
    unsigned int i;

    // Waiting for a first tick to come
    tick_engine.wait();

    // Checking N correctness
    if (N == std::numeric_limits<unsigned int>::max())
    {
        throw BehaviorTreeException("'" + get_name() + "' has no valid N threashold set. You should set it before ticking the node.");
    }

    // Vector construction
    for (i=0; i<M; i++)
    {
        ChildStatesUpdated.push_back(false);
    }

    while(true)
    {
        // Waiting for a tick to come
        tick_engine.wait();

        if(ReadState() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return;
        }

        // Variables reset
        StateUpdate = false;
        Successes = Failures = Runnings = 0;
        for (i=0; i<M; i++)
        {
            ChildStatesUpdated[i] = false;
        }

        // Checking if i was halted
        if (ReadState() != BT::HALTED)
        {
            std::cout << get_name() << " ticked, ticking actions..." << std::endl;

            // If not, before ticking the children, checking if a state can
            // be immediatelly returned.
            for(i = 0; i<M; i++)
            {
                if (ChildNodes[i]->get_type() != BT::ACTION_NODE)
                {
                    continue;
                }

                ChildStates[i] = ChildNodes[i]->ReadState();

                if (ChildStates[i] == BT::SUCCESS)
                {
                    // The action node has finished, sync
                    ChildNodes[i]->tick_engine.tick();

                    Successes++;
                    ChildStatesUpdated[i] = true;
                }
                else if (ChildStates[i] == BT::FAILURE)
                {
                    // The action node has finished, sync
                    ChildNodes[i]->tick_engine.tick();

                    Failures++;
                    ChildStatesUpdated[i] = true;
                }
                else if (ChildStates[i] == BT::RUNNING)
                {
                    Runnings++;
                    ChildStatesUpdated[i] = true;
                }

                if (Successes >= N)
                {
                    // Returning success
                    SetNodeState(BT::SUCCESS);
                    std::cout << get_name() << " returning Success! " << std::endl;
                    StateUpdate = true;

                    // Exit the for loop
                    break;
                }
                else if (Failures > M - N)
                {
                    // Returning failure
                    SetNodeState(BT::FAILURE);
                    std::cout << get_name() << " returning Failure! " << std::endl;
                    StateUpdate = true;

                    // Exit the for loop
                    break;
                }
                else if (Runnings > M - N && Runnings >= N)
                {
                    // Neither a Success nor a Failure could be returned
                    // Returning Running
                    SetNodeState(BT::RUNNING);
                    std::cout << get_name() << " returning Running! " << std::endl;
                    StateUpdate = true;

                    // Exit the for loop
                    break;
                }
            }

            if (StateUpdate == true)
            {
                // If it is known what to return...
                std::cout << get_name() << " knows what to return... " << std::endl;

                if (ReadState() == BT::SUCCESS || ReadState() == BT::FAILURE)
                {
                    // Halting the running actions
                    for (i=0; i<M; i++)
                    {
                        if (ChildStatesUpdated[i] == false || ChildStates[i] != BT::RUNNING)
                        {
                            continue;
                        }

                        std::cout << get_name() << " trying halting (action) child number " << i << "..." << std::endl;

                        if (ChildNodes[i]->Halt() == false)
                        {
                            // this means that, before this node could set its child state
                            // to "Halted", the child had already written the action outcome;
                            // sync with him ignoring its state;
                            ChildNodes[i]->tick_engine.tick();

                            std::cout << get_name() << " halting of child number " << i << " failed!" << std::endl;
                        }
                        else
                        {
                            std::cout << get_name() << " halting of child number " << i << " succedeed!" << std::endl;
                        }

                        // updating its vector cell;
                        ChildStates[i] = BT::IDLE;
                    }

                    // Ticking the other children, but halting them if they
                    // return Running.
                    std::cout << get_name() << " ticking the remaining children... " << std::endl;

                    for(i = 0; i<M; i++)
                    {
                        if (ChildStatesUpdated[i] == true)
                        {
                            continue;
                        }

                        // ticking it;
                        ChildNodes[i]->tick_engine.tick();

                        // retrive its state as soon as it is available;
                        ChildStates[i] = ChildNodes[i]->GetNodeState();
                        if (ChildStates[i] == BT::RUNNING)
                        {
                            std::cout << get_name() << " halting (control) child number " << i << "..." << std::endl;

                            // if it is running, halting it;
                            ChildNodes[i]->Halt();

                            // sync with it (it's waiting on the semaphore);
                            ChildNodes[i]->tick_engine.tick();
                        }

                        // updating its vector cell;
                        ChildStates[i] = BT::IDLE;
                        ChildStatesUpdated[i] = true;
                    }

                    // Resetting the node state
                    if (ReadState() != BT::HALTED)
                    {
                        WriteState(BT::IDLE);
                    }

                    // Next cicle
                    continue;
                }
                else if (Runnings + Failures + Successes < M)
                {
                    // Returning running! But some children haven't been ticked yet!

                    std::cout << get_name() << " ticking the remaining children and ignoring their state... " << std::endl;

                    // Ticking the remaining children (ignoring their states)
                    for(i = 0; i<M; i++)
                    {
                        if (ChildStatesUpdated[i] == true)
                        {
                            continue;
                        }

                        if (ChildNodes[i]->get_type() == BT::ACTION_NODE)
                        {
                            // if it's an action:
                            // read its state;
                            NodeState ActionState = ChildNodes[i]->ReadState();

                            if (ActionState == BT::IDLE)
                            {
                                // if it's "Idle":
                                // ticking it;
                                ChildNodes[i]->tick_engine.tick();

                                // retriving its state as soon as it is available;
                                ChildStates[i] = ChildNodes[i]->GetNodeState();
                            }
                            else if (ActionState == BT::RUNNING)
                            {
                                // It's ok, it can continue running
                                ChildStates[i] = BT::RUNNING;
                            }
                            else
                            {
                                // if it's "Success" of "Failure" (it can't be "Halted"!):
                                // sync with it;
                                ChildNodes[i]->tick_engine.tick();

                                // ticking it;
                                ChildNodes[i]->tick_engine.tick();

                                // retriving its state as soon as it is available;
                                ChildStates[i] = ChildNodes[i]->GetNodeState();
                            }
                        }
                        else
                        {
                            // if it's not an action:
                            // ticking it;
                            ChildNodes[i]->tick_engine.tick();

                            // retrive its state as soon as it is available;
                            ChildStates[i] = ChildNodes[i]->GetNodeState();
                        }
                    }

                    continue;
                }
                else
                {
                    // Returning Running! All the children have already been ticked.

                    continue;
                }
            }

            // If it wasn't possible to decide which state to return
            // by considerating only the action nodes, the remaining children
            // must be ticked and their state must be considered

            std::cout << get_name() << " doesn't know yet what to return, ticking the remaining children..." << std::endl;

            // For each remained child (conditions and controls):
            for(i = 0; i<M; i++)
            {
                if (ChildStatesUpdated[i] == true)
                {
                    continue;
                }

                // ticking it;
                ChildNodes[i]->tick_engine.tick();

                // retrive its state as soon as it is available;
                ChildStates[i] = ChildNodes[i]->GetNodeState();

                if (ChildStates[i] == BT::SUCCESS)
                {
                    Successes++;
                    ChildStatesUpdated[i] = true;
                }
                else if (ChildStates[i] == BT::FAILURE)
                {
                    Failures++;
                    ChildStatesUpdated[i] = true;
                }
                else
                {
                    Runnings++;
                    ChildStatesUpdated[i] = true;
                }

                if (Successes >= N)
                {
                    // Returning success
                    SetNodeState(BT::SUCCESS);
                    std::cout << get_name() << " returning Success! " << std::endl;
                    StateUpdate = true;

                    // Exit the for loop
                    break;
                }
                else if (Failures > M - N)
                {
                    // Returning failure
                    SetNodeState(BT::FAILURE);
                    std::cout << get_name() << " returning Failure! " << std::endl;
                    StateUpdate = true;

                    // Exit the for loop
                    break;
                }
                else if (Runnings > M - N && Runnings >= N)
                {
                    // Neither a Success nor a Failure could be returned
                    // Returning Running
                    SetNodeState(BT::RUNNING);
                    std::cout << get_name() << " returning Running! " << std::endl;
                    StateUpdate = true;

                    // Exit the for loop
                    break;
                }
            }

            if (StateUpdate == true && ReadState() != BT::RUNNING)
            {
                std::cout << get_name() << " knows what to return... " << std::endl;

                // Halting all the running nodes (that have been ticked!)
                for (i=0; i<M; i++)
                {
                    if (ChildStatesUpdated[i] == false || ChildStates[i] != BT::RUNNING)
                    {
                        continue;
                    }

                    if (ChildNodes[i]->get_type() == BT::ACTION_NODE)
                    {
                        std::cout << get_name() << " trying halting (action) child number " << i << "..." << std::endl;

                        if (ChildNodes[i]->Halt() == false)
                        {
                            // this means that, before this node could set its child state
                            // to "Halted", the child had already written the action outcome;
                            // sync with him ignoring its state;
                            ChildNodes[i]->tick_engine.tick();

                            std::cout << get_name() << " halting of child number " << i << " failed!" << std::endl;
                        }
                        else
                        {
                            std::cout << get_name() << " halting of child number " << i << " succedeed!" << std::endl;
                        }
                    }
                    else
                    {
                        // halting it;
                        ChildNodes[i]->Halt();

                        // sync with it (it's waiting on the semaphore);
                        ChildNodes[i]->tick_engine.tick();

                        std::cout << get_name() << " halting child number " << i << "!" << std::endl;
                    }

                    // updating its vector cell;
                    ChildStates[i] = BT::IDLE;
                }

                // Ticking the other children, but halting them is they
                // return Running.
                std::cout << get_name() << " ticking the remaining children... " << std::endl;

                for(i = 0; i<M; i++)
                {
                    if (ChildStatesUpdated[i] == true)
                    {
                        continue;
                    }

                    // ticking it;
                    ChildNodes[i]->tick_engine.tick();

                    // retrive its state as soon as it is available
                    ChildStates[i] = ChildNodes[i]->GetNodeState();

                    if (ChildStates[i] == BT::RUNNING)
                    {
                        std::cout << get_name() << " halting (control) child number " << i << "..." << std::endl;

                        // if it is running, halting it;
                        ChildNodes[i]->Halt();

                        // sync with it (it's waiting on the semaphore);
                        ChildNodes[i]->tick_engine.tick();

                        // updating its vector cell;
                        ChildStates[i] = BT::IDLE;
                    }
                }

                // Resetting the node state
                if (ReadState() != BT::HALTED)
                {
                    WriteState(BT::IDLE);
                }
            }
            else if (StateUpdate == true && Runnings + Failures + Successes < M)
            {
                // Returning running, but there still children to be ticked

                std::cout << get_name() << " ticking the remaining children and ignoring their state... " << std::endl;

                // Ticking the remaining children (ignoring their states)
                for(i = 0; i<M; i++)
                {
                    if (ChildStatesUpdated[i] == true)
                    {
                        continue;
                    }

                    // ticking it;
                    ChildNodes[i]->tick_engine.tick();

                    // state sync;
                    ChildStates[i] = ChildNodes[i]->GetNodeState();
                }
            }
            else if (StateUpdate == false)
            {
                // Returning running!
                SetNodeState(BT::RUNNING);
                std::cout << get_name() << " returning Running! " << std::endl;
                StateUpdate = true;
            }
        }
        else
        {
            // If it was halted, all the "busy" children must be halted too
            std::cout << get_name() << " halted! Halting all the children..." << std::endl;

     /*       for(i=0; i<M; i++)
            {
                if (ChildNodes[i]->get_type() != Action && ChildStates[i] == Running)
                {
                    // if the control node was running:
                    // halting it;
                    ChildNodes[i]->Halt();

                    // sync with it (it's waiting on the semaphore);
                    ChildNodes[i]->tick_engine.tick();

                    std::cout << name << " halting child number " << i << "!" << std::endl;
                }
                else if (ChildNodes[i]->get_type() == Action && ChildNodes[i]->ReadState() == Running)
                {
                    std::cout << name << " trying halting child number " << i << "..." << std::endl;

                    // if it's a action node that hasn't finished its job:
                    // trying to halt it:
                    if (ChildNodes[i]->Halt() == false)
                    {
                        // this means that, before this node could set its child state
                        // to "Halted", the child had already written the action outcome;
                        // sync with him ignoring its state;
                        ChildNodes[i]->tick_engine.tick();

                        std::cout << name << " halting of child number " << i << " failed!" << std::endl;
                    }
                    else
                    {
                        std::cout << name << " halting of child number " << i << " succedeed!" << std::endl;
                    }
                }
                else if (ChildNodes[i]->get_type() == Action && ChildNodes[i]->ReadState() != Idle)
                {
                    // if it's a action node that has finished its job:
                    // ticking it without saving its returning state;
                    ChildNodes[i]->tick_engine.tick();
                }

                // updating its vector cell
                ChildStates[i] = Idle;
            }
*/
            HaltChildren(0);
            // Resetting the node state
            WriteState(BT::IDLE);
        }
    }
}


int BT::ParallelNode::DrawType()
{
    // Lock acquistion

    return BT::PARALLEL;
}
