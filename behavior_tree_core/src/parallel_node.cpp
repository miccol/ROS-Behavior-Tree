#include <parallel_node.h>


BT::ParallelNode::ParallelNode(std::string name) : ControlNode::ControlNode(name)
{
    // Initializations
    threshold_ = std::numeric_limits<unsigned int>::max();
    is_state_updated_ = false;

    // thread_ start
    thread_ = boost::thread(&ParallelNode::Exec, this);
}

BT::ParallelNode::~ParallelNode() {}

void BT::ParallelNode::SetThreshold(unsigned int new_threshold)
{
    // Vector size initialization
    N_of_children_ = children_nodes_.size();

    // Checking threshold_ correctness
    if (threshold_ > N_of_children_)
    {
        std::stringstream S;
        S << "Wrong threshold_ threshold for '" << get_name() << "'. N_of_children_=" << N_of_children_ << " while threshold_=" << threshold_ << ". threshold_ should be <= M.";
        throw BehaviorTreeException(S.str());
    }

    threshold_ = new_threshold;

    // First tick for the thread
    tick_engine.tick();
}

void BT::ParallelNode::Exec()
{
    unsigned int i;

    // Waiting for a first tick to come
    tick_engine.wait();

    // Checking threshold_ correctness
    if (threshold_ == std::numeric_limits<unsigned int>::max())
    {
        throw BehaviorTreeException("'" + get_name() + "' has no valid threshold_ threashold set. You should set it before ticking the node.");
    }

    // Vector construction
    for (i=0; i < N_of_children_; i++)
    {
        is_children_states_updated_vtr_.push_back(false);
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
        is_state_updated_ = false;
        n_of_successes_ = n_of_failures_ = n_of_runnings_ = 0;
        for (i = 0; i < N_of_children_; i++)
        {
            is_children_states_updated_vtr_[i] = false;
        }

        // Checking if i was halted
        if (ReadState() != BT::HALTED)
        {
            std::cout << get_name() << " ticked, ticking actions..." << std::endl;

            // If not, before ticking the children, checking if a state can
            // be immediatelly returned.
            for(i = 0; i<N_of_children_; i++)
            {
                if (children_nodes_[i]->get_type() != BT::ACTION_NODE)
                {
                    continue;
                }

                children_states_[i] = children_nodes_[i]->ReadState();

                if (children_states_[i] == BT::SUCCESS)
                {
                    // The action node has finished, sync
                    children_nodes_[i]->tick_engine.tick();

                    n_of_successes_++;
                    is_children_states_updated_vtr_[i] = true;
                }
                else if (children_states_[i] == BT::FAILURE)
                {
                    // The action node has finished, sync
                    children_nodes_[i]->tick_engine.tick();

                    n_of_failures_++;
                    is_children_states_updated_vtr_[i] = true;
                }
                else if (children_states_[i] == BT::RUNNING)
                {
                    n_of_runnings_++;
                    is_children_states_updated_vtr_[i] = true;
                }

                if (n_of_successes_ >= threshold_)
                {
                    // Returning success
                    SetNodeState(BT::SUCCESS);
                    std::cout << get_name() << " returning Success! " << std::endl;
                    is_state_updated_ = true;

                    // Exit the for loop
                    break;
                }
                else if (n_of_failures_ > N_of_children_ - threshold_)
                {
                    // Returning failure
                    SetNodeState(BT::FAILURE);
                    std::cout << get_name() << " returning Failure! " << std::endl;
                    is_state_updated_ = true;

                    // Exit the for loop
                    break;
                }
                else if (n_of_runnings_ > N_of_children_ - threshold_ && n_of_runnings_ >= threshold_)
                {
                    // Neither a Success nor a Failure could be returned
                    // Returning Running
                    SetNodeState(BT::RUNNING);
                    std::cout << get_name() << " returning Running! " << std::endl;
                    is_state_updated_ = true;

                    // Exit the for loop
                    break;
                }
            }

            if (is_state_updated_ == true)
            {
                // If it is known what to return...
                std::cout << get_name() << " knows what to return... " << std::endl;

                if (ReadState() == BT::SUCCESS || ReadState() == BT::FAILURE)
                {
                    // Halting the running actions
                    for (i=0; i<N_of_children_; i++)
                    {
                        if (is_children_states_updated_vtr_[i] == false || children_states_[i] != BT::RUNNING)
                        {
                            continue;
                        }

                        std::cout << get_name() << " trying halting (action) child number " << i << "..." << std::endl;

                        if (children_nodes_[i]->Halt() == false)
                        {
                            // this means that, before this node could set its child state
                            // to "Halted", the child had already written the action outcome;
                            // sync with him ignoring its state;
                            children_nodes_[i]->tick_engine.tick();

                            std::cout << get_name() << " halting of child number " << i << " failed!" << std::endl;
                        }
                        else
                        {
                            std::cout << get_name() << " halting of child number " << i << " succedeed!" << std::endl;
                        }

                        // updating its vector cell;
                        children_states_[i] = BT::IDLE;
                    }

                    // Ticking the other children, but halting them if they
                    // return Running.
                    std::cout << get_name() << " ticking the remaining children... " << std::endl;

                    for(i = 0; i<N_of_children_; i++)
                    {
                        if (is_children_states_updated_vtr_[i] == true)
                        {
                            continue;
                        }

                        // ticking it;
                        children_nodes_[i]->tick_engine.tick();

                        // retrive its state as soon as it is available;
                        children_states_[i] = children_nodes_[i]->GetNodeState();
                        if (children_states_[i] == BT::RUNNING)
                        {
                            std::cout << get_name() << " halting (control) child number " << i << "..." << std::endl;

                            // if it is running, halting it;
                            children_nodes_[i]->Halt();

                            // sync with it (it's waiting on the semaphore);
                            children_nodes_[i]->tick_engine.tick();
                        }

                        // updating its vector cell;
                        children_states_[i] = BT::IDLE;
                        is_children_states_updated_vtr_[i] = true;
                    }

                    // Resetting the node state
                    if (ReadState() != BT::HALTED)
                    {
                        WriteState(BT::IDLE);
                    }

                    // Next cicle
                    continue;
                }
                else if (n_of_runnings_ + n_of_failures_ + n_of_successes_ < N_of_children_)
                {
                    // Returning running! But some children haven't been ticked yet!

                    std::cout << get_name() << " ticking the remaining children and ignoring their state... " << std::endl;

                    // Ticking the remaining children (ignoring their states)
                    for(i = 0; i<N_of_children_; i++)
                    {
                        if (is_children_states_updated_vtr_[i] == true)
                        {
                            continue;
                        }

                        if (children_nodes_[i]->get_type() == BT::ACTION_NODE)
                        {
                            // if it's an action:
                            // read its state;
                            ReturnStatus ActionState = children_nodes_[i]->ReadState();

                            if (ActionState == BT::IDLE)
                            {
                                // if it's "Idle":
                                // ticking it;
                                children_nodes_[i]->tick_engine.tick();

                                // retriving its state as soon as it is available;
                                children_states_[i] = children_nodes_[i]->GetNodeState();
                            }
                            else if (ActionState == BT::RUNNING)
                            {
                                // It's ok, it can continue running
                                children_states_[i] = BT::RUNNING;
                            }
                            else
                            {
                                // if it's "Success" of "Failure" (it can't be "Halted"!):
                                // sync with it;
                                children_nodes_[i]->tick_engine.tick();

                                // ticking it;
                                children_nodes_[i]->tick_engine.tick();

                                // retriving its state as soon as it is available;
                                children_states_[i] = children_nodes_[i]->GetNodeState();
                            }
                        }
                        else
                        {
                            // if it's not an action:
                            // ticking it;
                            children_nodes_[i]->tick_engine.tick();

                            // retrive its state as soon as it is available;
                            children_states_[i] = children_nodes_[i]->GetNodeState();
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
            for(i = 0; i<N_of_children_; i++)
            {
                if (is_children_states_updated_vtr_[i] == true)
                {
                    continue;
                }

                // ticking it;
                children_nodes_[i]->tick_engine.tick();

                // retrive its state as soon as it is available;
                children_states_[i] = children_nodes_[i]->GetNodeState();

                if (children_states_[i] == BT::SUCCESS)
                {
                    n_of_successes_++;
                    is_children_states_updated_vtr_[i] = true;
                }
                else if (children_states_[i] == BT::FAILURE)
                {
                    n_of_failures_++;
                    is_children_states_updated_vtr_[i] = true;
                }
                else
                {
                    n_of_runnings_++;
                    is_children_states_updated_vtr_[i] = true;
                }

                if (n_of_successes_ >= threshold_)
                {
                    // Returning success
                    SetNodeState(BT::SUCCESS);
                    std::cout << get_name() << " returning Success! " << std::endl;
                    is_state_updated_ = true;

                    // Exit the for loop
                    break;
                }
                else if (n_of_failures_ > N_of_children_ - threshold_)
                {
                    // Returning failure
                    SetNodeState(BT::FAILURE);
                    std::cout << get_name() << " returning Failure! " << std::endl;
                    is_state_updated_ = true;

                    // Exit the for loop
                    break;
                }
                else if (n_of_runnings_ > N_of_children_ - threshold_ && n_of_runnings_ >= threshold_)
                {
                    // Neither a Success nor a Failure could be returned
                    // Returning Running
                    SetNodeState(BT::RUNNING);
                    std::cout << get_name() << " returning Running! " << std::endl;
                    is_state_updated_ = true;

                    // Exit the for loop
                    break;
                }
            }

            if (is_state_updated_ == true && ReadState() != BT::RUNNING)
            {
                std::cout << get_name() << " knows what to return... " << std::endl;

                // Halting all the running nodes (that have been ticked!)
                for (i=0; i<N_of_children_; i++)
                {
                    if (is_children_states_updated_vtr_[i] == false || children_states_[i] != BT::RUNNING)
                    {
                        continue;
                    }

                    if (children_nodes_[i]->get_type() == BT::ACTION_NODE)
                    {
                        std::cout << get_name() << " trying halting (action) child number " << i << "..." << std::endl;

                        if (children_nodes_[i]->Halt() == false)
                        {
                            // this means that, before this node could set its child state
                            // to "Halted", the child had already written the action outcome;
                            // sync with him ignoring its state;
                            children_nodes_[i]->tick_engine.tick();

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
                        children_nodes_[i]->Halt();

                        // sync with it (it's waiting on the semaphore);
                        children_nodes_[i]->tick_engine.tick();

                        std::cout << get_name() << " halting child number " << i << "!" << std::endl;
                    }

                    // updating its vector cell;
                    children_states_[i] = BT::IDLE;
                }

                // Ticking the other children, but halting them is they
                // return Running.
                std::cout << get_name() << " ticking the remaining children... " << std::endl;

                for(i = 0; i<N_of_children_; i++)
                {
                    if (is_children_states_updated_vtr_[i] == true)
                    {
                        continue;
                    }

                    // ticking it;
                    children_nodes_[i]->tick_engine.tick();

                    // retrive its state as soon as it is available
                    children_states_[i] = children_nodes_[i]->GetNodeState();

                    if (children_states_[i] == BT::RUNNING)
                    {
                        std::cout << get_name() << " halting (control) child number " << i << "..." << std::endl;

                        // if it is running, halting it;
                        children_nodes_[i]->Halt();

                        // sync with it (it's waiting on the semaphore);
                        children_nodes_[i]->tick_engine.tick();

                        // updating its vector cell;
                        children_states_[i] = BT::IDLE;
                    }
                }

                // Resetting the node state
                if (ReadState() != BT::HALTED)
                {
                    WriteState(BT::IDLE);
                }
            }
            else if (is_state_updated_ == true && n_of_runnings_ + n_of_failures_ + n_of_successes_ < N_of_children_)
            {
                // Returning running, but there still children to be ticked

                std::cout << get_name() << " ticking the remaining children and ignoring their state... " << std::endl;

                // Ticking the remaining children (ignoring their states)
                for(i = 0; i<N_of_children_; i++)
                {
                    if (is_children_states_updated_vtr_[i] == true)
                    {
                        continue;
                    }

                    // ticking it;
                    children_nodes_[i]->tick_engine.tick();

                    // state sync;
                    children_states_[i] = children_nodes_[i]->GetNodeState();
                }
            }
            else if (is_state_updated_ == false)
            {
                // Returning running!
                SetNodeState(BT::RUNNING);
                std::cout << get_name() << " returning Running! " << std::endl;
                is_state_updated_ = true;
            }
        }
        else
        {
            // If it was halted, all the "busy" children must be halted too
            std::cout << get_name() << " halted! Halting all the children..." << std::endl;

     /*       for(i=0; i<M; i++)
            {
                if (children_nodes_[i]->get_type() != Action && children_states_[i] == Running)
                {
                    // if the control node was running:
                    // halting it;
                    children_nodes_[i]->Halt();

                    // sync with it (it's waiting on the semaphore);
                    children_nodes_[i]->tick_engine.tick();

                    std::cout << name << " halting child number " << i << "!" << std::endl;
                }
                else if (children_nodes_[i]->get_type() == Action && children_nodes_[i]->ReadState() == Running)
                {
                    std::cout << name << " trying halting child number " << i << "..." << std::endl;

                    // if it's a action node that hasn't finished its job:
                    // trying to halt it:
                    if (children_nodes_[i]->Halt() == false)
                    {
                        // this means that, before this node could set its child state
                        // to "Halted", the child had already written the action outcome;
                        // sync with him ignoring its state;
                        children_nodes_[i]->tick_engine.tick();

                        std::cout << name << " halting of child number " << i << " failed!" << std::endl;
                    }
                    else
                    {
                        std::cout << name << " halting of child number " << i << " succedeed!" << std::endl;
                    }
                }
                else if (children_nodes_[i]->get_type() == Action && children_nodes_[i]->ReadState() != Idle)
                {
                    // if it's a action node that has finished its job:
                    // ticking it without saving its returning state;
                    children_nodes_[i]->tick_engine.tick();
                }

                // updating its vector cell
                children_states_[i] = Idle;
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
