/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <decorator_node.h>
#include <string>

BT::DecoratorNode::DecoratorNode(std::string name) : ControlNode::ControlNode(name) {}

BT::DecoratorNode::~DecoratorNode() {}

void BT::DecoratorNode::AddChild(TreeNode* child)
{
    // Verify if the decoretor doesn't already has a child
    if (children_nodes_.size() > 0) 
    {
        throw BehaviorTreeException("'" + get_name() + " has a child already. A decorator is not capable of handling more than one child node.");
        return;
    }

    child->set_has_parent(true);
    children_nodes_.push_back(child);
    children_states_.push_back(BT::IDLE);
}

BT::ReturnStatus BT::DecoratorNode::Tick()
{
    {
        // gets the number of children. The number could change if, at runtime, one edits the tree.
        N_of_children_ = children_nodes_.size();

        if (N_of_children_ > 0)
        {
            if (children_nodes_[0]->get_type() == BT::ACTION_NODE)
            {
                child_i_status_ = children_nodes_[0]->get_status();

                if (child_i_status_ == BT::IDLE || child_i_status_ == BT::HALTED)
                {
                    DEBUG_STDOUT(get_name() << "NEEDS TO TICK " << children_nodes_[0]->get_name());
                    children_nodes_[0]->tick_engine.Tick();

                    // waits for the tick to arrive to the child
                    do
                    {
                        child_i_status_ = children_nodes_[0]->get_status();
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                    while (child_i_status_ != BT::RUNNING && child_i_status_ != BT::SUCCESS && child_i_status_ != BT::FAILURE);
                }
            }
            else 
            {
                child_i_status_ = children_nodes_[0]->Tick();
            }

            if (child_i_status_ == BT::SUCCESS || child_i_status_ == BT::FAILURE)
            {
                children_nodes_[0]->set_status(BT::IDLE);
            }

            if (child_i_status_ != BT::FAILURE)
            {
                HaltChildren(1);
            }

            set_status(child_i_status_);
            return child_i_status_;
        } 
    }
    return BT::EXIT;
}

int BT::DecoratorNode::DrawType()
{
    // Lock acquistion

    return BT::DECORATOR;
}
