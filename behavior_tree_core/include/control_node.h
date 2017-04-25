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

#ifndef CONTROL_NODE_H
#define CONTROL_NODE_H


#include <tree_node.h>
#include <vector>
#include <string>

namespace BT
{
class ControlNode : public TreeNode
{
protected:
    // Children vector
    std::vector<TreeNode*> children_nodes_;

    // Children states
    std::vector<ReturnStatus> children_states_;

    // Vector size
    unsigned int N_of_children_;
    // child i status. Used to rout the ticks
    ReturnStatus child_i_status_;

public:
    // Constructor
    explicit ControlNode(std::string name);
    ~ControlNode();

    // The method used to fill the child vector
    void AddChild(TreeNode* child);

    // The method used to know the number of children
    unsigned int GetChildrenNumber();
    std::vector<TreeNode*> GetChildren();
    // The method used to interrupt the execution of the node
    void Halt();
    void ResetColorState();
    void HaltChildren(int i);
    int Depth();

    // Methods used to access the node state without the
    // conditional waiting (only mutual access)
    bool WriteState(ReturnStatus new_state);
};
}  // namespace BT

#endif  // CONTROL_NODE_H
