#include <LeafNode.h>


BT::LeafNode::LeafNode(std::string name) : TreeNode(name) {}

BT::LeafNode::~LeafNode() {}


void BT::LeafNode::ResetColorState()
{
    // Lock acquistion

    color_state_ = BT::IDLE;
}

int BT::LeafNode::Depth()
{
    return 0;
}
