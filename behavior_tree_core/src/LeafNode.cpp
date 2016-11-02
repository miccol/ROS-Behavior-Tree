#include <LeafNode.h>


BT::LeafNode::LeafNode(std::string name) : TreeNode(name) {}

BT::LeafNode::~LeafNode() {}


void BT::LeafNode::ResetColorState()
{
    // Lock acquistion

    ColorState = BT::Idle;
}

int BT::LeafNode::GetDepth()
{
    return 0;
}
