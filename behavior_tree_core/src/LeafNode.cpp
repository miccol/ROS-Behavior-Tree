#include <LeafNode.h>


BT::LeafNode::LeafNode(std::string Name) : TreeNode(Name) {}

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
