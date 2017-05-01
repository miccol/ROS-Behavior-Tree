/* Copyright (C) 2017 Iason Sarantopoulos - All Rights Reserved
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions: The
 * above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <dot_bt.h>
#include <control_node.h>
#include <std_msgs/String.h>

namespace BT
{
DotBt::DotBt(TreeNode* root, const std::string& topic, double ros_rate) :
  root_(root),
  topic_(topic),
  loop_rate_(ros_rate)
{
  dotbt_publisher_ = n_.advertise<std_msgs::String>(topic_, 1);
}

DotBt::~DotBt() {}

std::string DotBt::defineNodeDot(TreeNode* node)
{
  std::string output;
  output = getAlias(node->get_name()) + " ";
  //output = "temp_node ";

  switch (node->DrawType())
  {
    case SELECTORSTAR:
      output += "[label=\"?*\" shape=\"box\"];";
      break;
    case BT::SEQUENCESTAR:
      output += "[label=\">*\" shape=\"box\"];";
      break;
    case BT::SELECTOR:
      output += "[label=\"?\" shape=\"box\"];";
      break;
    case BT::SEQUENCE:
      output += "[label=\">\" shape=\"box\"];";
      break;
    case BT::PARALLEL:
      output += "[label=\"=>\" shape=\"box\"];";
      break;
    case BT::DECORATOR:
      output += "[label=\"D\" shape=\"diamond\"];";
      break;
    case BT::ACTION:
      output += "[label=\"" + node->get_name() + "\" shape=\"box\" fillcolor=\"green\" style=\"filled\"];";
      break;
    case BT::CONDITION:
      output += "[label=\"" + node->get_name() + "\" shape=\"ellipse\" fillcolor=\"green\" style=\"filled\"];";
      break;
    default:
      break;
  }
  return output;
}

void DotBt::produceDot(TreeNode* node, TreeNode* parent)
{
  if (parent == NULL)
  {
    dot_file_ = "digraph behavior_tree {\n";
  }

  dot_file_ += defineNodeDot(node) + "\n";

  if (parent != NULL)
  {
    dot_file_ += getAlias(parent->get_name()) + " -> " + getAlias(node->get_name()) + ";\n";
  }

  BT::ControlNode* n = dynamic_cast<BT::ControlNode*> (node);
  if (n != NULL)
  {
    std::vector<TreeNode *> children = n->GetChildren();
    for (unsigned int i = 0; i < children.size(); i++)
    {
      produceDot(children.at(i), node);
    }
  }

  if (parent == NULL)
  {
    dot_file_ += "\n}";
  }
}

std::string DotBt::getAlias(const std::string &name)
{
  // Transform name to lower case
  std::string out = boost::to_lower_copy<std::string>(name);

  // Replace spaces with underscore
  for (std::string::iterator it = out.begin(); it != out.end(); ++it)
  {
    if (*it == ' ')
    {
      *it = '_';
    }
  }
  return out;
}

std::string DotBt::getDotFile()
{
  return dot_file_;
}

void DotBt::publish()
{
  std_msgs::String msg;
  while (ros::ok())
  {
    produceDot(root_);
    msg.data = dot_file_;
    dotbt_publisher_.publish(msg);
    ros::spinOnce();
    loop_rate_.sleep();
  }
}
}  // namespace BT
