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
#include <cctype>
#include <algorithm>

namespace BT
{
DotBt::DotBt(TreeNode* root, const std::string& topic, double ros_rate, bool left_right, bool multiple_parents) :
  root_(root),
  topic_(topic),
  loop_rate_(ros_rate),
  left_right_(left_right),
  multiple_parents_(multiple_parents)
{
  dotbt_publisher_ = n_.advertise<std_msgs::String>(topic_, 1);

  ROS_INFO_STREAM("Visualization: Start publishing the tree in topic: "
      << topic_ << " with rate: " << ros_rate << " Hz.");
}

DotBt::~DotBt() {}

std::string DotBt::defineNodeDot(TreeNode* node, const std::string& alias)
{
  std::string output = alias + " ";

  // Find the type of the node and its shape and symbol (label).
  switch (node->DrawType())
  {
    case SELECTORSTAR:
      output += "[label=\"*\n?\" penwidth=\"2\"  shape=\"box\"";
      break;
    case BT::SEQUENCESTAR:
      output += "[label=\"*\n-->\" penwidth=\"2\"  shape=\"box\"";
      break;
    case BT::SELECTOR:
      output += "[label=\"?\" penwidth=\"2\"  shape=\"box\"";
      break;
    case BT::SEQUENCE:
      output += "[label=\"-->\" penwidth=\"2\"  shape=\"box\"";
      break;
    case BT::PARALLEL:
      output += "[label=\"-->\n-->\" penwidth=\"2\"  shape=\"box\"";
      break;
    case BT::DECORATOR:
      output += "[label=\"D\" penwidth=\"2\" shape=\"diamond\"";
      break;
    case BT::ACTION:
      output += "[label=\"" + node->get_name() + "\" penwidth=\"2\" shape=\"box\" fillcolor=\"palegreen\" style=\"filled\"";
      break;
    case BT::CONDITION:
      output += "[label=\"" + node->get_name() + "\" penwidth=\"2\" shape=\"ellipse\" fillcolor=\"khaki1\" style=\"filled\"";
      break;
    default:
      break;
  }

  // Get the current status of the node for the coloring.
  switch (node->get_color_status())
  {
    case BT::RUNNING:
      output += " color=\"black\" ];";
      break;
    case BT::SUCCESS:
      output += " color=\"green\" ];";
      break;
    case BT::FAILURE:
      output += " color=\"red\" ];";
      break;
    case BT::IDLE:
      output += " color=\"gray88\" ];";
      break;
    case BT::HALTED:
      output += " color=\"orange\" ];";
      break;
    default:
      output += " color=\"gray88\" ];";
      break;
  }

  return output;
}

void DotBt::produceDot(TreeNode* node, TreeNode* parent, const std::string& parent_alias)
{
  // If this node is the root of the tree initialize the directed graph
  if (parent == NULL)
  {
    dot_file_ = "graph behavior_tree {\n";
    if (left_right_)
    {
      dot_file_ += "rankdir=LR;\n";
    }

    if (!multiple_parents_)
    {
      aliases_.clear();
    }
  }

  // Create an alias for naming the DOT object.
  std::string alias = getAlias(node->get_name());

  // Search if this alias has . In this case change the alias in order to use a
  // different visualization instance for this case.


  if (std::find(aliases_.begin(), aliases_.end(), alias) != aliases_.end())
  {
    alias += std::to_string(multiple_alias_solver_++);
  }
    aliases_.push_back(alias);


  // Add the definition of this node
  dot_file_ += defineNodeDot(node, alias) + "\n";

  // If the node has a parent, add it as a child of its parent.
  if (parent != NULL)
  {
    dot_file_ += parent_alias + " -- " + alias + ";\n";
  }

  // If this node has children run recursively for each child.
  BT::ControlNode* n = dynamic_cast<BT::ControlNode*> (node);
  if (n != NULL)
  {
    std::vector<TreeNode *> children = n->GetChildren();
    for (unsigned int i = 0; i < children.size(); i++)
    {

//        if (children[i]->has_alias())
//        {
//            // cheking if I need to halt the child (has alias)
//            for (unsigned int k=0; k < i; k++)
//            {
//                if (children[k] == children[i] && children[k]->get_status() == BT::HALTED )
//                {
//                }
//                else
//                {
//                    color_child = false;
//                    break;
//                }
//            }
//        }



      produceDot(children.at(i), node, alias);
    }
  }

  // In case every recursive calls returns to the root call, close the file.
  if (parent == NULL)
  {
    dot_file_ += "\n}";
  }
}

std::string DotBt::getAlias(const std::string &name)
{
  // Transform name to lower case
  std::string out = boost::to_lower_copy<std::string>(name);

  // If first character is digit add a letter at the beginning
  // in order to avoid weird aliases
  if (std::isdigit(out.at(0)))
  {
    out.insert(0, "a");
  }


  // Replace spaces and dashes with underscore
  for (std::string::iterator it = out.begin(); it != out.end(); ++it)
  {
    if (*it == ' ' || *it == '-')
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

  // Start the loop for publishing the tree
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
