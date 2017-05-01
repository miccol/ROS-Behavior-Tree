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

namespace BT
{
DotBt::DotBt() {}
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
      output += "[label=\"" + node->get_name() + "\" shape=\"box\"];";
      break;
    case BT::CONDITION:
      output += "[label=\"" + node->get_name() + "\" shape=\"ellipse\"];";
      break;
    default:
      break;
  }
  return output;
}

void DotBt::produceDot(TreeNode* node)
{
  std::string dot_file;
  dot_file = "digraph behavior_tree {\n";
  dot_file += defineNodeDot(node);
 
  dot_file += "\n}";
 std::cout << dot_file << std::endl;
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
}  // namespace BT
