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

#ifndef DOT_BT_H
#define DOT_BT_H

#include <string>
#include <tree_node.h>
#include <boost/algorithm/string.hpp>

namespace BT
{

class DotBt
{
public:
  explicit DotBt();
  ~DotBt();
  void produceDot(TreeNode* node, TreeNode* parent = NULL);
  std::string getDotFile();
private:
  std::string defineNodeDot(TreeNode* node);

  /**
   * @brief Returns the alias of a node.
   *
   * In general tranforms a string to lower case and replace the space with
   * underscores. E.g. the string "My String" will be returned as "my_string".
   *
   * @param name The initial string as input
   * @returns The final string as output.
   */
  std::string getAlias(const std::string &name);

  std::string dot_file_;
};
}  // namespace BT

#endif  // DOT_BT_H
