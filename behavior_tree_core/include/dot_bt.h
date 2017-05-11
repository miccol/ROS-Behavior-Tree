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
#include <ros/ros.h>
#include <tree_node.h>
#include <boost/algorithm/string.hpp>
#include <vector>

namespace BT
{
/**
 * @brief Provides tools for translate a BT in DOT and publishing it to ROS for
 * visualization in RQT.
 *
 * The class generates code of the DOT graph description language which
 * describes the current BT running. It also provide ROS publisher for
 * publishing this code in a ROS topic. Then, the user can use the rqt_dot
 * plugin in order to visualize in real-time the current tree and the status of
 * each node. Regarding multi-parenting this class by default visualize the
 * node multiple times under its different parents, in order to have a
 * straight-forward visualization. Notice that in the implementation level the
 * same node will be ticked. The class also provides the option to visualize
 * the nodes with the same name as one node and the nodes with multiple parents
 * as without replication (in order to detect visually these undesired
 * replications).
 *
 * Find below an example of use:
 *
 * @code{.cpp}
 * #include <dot_bt.h>
 * #include <thread>
 *
 * // ...
 *
 * // Assume root is a pointer TreeNode* to the root of your tree
 * std::string topic = "/my_topic" // The ROS topic to publish the tree
 * double rate = 50; // The rate of publishing in Hz
 * BT::DotBt dot_bt(root, topic, rate);  // Call the constructor
 * std::thread t(&BT::DotBt::publish, dot_bt); // A separate thread publishes the tree
 * @endcode
 */
class DotBt
{
public:
  /**
   * @brief The default constructor.
   *
   * @param root A pointer to the root of the tree.
   * @param topic The name of the ROS topic to publish the tree. Defaults to
   * "/bt_dotcode".
   * @param ros_rate The rate of the publishing in Hz. Defaults to 50Hz.
   * @param left_right Set true if a left to right visualization is desired.
   * Defaults to true, i.e. top-down visualization
   * @param multiple_parents Set true if it is desired to visualize nodes with
   * multiple parents (or nodes with the same name) without duplication. It is
   * recommended to use the default (false) value for better results.
   */
  explicit DotBt(TreeNode* root,
                 const std::string& topic = "/bt_dotcode",
                 double ros_rate = 50,
                 bool left_right = false,
                 bool multiple_parents = false);

  /**
   * @brief An empty destructor.
   */
  ~DotBt();

  /**
   * @brief Returns the current DOT code produced for the BT.
   *
   * @returns The current DOT code
   */
  std::string getDotFile();

  /**
   * @brief Publishes the tree for visualization.
   *
   * This is the main API of the class. It publishes in the given topic with
   * the given name the produced DOT code for the current BT running.
   * Run it in a separate thread in your BT application.
   */
  void publish();
private:
  /**
   * @brief Produces DOT code for the tree recursively.
   *
   * Initially defines the current node calling DotBt::produceDot and then if
   * the current node has a parent produces the DOT code for adding this node
   * as the child of its parent. If the node has children repeats this process
   * recursively. It also checks if the produced alias of the node is already
   * existing in order to visualize with a different instance nodes with
   * multiple parents or nodes with the same name.
   *
   * @param node The current node.
   * @param parent The parent of the current node. Defaults to NULL for the
   * root of the tree.
   * @param parent_alias The alias of the parent to be used in the DOT code.
   * Defaults to empty string in case this node is the root of the tree.
   */
  void produceDot(TreeNode* node, TreeNode* parent = NULL, const std::string& parent_alias = "");

  /**
   * @brief Produces DOT code for the definition of the node.
   *
   * The current node's alias is used as the DOT object of the node. Then
   * checks the type of the node (Action, Sequence etc) and gives the node the
   * correct shape and label. Finally it checks the status of the node
   * (Running, Idle, Failed etc) in order to give the correct color to each
   * node.
   *
   * @param node A pointer to the node to be defined.
   * @param alias The alias of the given node.
   * @returns The definition of the Node in DOT
   */
  std::string defineNodeDot(TreeNode* node, const std::string& alias);

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

  /**
   * @brief Stores the DOT code of the current tree.
   */
  std::string dot_file_;

  /**
   * @brief A node handle used by the ROS publisher DotBt::dotbt_publisher_.
   */
  ros::NodeHandle n_;

  /**
   * @brief A ROS publisher for publishing DotBt::dot_file_.
   */
  ros::Publisher dotbt_publisher_;
  
  /**
   * @brief The root of the Behavior Tree.
   */
  TreeNode* root_;

  /**
   * @brief Stores the name of the topic that DotBt::dotbt_publisher_ will publish.
   */
  std::string topic_;

  /**
   * @brief The rate at which the DotBt::dotbt_publisher_ will publish the tree.
   */
  ros::Rate loop_rate_;

  /**
   * @brief Stores the aliases of each node of tree. It is used for avoiding
   * conflicts due to nodes with the same name or multiple parents.
   */
  std::vector<std::string> aliases_;

  /**
   * @brief True for left to right visualization. False for top to down.
   */
  bool left_right_;

  /**
   * @brief True if you want to visualize nodes with multiple parents without
   * duplication.
   */
  bool multiple_parents_;

  int multiple_alias_solver_;
};
}  // namespace BT

#endif  // DOT_BT_H
