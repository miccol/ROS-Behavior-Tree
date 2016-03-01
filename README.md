ROS-Behavior-Tree
Metapackage for BT implementation in ROS. Contains 2 packages: behavior_tree_core and behavior_tree_leaves.

behavior_tree_core: Contains the core BT source code.

behavior_tree_leaves: Contains action and condition specifications for BT leaf nodes

User manual available in the project folder.


Installation instructions:

$cd your/catkin/workspace/src
$git clone https://github.com/miccol/ROS-Behavior-Tree.git
$mv Behavior−Tree behavior_tree
$cd ..
$catkin_make

Check the installation by running a BT of test:
$ roslaunch behavior_tree_core test_behavior_tree.launch
