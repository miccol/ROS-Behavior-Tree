NEWS!
-----------


<a href="https://btirai.github.io"><img width="200" align="left" url= "https://btirai.github.io/" src="http://michelecolledanchise.com/wp-content/uploads/2018/05/btbook.jpg"></a>

Our book **Behavior Trees in Robotics and AI**, published by CRC Press Taylor & Francis, is available for purchase (ebook and hardcover) on the CRC Press Store or Amazon. The Preprint version (**free**) is available here: https://arxiv.org/abs/1709.00084<br><br>
**Tutorials** available at https://btirai.github.io/



<br><br><br><br><br><br><br><br><br><br>

-----------


NOTE:
------
The [YARP version](https://github.com/miccol/YARP-Behavior-Trees) of this library has a GUI as the following:
![alt tag](https://github.com/miccol/YARP-Behavior-Trees/blob/master/YARPBTRun.JPG)


ROS-Behavior-Tree ![License MIT](https://img.shields.io/dub/l/vibe-d.svg)
====
![Version](https://img.shields.io/badge/version-v1.3-orange.svg) <br/> 
A ROS behavior tree library. The leaf nodes (user defined) can be either in `C++` or `python`. Read the user manual for more information.

REFERENCE
------------
Please refer to the following paper when using the library:

**How Behavior Trees Modularize Hybrid Control Systems and Generalize Sequential Behavior Compositions, the Subsumption Architecture, and Decision Trees.** Michele Colledanchise and Petter Ogren. IEEE Transaction on Robotics 2017.

bibtex entry:

`@ARTICLE{TRO17Colledanchise,` <br/>
`author={M. Colledanchise and P. Ögren},` <br/>
`journal={IEEE Transactions on Robotics},` <br/>
`title={{How Behavior Trees Modularize Hybrid Control Systems and Generalize Sequential Behavior Compositions, the Subsumption Architecture, and Decision Trees}},` <br/> 
`year={2017},` <br/>
`volume={33},` <br/>
`number={2},` <br/>
`pages={372-389},` <br/>
`keywords={Computer architecture;Decision trees;High definition video;Robot control;Switches;Behavior trees (BTs);decision trees;finite state machines (FSMs);hybrid dynamical systems (HDSs);modularity;sequential behavior compositions;subsumption architecture}, ` <br/>
`doi={10.1109/TRO.2016.2633567},` <br/>
`ISSN={1552-3098},` <br/>
`month={April},}`<br/>

INFO
------------
Contains 2 packages: behavior_tree_core and behavior_tree_leaves.

behavior_tree_core: Contains the core BT source code, including the tree and the leaf nodes.

behavior_tree_leaves: Contains action and condition specifications for BT leaf nodes **running as external ROS nodes**.

User manual available in the project folder (BTUserManual.pdf):




BUILD STATUS
------------

<table align="center">
  <tr>
    <th width="9%" />
    <th width="13%">Hydro</th>
    <th width="13%">Indigo</th>
    <th width="13%">Jade</th>
    <th width="13%">Kinetic</th>
</tr>
    <td><b>Release</b></td>
    <td align="center">
      <img src="http://build.ros.org/view/Jdev/job/Jdev__behavior_tree__ubuntu_trusty_amd64/badge/icon"/>
    </td>
    <td align="center">
      <img src="http://build.ros.org/view/Jdev/job/Jdev__behavior_tree__ubuntu_trusty_amd64/badge/icon"/>
    </td>
    <td align="center">
      <img src="http://build.ros.org/view/Jdev/job/Jdev__behavior_tree__ubuntu_trusty_amd64/badge/icon"/>
    </td>
    <td align="center">
      <img src="http://build.ros.org/view/Jdev/job/Jdev__behavior_tree__ubuntu_trusty_amd64/badge/icon"/>
    </td>
</tr>
</table>

DEPENDENCIES
------------

Regarding visualization purposes:
* [Opengl](https://www.opengl.org/)
* [Glut](https://www.opengl.org/resources/libraries/glut/)
* [xdot](https://github.com/jbohren/xdot): For visualizing using DOT language.
* [rqt_dot](https://github.com/jbohren/rqt_dot): For visualizing the tree in RQT with DOT language.

Regarding unit tests:
* [GTest](https://github.com/google/googletest)

BT NODES SUPPORT
----------------
**Selector:** Selector nodes are used to find and execute the first child that does not fail. A Selector node will return immediately with a status code of success or running when one of its children returns success or running. The children are ticked in order of importance, from `left` to `right`.

**Sequence:** Sequence nodes are used to find and execute the first child that has not yet succeeded. A sequence node will return immediately with a status code of `failure` or `running` when one of its children returns failure or running. The children are ticked in order, from `left` to `right`.

**Parallel:** The parallel node ticks its children in parallel and returns success if `M ≤ N` children return success, it returns failure if `N − M + 1` children return failure, and it returns running otherwise.

**Decorator:** The decorator node manipulates the return status of its child according to the policy defined by the user (e.g. it inverts the success/failure status of the child). In this library the decorators implemented are the two common ones: *Decorator Retry* which retries the execution of a node if this fails; and *Decorator Negation* That inverts the Success/Failure outcome.

**Action:** An Action node performs an action, and returns Success if the action is completed, Failure if it can not be completed and Running if completion is under way.

**Condition:** A Condition node determines if a desired condition `c` has been met. Conditions are technically a subset of the Actions, but are given a separate category and graphical symbol to improve readability of the BT and emphasize the fact that they never return running and do not change any internal states/variables of the BT.





SETUP
-----------
**USER MANUAL available inside the repo's folder**

The first step to use BT++ is to retrieve its source code. You can either download it 
here (https://github.com/miccol/ROS-Behavior-Tree) or clone the repository:

`$ cd /path/to/catkin_ws/src` <br/>
`$ git clone https://github.com/miccol/ROS-Behavior-Tree.git`<br/>

Once you have the repository. Compile the library:

`$ cd /path/to/catkin_ws/` <br/>
`$ catkin_make` <br/>

Check the installation by launching an example.

`$ roslaunch behavior_tree_leaves test_behavior_tree.launch` <br/>

Run `rqt_dot` plugin for the visualization in ROS and put the ROS topic in
which the tree is published. The default topic is `/bt_dotcode`.

```
rosrun rqt_dot rqt_dot
```
NOTES
-------
In case you are puzzled about why a sequence (or fallback) node with 2 or more actions as children never get past the first action, see [this](https://github.com/miccol/ROS-Behavior-Tree/issues/16) discussion.

LICENSE
-------
The MIT License (MIT)

Copyright (c) 2014-2018 Michele Colledanchise

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
