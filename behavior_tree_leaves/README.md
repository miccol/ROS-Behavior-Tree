# README #
ROS package for Behavior Tree's leaf nodes (Actions and Conditions).
The package is based on the actionlib library (http://wiki.ros.org/actionlib) 


### What is this repository for? ###

This package contains the ros nodes executing actions and conditions. These nodes are clients in the actionlib library. 

### How do I get set up? ###

Download the repository 

     cd ~/catkin_ws/src
     git clone https://miccol@bitbucket.org/miccol/amazon-challenge-bt-nodes.git

(OPTIONAL) edit the folder name if you want


     mv amazon-challenge-bt-nodes/ bt_actions/


###Set up an Behavior Tree's action in C++
The file src/example_action_server.cpp is a template on how the your ROS node should look like it it performs and action (it is an action in the Behavior Tree).
Your action is the Server and does stuff. The Behavior Tree is the Client and tells to all the Server which ones have to start (TICK) and which have to stop (HALT).

Edit in example_action_server.cpp the executeCB procedure, adding your code to execute when the node is TICKED and when is HALTED. If the action is finished the node should send the return status (SUCCESS/FAILURE) calling setStatus(SUCCESS/FAILURE).

    void executeCB(const bt_actions::BTGoalConstPtr &goal)
	  {

	    // publish info to the console for the user
	    ROS_INFO("Starting Action");

	    // start executing the action
	    while(/*YOUR CONDITION*/)
	    {
	      // check that preempt has not been requested by the client
	      if (as_.isPreemptRequested() || !ros::ok())
	      {
		ROS_INFO("Action Halted");


	 /*
		    HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
	*/


		// set the action state to preempted
		as_.setPreempted();
		success = false;
		break;
	      }


	      ROS_INFO("Executing Action");
	/*
		  HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
	*/

	 //If the action succeeded
	      setStatus(SUCCESS);
	 //If the action Failed
	      setStatus(FAILURE);

	   }


Then set a name for your action. The name has the be unique. It will be used bt the behavior tree to recognize it.

       ros::init(argc, argv, /*name of your action as a std::string*/);



###Set up an Behavior Tree's condition in C++
The procedure is similar to the one above.
The file src/example_condition_server.cpp is a template on how the your ROS node should look like when it checks a condition (it is a condition in the Behavior Tree).
Your condition is the Server and checks stuff. The Behavior Tree is the Client and tells to all the Server which ones have to start (TICK) and which have to stop (HALT).

Edit in example_condition_server.cpp the executeCB procedure, adding your code to execute when the node is TICKED (it is never halted). If the condition is satisfied then the condition returns the status setStatus(SUCCESS). If the condition is not satisfied then the condition returns the status setStatus(SUCCESS). 
IMPORTANT! A condition is supposed to reply very fast. If it takes too much time (apporx more that  seconds), the node has to be defined as an action.


		  void executeCB(const bt_actions::BTGoalConstPtr &goal)
		  {
		    if(/*condition satisfied*/)
                    {
		        setStatus(SUCCESS);
		    }else{
		        setStatus(FAILURE);
		    }
		  }

Then set a name for your condition. The name has the be unique. It will be used bt the behavior tree to recognize it.

       ros::init(argc, argv, /*name of your action as a std::string*/);

###Set up an Behavior Tree's action in python
The file src/example_action_server.py is a template on how the your ROS node should look like it it performs and action (it is an action in the Behavior Tree).
Your action is the Server and does stuff. The Behavior Tree is the Client and tells to all the Server which ones have to start (TICK) and which have to stop (HALT).

Edit in example_action_server.py the execute_cb procedure, adding your code to execute when the node is TICKED and when is HALTED. If the action is finished the node should send the return status (SUCCESS/FAILURE) calling setStatus(SUCCESS/FAILURE).





    def execute_cb(self, goal):
		    # publish info to the console for the user
		    rospy.loginfo('Starting Action')
		    
		    # start executing the action
		    while #your condition:
		      # check that preempt has not been requested by the client
		      if self._as.is_preempt_requested():
			  #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
			  rospy.loginfo('Action Halted')
			  self._as.set_preempted()
			  success = False
			  break

		    rospy.loginfo('Executing Action')      
		    #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
		      
			#IF THE ACTION HAS SUCCEEDED
			self.set_status('SUCCESS')
			#IF THE ACTION HAS FAILED
			self.set_status('FAILURE')


Then set a name for your action. The name has the be unique. It will be used bt the behavior tree to recognize it.

     rospy.init_node(#NAME OF YOUR ACTION)

###Set up an Behavior Tree's condition in python
TODO



To gain familarity on how this works there is one example in C++ and one in python.
    
###Example in C++ ###
Run the Server

    rosrun bt_action Action 

Run the Client

    rosrun bt_action ActionClient 

Press: 1 to start the execution; 2 to halt the execution and 3 to terminate the program.



###Example in python ###
Run the Server

    roscd bt_action/src 
    python example_action_server.py 

Run the Client (Still in C++ but you don't modify this )

    rosrun bt_action ActionClient 

Press: 1 to start the execution; 2 to halt the execution and 3 to terminate the program.



###Examples used with the NAO robot

/src/bumpers_ok_server.cpp contains an example of a condition tested on the NAO robot

/src/move_forward_server.cpp contains an example of an action tested on the NAO robot

###Test Your Action

Write your action (C++ or python) and then set as name 'action'
i.e.: 

in C++

    ros::init(argc, argv, "action");

in python

    rospy.init_node('action')

Then run your node and the client (The client emulates the Behavior Tree)

    rosrun bt_action ActionClient

### Test your condition


TODO

### Who do I talk to? ###

Michele