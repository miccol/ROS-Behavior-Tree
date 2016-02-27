#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behavior_tree_core/BTAction.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_action");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<behavior_tree_core::BTAction> ac("action", true);
    behavior_tree_core::BTResult node_result;
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  behavior_tree_core::BTGoal goal;
  //goal.order = 20;


int command=0;
bool isRunning = false;



 	while(command!=3){

	ROS_INFO("Send a command: 1:start the action | 2:stop the action | 3:exit the program");
	std::cin >> command;

		switch(command){
			case 1: 
                if(!isRunning){
                    ROS_INFO("I am running the request");
	  				ac.sendGoal(goal);
					//ac.ClientGoalHandle();
                    isRunning=true;
					//ac.waitForResult(ros::Duration(30.0));
                    node_result = *(ac.getResult());
                     ROS_INFO("Action finished, status: %d",node_result.status);
                }else{
                    ROS_INFO("I am re-running the request");
                    ac.cancelGoal();
                    ac.sendGoal(goal);
                     ROS_INFO("Action finished, status: %d",node_result.status);
                }
			break;
			case 2: 
                ROS_INFO("I am cancelling the request");
				ac.cancelGoal();
				isRunning=false;
			break;
			default:
			break;
		}




	}

return 0;

}

