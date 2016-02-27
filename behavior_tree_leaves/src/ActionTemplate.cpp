#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>// needed for actionlib
#include <behavior_tree_core/BTAction.h>



enum Status {RUNNING,SUCCESS, FAILURE};//BT return status


class BTAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<amazon_challenge_behavior_tree_leaves::BTAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  behavior_tree_leaves::BTFeedback feedback_; //action feedback (SUCCESS, FAILURE)
  behavior_tree_leaves::BTResult result_;//action feedback  (same as feedback for us)


public:


  BTAction(std::string name) :
    as_(nh_, name, boost::bind(&BTAction::executeCB, this, _1), false),
    action_name_(name)
  {
   //Starts the action server
    as_.start();


  }

  ~BTAction(void)
  {


  }

  void executeCB(const behavior_tree_leaves::BTGoalConstPtr &goal)
  {

    // publish info to the console for the user
    ROS_INFO("Starting Action");

    // start executing the action
    while(/*YOUR LOOP CONDITION (IF ANY)*/)
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



   }
    //If the action succeeded
         setStatus(SUCCESS);
    //If the action Failed
         setStatus(FAILURE);

  }


//returns the status to the client (Behavior Tree)
  void setStatus(int status){
      //Set The feedback and result of BT.action
      feedback_.status = status;
      result_.status = feedback_.status;
      // publish the feedback
      as_.publishFeedback(feedback_);
      // setSucceeded means that it has finished the action (it has returned SUCCESS or FAILURE).
      as_.setSucceeded(result_);

      switch(status){//Print for convenience
      case SUCCESS:
        ROS_INFO("Action %s Succeeded", ros::this_node::getName().c_str() );
        break;
      case FAILURE:
          ROS_INFO("Action %s Failed", ros::this_node::getName().c_str() );
        break;
      default:
        break;
      }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, /*name of your action as a std::string*/);
      ROS_INFO(" Enum: %d",RUNNING);
      ROS_INFO(" Action Ready for Ticks");
  BTAction bt_action(ros::this_node::getName());
  ros::spin();

  return 0;
}
