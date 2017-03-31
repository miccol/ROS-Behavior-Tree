#include <actions/ros_action.h>



BT::ROSAction::ROSAction(std::string name) : action_client_(name,true), ActionNode::ActionNode(name)
{


    actionlib::SimpleActionClient<behavior_tree_core::BTAction> action_client_(get_name(), true);
    ROS_INFO("Waiting For the Acutator named %s to start", get_name().c_str());
    action_client_.waitForServer(); //will wait for infinite time until the server starts
    ROS_INFO("The Acutator %s has started", get_name().c_str());
    // thread_ start
    thread_ = std::thread(&ROSAction::WaitForTick, this);


}

BT::ROSAction::~ROSAction() {}

void BT::ROSAction::WaitForTick()
{




      node_result.status = BT::RUNNING;//
      while(true)
      {
          // Waiting for a tick to come
          tick_engine.Wait();

          // Running state
          node_result.status = BT::RUNNING;
          // Perform action...
          ROS_INFO("I am running the request to %s",get_name().c_str());
          action_client_.sendGoal(goal);
          do
          {
              node_result = *(action_client_.getResult());//checking the result
          } while(node_result.status == BT::RUNNING && get_status() != BT::HALTED);

          if(get_status() == BT::HALTED)
          {
              ROS_INFO("The Node is Halted");
              ROS_INFO("I am Halting the client");
              action_client_.cancelGoal();
          }
          else{
              ROS_INFO("The Server Has Replied");
              // Set this node status according to what the external node replied
              set_status((ReturnStatus)node_result.status);
          }
      }
}


void BT::ROSAction::Halt()
{
    set_status(BT::HALTED);
}
