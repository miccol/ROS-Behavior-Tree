#include <conditions/ros_condition.h>


enum Status {RUNNING,SUCCESS, FAILURE};


BT::ROSCondition::ROSCondition(std::string name) : action_client_(name,true), ConditionNode::ConditionNode(name)
{
    ROS_INFO("Waiting For the Acutator named %s to start", get_name().c_str());
    action_client_.waitForServer(); //will wait for infinite time until the server starts
    ROS_INFO("Actuator %s Started", get_name().c_str());
}

BT::ROSCondition::~ROSCondition() {}

BT::ReturnStatus BT::ROSCondition::Tick()
{

    ROS_INFO("I am running the request");

    // Condition checking and state update
    action_client_.sendGoal(goal);
    action_client_.waitForResult(ros::Duration(30.0));
    node_result = *(action_client_.getResult());
    set_status((ReturnStatus)node_result.status);
    return (ReturnStatus)node_result.status;

}
