/*
 * Example for abstract class implementation
 */

#include <ros/ros.h>
#include <bt_action.h>


class StartCondition: public BTAction
{
public:
    bool start_condition = false;
    ros::NodeHandle nh_;

    StartCondition(ros::NodeHandle* n) :
            BTAction("StartCondition"),
            nh_(*n)
    {
        nh_.param("/start_condition", start_condition, false);
    }

    void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal)
    {
        set_status(SUCCESS);
        // if (start_condition)
            // set_status(SUCCESS);
        // else
            // set_status(FAILURE);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "start_conditions");
    ros::NodeHandle nh;
    StartCondition sc(&nh);
    ros::spin();
    return 0;
}
