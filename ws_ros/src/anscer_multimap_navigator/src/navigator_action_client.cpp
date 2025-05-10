#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <anscer_multimap_navigator/MultimapNavigateAction.h>

typedef actionlib::SimpleActionClient<anscer_multimap_navigator::MultimapNavigateAction> Client;

int main(int argc, char** argv)
{   
    // init client node
    ros::init(argc, argv, "navigator_action_client_node");

    // init ac client
    Client _ac_client("multimap_navigator_action", true);

    ROS_INFO("[Action Client] Waiting for action server to start...");
    _ac_client.waitForServer();

    anscer_multimap_navigator::MultimapNavigateGoal goal;
    // sample loc
    goal.map_name = "room1";
    goal.x = 1.23;
    goal.y = 4.56;

    ROS_INFO("[Action Client] Sending goal: name='%s', x=%f, y=%f", goal.map_name.c_str(), goal.x, goal.y);
    _ac_client.sendGoal(goal);

    // wait till action server finishes
    bool is_done = _ac_client.waitForResult(ros::Duration(5.0));

    if (is_done)
    {
        auto result = _ac_client.getResult();
        ROS_INFO("[Action Client] Server responded: success=%s, message='%s'",
                result->success ? "true" : "false",
                result->message.c_str());
    }
    else
    {
        ROS_WARN("[Action Client] Action did not finish before the timeout.");
    }

    return 0;
}