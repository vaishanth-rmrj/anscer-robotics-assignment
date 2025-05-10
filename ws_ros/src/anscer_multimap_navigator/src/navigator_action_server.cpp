#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <anscer_multimap_navigator/MultimapNavigateAction.h>

typedef actionlib::SimpleActionServer<anscer_multimap_navigator::MultimapNavigateAction> Server;

class MultimapNavigatorActionServer
{
protected:
  ros::NodeHandle nh_;
  Server server_;
  std::string action_name_;

public:
  MultimapNavigatorActionServer(const std::string& name)
    : server_(nh_, name, boost::bind(&MultimapNavigatorActionServer::executeCB, this, _1), false),
      action_name_(name)
  {
    server_.start();
    ROS_INFO("[Server] '%s' started, waiting for goals...", action_name_.c_str());
  }

  void executeCB(const anscer_multimap_navigator::MultimapNavigateGoalConstPtr& goal)
  {
    ROS_INFO("[Server] Received goal: name='%s', x=%f, y=%f",
        goal->map_name.c_str(), goal->x, goal->y);

    // debug: immediately succeed
    anscer_multimap_navigator::MultimapNavigateResult result;
    result.success = true;
    result.message = "Location '" + goal->map_name + "' processed";
    server_.setSucceeded(result);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multimap_navigator_action_server_node");
  MultimapNavigatorActionServer ac_server("multimap_navigator_action");
  ros::spin();
  return 0;
}