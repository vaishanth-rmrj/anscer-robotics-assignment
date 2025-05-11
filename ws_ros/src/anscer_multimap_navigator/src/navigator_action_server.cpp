#include <anscer_multimap_navigator/navigator_action_server.h>

namespace anscer_multimap_navigator {

MultimapNavigatorActionServer::MultimapNavigatorActionServer(const std::string& name)
  : server_(nh_, name, boost::bind(&MultimapNavigatorActionServer::executeCallback, this, _1), false),
    move_base_ac_("move_base", true),
    action_name_(name),
    db_("localhost", "11511", "anscer", "anscer", "wormhole_locations"),
    switch_map_client_(nh_.serviceClient<SwitchMap>("switch_map"))
{
  ROS_INFO("[Navigator] waiting for move_base action server");
  move_base_ac_.waitForServer();

  if (!db_.isConnected()) {
    ROS_WARN("[Navigator] could not connect to wormholeDB");
  }

  ROS_INFO("[Navigator] waiting for switch_map service");
  switch_map_client_.waitForExistence();

  server_.start();
  ROS_INFO("[Navigator] '%s' started, waiting for goals", action_name_.c_str());
}

void MultimapNavigatorActionServer::spin() {
  ros::spin();
}

bool MultimapNavigatorActionServer::fetchCurrentMapName(std::string& map_name) {
  if (!nh_.getParam("map_name", map_name)) {
    ROS_ERROR("[Navigator] failed to read /map_name");
    return false;
  }
  return true;
}

bool MultimapNavigatorActionServer::getWormholeLocation(const std::string& map_name, double& x, double& y) {
  auto entry = db_.getEntry(map_name);
  if (!entry) {
    ROS_WARN("[Navigator] no wormhole entry for map '%s'", map_name.c_str());
    return false;
  }
  const auto& loc = entry->location_.data();
  if (loc.size() < 2) {
    ROS_WARN("[Navigator] invalid location data for map '%s'", map_name.c_str());
    return false;
  }
  x = loc[0];
  y = loc[1];
  return true;
}

bool MultimapNavigatorActionServer::sendMoveBaseGoal(double x, double y, double timeout) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("[Navigator] sending move_base goal x=%.2f y=%.2f", x, y);
  move_base_ac_.sendGoal(goal);
  bool ok = move_base_ac_.waitForResult(ros::Duration(timeout));

  if (!ok || move_base_ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("[Navigator] move_base failed: %s", move_base_ac_.getState().toString().c_str());
    return false;
  }
  return true;
}

bool MultimapNavigatorActionServer::callSwitchMapService(const std::string& target_map) {
  SwitchMap srv;
  srv.request.map_name = target_map;
  if (!switch_map_client_.call(srv)) {
    ROS_ERROR("[Navigator] failed to call switch_map");
    return false;
  }
  if (!srv.response.success) {
    ROS_WARN("[Navigator] switch_map failed: %s", srv.response.message.c_str());
    return false;
  }
  ROS_INFO("[Navigator] switched map: %s", srv.response.message.c_str());
  return true;
}

void MultimapNavigatorActionServer::executeCallback(const MultimapNavigateGoalConstPtr& goal) {
  ROS_INFO("[Navigator] goal received: map='%s' x=%.2f y=%.2f", goal->map_name.c_str(), goal->x, goal->y);

  std::string current_map;
  if (!fetchCurrentMapName(current_map)) {
    server_.setAborted(MultimapNavigateResult(), "could not read current map name");
    return;
  }

  if (goal->map_name != current_map) {
    double wx, wy;
    if (!getWormholeLocation(current_map, wx, wy)) {
      MultimapNavigateResult res;
      res.success = false;
      res.message = "failed to retrieve wormhole location";
      server_.setAborted(res);
      return;
    }
    if (!sendMoveBaseGoal(wx, wy, 90.0)) {
      server_.setAborted(MultimapNavigateResult(), "move_base failed to reach wormhole");
      return;
    }
    if (!callSwitchMapService(goal->map_name)) {
      MultimapNavigateResult res;
      res.success = false;
      res.message = "unable to switch map";
      server_.setAborted(res);
      return;
    }
  }

  if (!sendMoveBaseGoal(goal->x, goal->y, 90.0)) {
    server_.setAborted(MultimapNavigateResult(), "move_base failed to reach destination");
    return;
  }

  ROS_INFO("[Navigator] reached destination: x=%.2f y=%.2f", goal->x, goal->y);
  MultimapNavigateResult res;
  res.success = true;
  res.message = "location processed";
  server_.setSucceeded(res);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "multimap_navigator_action_server_node");
  anscer_multimap_navigator::MultimapNavigatorActionServer server("multimap_navigator_action");
  server.spin();
  return 0;
}