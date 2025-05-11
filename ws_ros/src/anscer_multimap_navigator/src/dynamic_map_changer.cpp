#include <anscer_multimap_navigator/dynamic_map_changer.h>

namespace anscer_multimap_navigator {

DynamicMapChanger::DynamicMapChanger() {
  loadParameter();
  configureCommunication();
  loadInitialMaps();
  publishSelected();
  ROS_INFO("published initial map '%s' on /map", selected_map_.c_str());
}

void DynamicMapChanger::spin() {
  ros::spin();
}

void DynamicMapChanger::loadParameter() {
  ros::param::get("map_name", selected_map_);
}

void DynamicMapChanger::configureCommunication() {
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
  map_sub1_ = nh_.subscribe("/map_1", 1, &DynamicMapChanger::map1Callback, this);
  map_sub2_ = nh_.subscribe("/map_2", 1, &DynamicMapChanger::map2Callback, this);
  switch_srv_ = nh_.advertiseService("switch_map", &DynamicMapChanger::switchCallback, this);
}

void DynamicMapChanger::loadInitialMaps() {
  ROS_INFO("waiting for initial maps...");
  if (auto msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map_1")) {
    storeMap("room1", *msg);
    ROS_INFO("loaded /map_1");

  }
  if (auto msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map_2")) {
    storeMap("room2", *msg);
    ROS_INFO("loaded /map_2");

  }
}

void DynamicMapChanger::storeMap(const std::string& name, const nav_msgs::OccupancyGrid& msg) {
  if (name == "room1") {
    map1_ = msg;
    has_map1_ = true;

  } else if (name == "room2") {
    map2_ = msg;
    has_map2_ = true;

  }
}

void DynamicMapChanger::map1Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  ROS_INFO("received /map_1");
  storeMap("room1", *msg);
}

void DynamicMapChanger::map2Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  ROS_INFO("received /map_2");
  storeMap("room2", *msg);
}

bool DynamicMapChanger::switchCallback(SwitchMap::Request& req, SwitchMap::Response& resp) {
  const auto& name = req.map_name;
  if ((name == "room1" && has_map1_) || (name == "room2" && has_map2_)) {
    selected_map_ = name;
    publishSelected();
    resp.success = true;
    resp.message = "switched to " + name;

  } else {
    resp.success = false;
    resp.message = "unknown or unavailable map: " + name;

  }
  ROS_INFO("%s", resp.message.c_str());
  return true;
}

void DynamicMapChanger::publishSelected() {
  if (selected_map_ == "room1" && has_map1_) {
    ROS_INFO("publishing room1 on /map");
    map_pub_.publish(map1_);

  } else if (selected_map_ == "room2" && has_map2_) {
    ROS_INFO("publishing room2 on /map");
    map_pub_.publish(map2_);

  } else {
    ROS_WARN("no map ready for '%s'", selected_map_.c_str());

  }
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_map_changer");
  anscer_multimap_navigator::DynamicMapChanger map_changer;
  map_changer.spin();
  return 0;
}