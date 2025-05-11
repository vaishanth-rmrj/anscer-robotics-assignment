#ifndef DYNAMIC_MAP_CHANGER_H
#define DYNAMIC_MAP_CHANGER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <anscer_multimap_navigator/SwitchMap.h>

namespace anscer_multimap_navigator {

/**
 * dynamically switches between multiple occupancy grid maps
 */
class DynamicMapChanger {
public:
  /**
   * initializes publisher, subscribers, service, and loads initial maps
   */
  DynamicMapChanger();

  /**
   * starts the ROS event loop
   */
  void spin();

private:
  ros::NodeHandle nh_;
  ros::Publisher map_pub_;
  ros::Subscriber map_sub1_;
  ros::Subscriber map_sub2_;
  ros::ServiceServer switch_srv_;

  nav_msgs::OccupancyGrid map1_;
  nav_msgs::OccupancyGrid map2_;
  bool has_map1_{false};
  bool has_map2_{false};
  std::string selected_map_;

  /**
   * reads initial map_name parameter
   */
  void loadParameter();

  /**
   * sets up publisher, subscribers, and service
   */
  void configureCommunication();

  /**
   * loads latched maps from topics
   */
  void loadInitialMaps();

  /**
   * stores incoming map by name
   * @param name identifier of the map topic
   * @param msg map data
   */
  void storeMap(const std::string& name, const nav_msgs::OccupancyGrid& msg);

  /**
   * publishes the currently selected map
   */
  void publishSelected();

  /**
   * callback for first map topic
   */
  void map1Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  /**
   * callback for second map topic
   */
  void map2Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  /**
   * handles service requests to switch maps
   */
  bool switchCallback(SwitchMap::Request& req, SwitchMap::Response& resp);
};

}

#endif