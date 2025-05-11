#ifndef MULTIMAP_NAVIGATOR_ACTION_SERVER_H
#define MULTIMAP_NAVIGATOR_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <anscer_multimap_navigator/MultimapNavigateAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <anscer_multimap_navigator/db_interface.h>
#include <anscer_multimap_navigator/SwitchMap.h>

namespace anscer_multimap_navigator {

/**
 * handles navigation goals across multiple maps using wormhole locations and move_base
 */
class MultimapNavigatorActionServer {
public:
  /**
   * constructs the action server with given name
   * @param name action server name
   */
  explicit MultimapNavigatorActionServer(const std::string& name);

  /**
   * blocks until shutdown
   */
  void spin();

private:
  using Server = actionlib::SimpleActionServer<MultimapNavigateAction>;
  using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  ros::NodeHandle nh_;
  Server server_;
  MoveBaseClient move_base_ac_;
  std::string action_name_;
  DBInterface db_;
  ros::ServiceClient switch_map_client_;

  /**
   * fetches current map name from parameter server
   * @param[out] map_name current map
   * @return true if read succeeds
   */
  bool fetchCurrentMapName(std::string& map_name);

  /**
   * retrieves wormhole location for map
   * @param map_name source map
   * @param[out] x wormhole x coordinate
   * @param[out] y wormhole y coordinate
   * @return true if entry found
   */
  bool getWormholeLocation(const std::string& map_name, double& x, double& y);

  /**
   * sends a goal to move_base and waits
   * @param x target x
   * @param y target y
   * @param timeout maximum wait in seconds
   * @return true if move_base succeeds
   */
  bool sendMoveBaseGoal(double x, double y, double timeout);

  /**
   * calls the switch_map service for target map
   * @param target_map name to switch to
   * @return true if service call and response are successful
   */
  bool callSwitchMapService(const std::string& target_map);

  /**
   * handles incoming navigation goals
   */
  void executeCallback(const MultimapNavigateGoalConstPtr& goal);
};

}

#endif 