#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <anscer_multimap_navigator/MultimapNavigateAction.h>
#include <move_base_msgs/MoveBaseAction.h>

// db interface imports
#include <boost/shared_ptr.hpp>

// project imports
#include <anscer_multimap_navigator/db_interface.h>

typedef actionlib::SimpleActionServer<anscer_multimap_navigator::MultimapNavigateAction> Server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MultimapNavigatorActionServer
{
protected:
  ros::NodeHandle nh_;
  Server server_;
  MoveBaseClient move_base_ac_;
  std::string action_name_;
  DBInterface db;

public:
  MultimapNavigatorActionServer(const std::string& name)
    : server_(nh_, name, boost::bind(&MultimapNavigatorActionServer::executeCB, this, _1), false),
      action_name_(name),
      db("localhost","11511","anscer","anscer","wormhole_locations"),
      move_base_ac_("move_base", true)
  {
    // wait for move_base to come up
    ROS_INFO("[Navigator Action Server] Waiting for move_base action server...");
    move_base_ac_.waitForServer();
    ROS_INFO("[Navigator Action Server] move_base action server is up.");

    // check for db connection
    if (!db.isConnected())
    {
      ROS_WARN("[Navigator Action Server] Unable to establish connection with wormholeDB. Aborting, Try again!");
      return;
    }

    server_.start();
    ROS_INFO("[Navigator Action Server] '%s' started, waiting for goals...", action_name_.c_str());
  }

  bool _fetch_current_map_name(std::string &map_name){
    if (!nh_.getParam("/map_server/map_name", map_name))
    {
      ROS_ERROR("[Navigator Action Server] Failed to read /map_server/map_name");      
      return false;
    }
    return true;
  }

  void executeCB(const anscer_multimap_navigator::MultimapNavigateGoalConstPtr& goal)
  {
    ROS_INFO("[Navigator Action Server] Received goal: name='%s', x=%f, y=%f",
        goal->map_name.c_str(), goal->x, goal->y);

    // get current map name from param server
    std::string current_map_name;
    if(!this->_fetch_current_map_name(current_map_name)){
        server_.setAborted(anscer_multimap_navigator::MultimapNavigateResult(), "Could not read current map name");
    }  
    

    // check if not same map
    if(goal->map_name != current_map_name){
        ROS_WARN("[Navigator Action Server] Goal map '%s' does not match current map '%s'",
            goal->map_name.c_str(), current_map_name.c_str());
        
        // get current map wormhole location
        auto wormhole_location = db.getEntry(current_map_name);
        if (wormhole_location == nullptr)
        {
          ROS_WARN("[Navigator Action Server] Failed to retrieve wormhole locations from DB for current map '%s'", current_map_name.c_str());

          anscer_multimap_navigator::MultimapNavigateResult result;
          result.success = false;
          result.message = "Failed to retrieve wormhole location!";  
          server_.setAborted(result);
          return;
        }

        // send wormhole location to move base
        move_base_msgs::MoveBaseGoal mb_wormhole;
        mb_wormhole.target_pose.header.frame_id = "map";
        mb_wormhole.target_pose.header.stamp    = ros::Time::now();
        mb_wormhole.target_pose.pose.position.x = wormhole_location->location_.data()[0];
        mb_wormhole.target_pose.pose.position.y = wormhole_location->location_.data()[1];
        mb_wormhole.target_pose.pose.orientation.w = 1.0; 

        ROS_INFO("[Navigator Action Server] Sending wormhole location to move_base: x=%.2f, y=%.2f", 
          wormhole_location->location_.data()[0],
          wormhole_location->location_.data()[1]
        );
        move_base_ac_.sendGoal(mb_wormhole);

        // wait for move base to finish
        bool has_reached_wormhole = move_base_ac_.waitForResult(ros::Duration(90.0));

        // check move base status
        if (!has_reached_wormhole || move_base_ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR("[Navigator Action Server] move_base failed: %s", move_base_ac_.getState().toString().c_str());
            server_.setAborted(anscer_multimap_navigator::MultimapNavigateResult(), "move_base failed");
            return;
        }

        anscer_multimap_navigator::MultimapNavigateResult result;
        result.success = true;
        result.message = "Wormhole Location reached";  
        server_.setAborted(result);
        return;
    }


    // send the goal to move base
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose.header.frame_id = "map";
    mb_goal.target_pose.header.stamp    = ros::Time::now();
    mb_goal.target_pose.pose.position.x = goal->x;
    mb_goal.target_pose.pose.position.y = goal->y;
    mb_goal.target_pose.pose.orientation.w = 1.0; 


    ROS_INFO("[Navigator Action Server] Sending goal to move_base: x=%.2f, y=%.2f", goal->x, goal->y);
    move_base_ac_.sendGoal(mb_goal);

    // wait for move base to finish
    bool has_moved = move_base_ac_.waitForResult(ros::Duration(90.0));

    // check move base status
    if (!has_moved || move_base_ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("[Navigator Action Server] move_base failed: %s", move_base_ac_.getState().toString().c_str());
        server_.setAborted(anscer_multimap_navigator::MultimapNavigateResult(), "move_base failed");
        return;
    }

    ROS_INFO("[Navigator Action Server] Robot Succesfully reached Destination at x=%.2f, y=%.2f", goal->x, goal->y);
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