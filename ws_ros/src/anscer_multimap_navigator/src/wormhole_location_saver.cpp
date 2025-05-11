
#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// project imports
#include <anscer_multimap_navigator/db_interface.h>

void get_robot_location(WormholeLocationData& location){

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // give buffer a moment to fill
  ros::Duration(0.5).sleep();
  geometry_msgs::TransformStamped transformStamped;

  try {
    ros::param::get("/map_server/map_name", location.map_name);

    // look up transform from "map" to "base_link"
    transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
    location.x = transformStamped.transform.translation.x;
    location.y = transformStamped.transform.translation.y;      

  } catch (tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(10.0, "Could not get transform: %s", ex.what());

  }
  ROS_INFO("Loaded map: %s", location.map_name.c_str());
  ROS_INFO("Robot pose in map frame: x=%.2f, y=%.2f", location.x, location.y);
  return;
}

int main(int argc, char **argv)
{ 

  ros::init(argc, argv, "wormhole_location_saver_node");
  ros::NodeHandle nh;

  // init db interface
  DBInterface db("localhost","11511","anscer","anscer","wormhole_locations");
  
  // check for connection
  if (!db.isConnected())
  {
    ROS_WARN("[Wormhole Location Saver] Database failed to connect.");
    return 0;
  }
  ROS_INFO("[Wormhole Location Saver] Database connected successfully.");

  WormholeLocationData location{};
  get_robot_location(location);  

  if (db.entryExists(location.map_name)) {
    db.updateEntry(location);
    ROS_INFO("[Wormhole Location Saver] Updated existing entry.");

  } else {
    db.addEntry(location);
    ROS_INFO("[Wormhole Location Saver] Inserted new entry.");
  }

  // get location saved in DB
  auto db_location = db.getEntry(location.map_name);
  if (db_location == nullptr)
  {
    ROS_WARN("[Wormhole Location Saver] Failed to retrieve wormhole locations from DB!");
    return 0;
  }
  ROS_INFO("[Wormhole Location Saver] Retrieved wormhole locations.");

  // display location
  ROS_INFO("DB Location Details: Map Name= %s | x=%.2f | y=%.2f", 
    db_location->map_name_.data().c_str(), 
    db_location->location_.data()[0], 
    db_location->location_.data()[1]
  );

  return 0;
}
