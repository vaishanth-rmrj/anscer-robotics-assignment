#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "get_location_node");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(1.0);
  int ping_count = 3;
  std::string map_name;
  while (ping_count){
    geometry_msgs::TransformStamped transformStamped;
    try {

      if (ros::param::get("/map_server/map_name", map_name)) {
        ROS_INFO("Loaded map: %s", map_name.c_str());
      } else {
        ROS_WARN("No map_name param set on the parameter server!");
      }
      // look up transform from "map" to "base_link"
      transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
      double x = transformStamped.transform.translation.x;
      double y = transformStamped.transform.translation.y;

      ROS_INFO("Robot pose in map frame: x=%.2f, y=%.2f", x, y);

    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(10.0, "Could not get transform: %s", ex.what());
    }
    rate.sleep();
    --ping_count;
  }
  return 0;
}
