#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "anscer_multimap_navigator/SwitchMap.h"

class DynamicMapChanger
{
    public:
    DynamicMapChanger()
    {
        // get current map name from param server
        ros::param::get("map_name", selected_map_);
        // default /map topic pub
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

        // sub to all map servers
        map_server_sub1_ = nh_.subscribe("/map_1", 1, &DynamicMapChanger::map_server_1_cb, this);
        map_server_sub2_ = nh_.subscribe("/map_2", 1, &DynamicMapChanger::map_server_2_cb, this);

        // create service to dynamical switch maps from nodes
        srv_ = nh_.advertiseService("switch_map", &DynamicMapChanger::switch_cb, this);

        // wait for the very first messages to arrive
        ROS_INFO("Waiting for initial maps...");
        // grab the latched maps
        auto msg1 = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map_1", ros::Duration(5.0));
        if (msg1) { map1_ = *msg1; has_map_1_ = true; ROS_INFO("Loaded /map_1"); }
        auto msg2 = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map_2", ros::Duration(5.0));
        if (msg2) { map2_ = *msg2; has_map_2_ = true; ROS_INFO("Loaded /map_2"); }

        // publish the default map
        publishSelected();
        ROS_INFO("Published initial of map %s on /map", selected_map_.c_str());
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Subscriber map_server_sub1_, map_server_sub2_;
    ros::ServiceServer srv_;

    nav_msgs::OccupancyGrid map1_, map2_;
    bool has_map_1_{false}, has_map_2_{false};
    std::string selected_map_;

    void map_server_1_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

        ROS_INFO("Received msg from /map_1");
        map1_ = *msg; 
        has_map_1_ = true;
    }

    void map_server_2_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        
        ROS_INFO("Received msg from /map_2");
        map2_ = *msg; 
        has_map_2_ = true;
    }

    bool switch_cb(anscer_multimap_navigator::SwitchMap::Request& req, anscer_multimap_navigator::SwitchMap::Response& resp)
    {
        const auto& name = req.map_name;
        if (name == "room1" && has_map_1_) {
            selected_map_ = name;
            publishSelected();
            resp.success = true;
            resp.message = "Switched to room1";
        }
        else if (name == "room2" && has_map_2_) {
            selected_map_ = name;
            publishSelected();
            resp.success = true;
            resp.message = "Switched to room2";
        }
        else {
            resp.success = false;
            resp.message = "Unknown or unavailable map: " + name;
        }
        ROS_INFO("%s", resp.message.c_str());
        return true;
    }

    void publishSelected()
    {
        if (selected_map_ == "room1"){
            ROS_INFO("Publishing room1 on /map");
            map_pub_.publish(map1_);

        } else if (selected_map_ == "room2") {
            ROS_INFO("Publishing room2 on /map");
            map_pub_.publish(map2_);

        } else {
            ROS_WARN("publishSelected(): no map ready for '%s'", selected_map_.c_str());
        }
                            
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_map_changer");
  DynamicMapChanger map_changer;
  ros::spin();
  return 0;
}
