#include <string>
#include <vector>

#include <database_interface/db_class.h>
#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>

// ros imports
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

struct WormholeLocationData {
  std::string map_name;
  double x;
  double y;
};

class WormholeLocations : public database_interface::DBClass
{

public:
  
  // db fields 
  database_interface::DBField<std::string> map_name_;
  database_interface::DBField< std::vector<double> > location_;

  WormholeLocations() : 
    map_name_(database_interface::DBFieldBase::TEXT, 
		this, "map_name", "wormhole_locations", true),
    location_(database_interface::DBFieldBase::TEXT, 
			this, "location", "wormhole_locations", true)
  {
    primary_key_field_ = &map_name_;

    //all other fields go into the fields_ array of the DBClass
    fields_.push_back(&location_);

    // let all fields be read automatically when an instance 
    setAllFieldsReadFromDatabase(true);
    //let all fields be written automatically when an instance 
    setAllFieldsWriteToDatabase(true);
  }
};

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
  database_interface::PostgresqlDatabase database(
    "localhost", 
    "11511",
    "anscer", 
    "anscer", 
    "wormhole_locations"
  );
  
  // check for connection
  if (!database.isConnected())
  {
    std::cerr << "Database failed to connect \n";
    return -1;
  }
  std::cerr << "Database connected successfully \n";

  WormholeLocationData location{};
  get_robot_location(location);
  

  // add wormhole locations to db
  WormholeLocations wormhole_location;
  wormhole_location.map_name_.data() = location.map_name;
  wormhole_location.location_.data() = {location.x, location.y};

  // check if wormhole map location exists
  std::vector< boost::shared_ptr<WormholeLocations> > existing_locations;
  std::string where_clause = "map_name='" + location.map_name + "'";
  database.getList(existing_locations, where_clause);

  if(existing_locations.size() > 0){

    // update entry with new location
    existing_locations[0]->map_name_.data() = location.map_name;
    existing_locations[0]->location_.data() = {location.x, location.y};
    if (!database.saveToDatabase( &(existing_locations[0]->location_) ) )
      std::cerr << "Failed to modify location\n";
    else
      std::cerr << "Location modified successfully\n";

  } else {

    // insert new location
    if (!database.insertIntoDatabase(&wormhole_location)) 
      std::cerr << "Wormhole Location insertion failed\n";
    else 
      std::cerr << "Wormhole Location insertion succeeded\n";
  }

  // get list of all locations
  std::vector< boost::shared_ptr<WormholeLocations> > wormhole_locations;
  if (!database.getList(wormhole_locations))
  {
    std::cerr << "Failed to get list of Wormhole Locations\n";
    return -1;
  }
  std::cerr << "Retrieved " << wormhole_locations.size() << " locations(s) \n";


  // display all locations
  std::cerr << "Locations:\n";
  for (size_t i = 0; i < wormhole_locations.size(); ++i)
  {
    std::cerr << wormhole_locations[i]->map_name_.data() << ": [";
    const auto &locs = wormhole_locations[i]->location_.data();
    for (size_t j = 0; j < locs.size(); ++j) {
        std::cerr << locs[j];
        if (j + 1 < locs.size()) 
            std::cerr << ", ";
    }

    std::cerr << "]\n\n";
  }

  return 0;
}
