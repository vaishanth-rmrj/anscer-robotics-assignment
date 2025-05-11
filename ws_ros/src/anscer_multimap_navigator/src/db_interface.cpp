#include <anscer_multimap_navigator/db_interface.h>

WormholeLocations::WormholeLocations() : 
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

DBInterface::DBInterface(
    const std::string &host,
    const std::string &port,
    const std::string &user,
    const std::string &password,
    const std::string &dbname
) : _db(host, port, user, password, dbname)
{}

bool DBInterface::isConnected() const
{
  return _db.isConnected();
}

bool DBInterface::entryExists(const std::string &map_name) const
{
  std::vector<boost::shared_ptr<WormholeLocations>> rows;
  std::string where = "map_name='" + map_name + "'";
  if (!_db.getList(rows, where)) {
    return false;
  }
  return !rows.empty();
}

bool DBInterface::addEntry(const WormholeLocationData &data)
{
  WormholeLocations new_location;
  new_location.map_name_.data() = data.map_name;
  new_location.location_.data() = {data.x, data.y};
  return _db.insertIntoDatabase(&new_location);
}

bool DBInterface::updateEntry(const WormholeLocationData &data)
{
  auto existing = getEntry(data.map_name);
  if (!existing) {
    return false;
  }
  existing->location_.data() = {data.x, data.y};
  // only save the location field
  return _db.saveToDatabase(&existing->location_);
}

boost::shared_ptr<WormholeLocations> DBInterface::getEntry(const std::string &map_name) const
{
  std::vector<boost::shared_ptr<WormholeLocations>> rows;
  std::string where = "map_name='" + map_name + "'";
  if (!_db.getList(rows, where) || rows.empty()) {
    return nullptr;
  }
  return rows.front();
}
