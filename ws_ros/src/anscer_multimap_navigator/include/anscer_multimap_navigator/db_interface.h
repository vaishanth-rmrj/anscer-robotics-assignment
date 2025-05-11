#ifndef DB_INTERFACE_H
#define DB_INTERFACE_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>
#include <database_interface/db_class.h>

#include <string>
#include <vector>

struct WormholeLocationData {
  std::string map_name;
  double      x   = 0.0;
  double      y   = 0.0;
};

// db table
class WormholeLocations : public database_interface::DBClass
{
public:
  database_interface::DBField<std::string>        map_name_;
  database_interface::DBField<std::vector<double>> location_;

  WormholeLocations();
};

/// thin wrapper around database_interface package
class DBInterface{

public:
    /// @param host        DB host (e.g. "localhost")
    /// @param port        DB port as string (e.g. "11511")
    /// @param user        username
    /// @param password    password
    /// @param dbname      database name (should match your table schema)
    DBInterface(
        const std::string &host,
        const std::string &port,
        const std::string &user,
        const std::string &password,
        const std::string &dbname
    );

    // check if database connection can be established
    bool isConnected() const;

    // check if entry exists based on map name
    bool entryExists(const std::string &map_name) const;

    // add new entry to db
    bool addEntry(const WormholeLocationData &data);

    // update already existing entry in db
    bool updateEntry(const WormholeLocationData &data);

    // get entry from db based on map name
    boost::shared_ptr<WormholeLocations> getEntry(const std::string &map_name) const;

private:
  database_interface::PostgresqlDatabase _db;
};
#endif
