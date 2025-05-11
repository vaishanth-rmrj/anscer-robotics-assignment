cmake_minimum_required(VERSION 3.0.2)
project(anscer_multimap_navigator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  std_srvs
  tf2_ros
  actionlib
  actionlib_msgs
  message_generation
  sql_database
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SwitchMap.srv
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  MultimapNavigate.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME} db_interface_lib
   CATKIN_DEPENDS roscpp std_msgs nav_msgs std_srvs database_interface actionlib actionlib_msgs message_runtime 
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /root/anscer-robotics-assignment/ws_ros/src/sql_database/database_interface/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/anscer_multimap_navigator.cpp
# )

add_library(db_interface src/db_interface.cpp)
target_link_libraries(db_interface
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/anscer_multimap_navigator_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES}
#)

add_executable(get_robot_location src/get_robot_map_location.cpp)
target_link_libraries(get_robot_location ${catkin_LIBRARIES})

add_executable(wormhole_location_saver src/wormhole_location_saver.cpp)
target_link_libraries(wormhole_location_saver postgresql_database)
target_link_libraries(wormhole_location_saver db_interface ${catkin_LIBRARIES})

### Navigator Action Server ###
add_executable(navigator_action_server src/navigator_action_server.cpp)
add_dependencies(navigator_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigator_action_server db_interface ${catkin_LIBRARIES})

### Navigator Action Client ###
add_executable(navigator_action_client src/navigator_action_client.cpp)
add_dependencies(navigator_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigator_action_client ${catkin_LIBRARIES})

### Dynamic Map Changer ###
add_executable(dynamic_map_changer src/dynamic_map_changer.cpp)
add_dependencies(dynamic_map_changer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_map_changer ${catkin_LIBRARIES})

install(
  TARGETS db_interface
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)