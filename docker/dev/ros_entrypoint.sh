#!/usr/bin/bash
set -e

# Set ROS to use port 11411
export ROS_MASTER_URI=http://localhost:11411
export ROS_HOSTNAME=localhost

# setup ros environment
source "/opt/ros/noetic/setup.bash"

exec "$@"