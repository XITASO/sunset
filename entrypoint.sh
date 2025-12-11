#!/bin/bash
set -e

echo $@

# Source the ROS setup script
source /opt/ros/humble/setup.bash
source /bt_workspace/install/setup.bash
source /ros_ws/install/setup.bash

# Execute the command passed to the Docker container
exec "$@"