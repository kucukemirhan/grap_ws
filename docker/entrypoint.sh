#!/bin/bash
set -e

# Source the ROS2 environment
source /opt/ros/humble/setup.sh

# Source the built workspace environment
source /home/ros/grap_ws/install/setup.bash

# Execute the container's CMD
exec "$@"
