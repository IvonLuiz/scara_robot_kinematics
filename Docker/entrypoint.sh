#!/bin/bash
set -e

# Source the ROS Noetic environment
source /opt/ros/noetic/setup.bash



# Execute the passed command
exec "$@"
