#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Source user's workspace if it exists
if [ -f "/home/user/install/setup.bash" ]; then
  source "/home/user/install/setup.bash"
fi

# Execute the command passed to the Docker container
exec "$@"