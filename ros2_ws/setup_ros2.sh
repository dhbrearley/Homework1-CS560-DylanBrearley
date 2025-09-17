#!/bin/bash

# Get the directory this script is located in
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS setup
source /opt/ros/humble/setup.bash

# Source the local setup using an absolute path
source "$SCRIPT_DIR/install/local_setup.bash"

