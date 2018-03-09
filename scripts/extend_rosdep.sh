#!/bin/bash

# Call this in BEFORE_SCRIPT $CATKIN_WORKSPACE
# https://github.com/ros-industrial/industrial_ci/issues/206

# The directory above the scripts directory
PKG_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"/..

echo "yaml file:///${PKG_DIR}/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/40-custom.list

echo "--> Running rosdep update"
rosdep update
