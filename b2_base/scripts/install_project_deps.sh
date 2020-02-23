#!/bin/bash

pushd $ROS_WS

catkin_make
source ${ROS_WS}/devel/setup.bash

apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

popd