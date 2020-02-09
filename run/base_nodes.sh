#!/usr/bin/env bash

DOCKER_IMAGE="sheaffej/b2-base"
CONTAINER_NAME="b2_base_nodes"

[ -z "$ROS_MASTER_URI" ] && echo "Please set ROS_MASTER_URI env" && exit 1

# MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# REPOS_DIR=$MYDIR/../..  # Directory containing the cloned git repos
# CODE_MOUNT="/workspaces"

docker run -d --rm \
--name ${CONTAINER_NAME} \
--privileged \
--net host \
--env DISPLAY \
--env ROS_MASTER_URI \
${DOCKER_IMAGE} roslaunch b2_base b2-base.launch
