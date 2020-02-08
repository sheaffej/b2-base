#!/usr/bin/env bash

DOCKER_IMAGE="sheaffej/b2-base"

[ -z "$ROS_MASTER_URI" ] && echo "Please set ROS_MASTER_URI env" && exit 1

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJ_DIR=$MYDIR/..  # Directory containing the cloned git repos
CODE_MOUNT="/workspaces"

docker run -it --rm \
--privileged \
--net host \
--env DISPLAY \
--env ROS_MASTER_URI \
--mount type=bind,source=$PROJ_DIR,target=$CODE_MOUNT/b2-base \
${DOCKER_IMAGE} $@
