#!/usr/bin/env bash

DOCKER_IMAGE="sheaffej/b2-base"
CONTAINER_NAME="base_nodes"
LABEL="b2"

[ -z "$ROS_MASTER_URI" ] && echo "Please set ROS_MASTER_URI env" && exit 1

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJ_DIR=$MYDIR/../..  # Directory containing the cloned git repos
DOWNLOADS_DIR=~/Downloads
CODE_MOUNT="/workspaces"

while [ $# -gt 0 ]; do
    case $1 in
        "test")
            TEST="test_mode:=true"
            TEST_LABEL="--label test"
            echo "Running in test mode"
            ;;
    esac
    shift
done

docker run -d --rm \
--name ${CONTAINER_NAME} \
--label ${LABEL} ${TEST_LABEL} \
--net host \
--privileged \
--env DISPLAY \
--env ROS_MASTER_URI \
--mount type=bind,source=$PROJ_DIR/b2-base,target=$CODE_MOUNT/b2-base \
--mount type=bind,source=$PROJ_DIR/roboclaw_driver,target=$CODE_MOUNT/roboclaw_driver \
${DOCKER_IMAGE} roslaunch --screen b2_base b2-base.launch ${TEST}
