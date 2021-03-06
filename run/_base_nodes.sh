#!/usr/bin/env bash

DOCKER_IMAGE="sheaffej/b2-base"
CONTAINER_NAME="base_nodes"

[ -z "$ROS_MASTER_URI" ] && echo "Please set ROS_MASTER_URI env" && exit 1

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJ_DIR=$MYDIR/../..  # Directory containing the cloned git repos
CODE_MOUNT="/workspaces"

while [ $# -gt 0 ]; do
    case $1 in
        "test")
            TEST="test_mode:=true"
            echo "Running in test mode"
            ;;
        "dev")
            VOLUMES="--mount type=bind,source=$PROJ_DIR/b2-base,target=$CODE_MOUNT/b2-base"
            VOLUMES="$VOLUMES --mount type=bind,source=$PROJ_DIR/roboclaw_driver,target=$CODE_MOUNT/roboclaw_driver"
            echo "Running in dev mode"
            ;;
        *)
            echo "Unknown argument" $1
    esac
    shift
done

echo "Starting container: ${CONTAINER_NAME}"
docker run -d --rm \
--name ${CONTAINER_NAME} \
--privileged \
--net host \
--env DISPLAY \
--env ROS_MASTER_URI \
$VOLUMES \
${DOCKER_IMAGE} roslaunch --screen b2_base b2-base.launch ${TEST}
