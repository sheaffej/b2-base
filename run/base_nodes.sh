#!/usr/bin/env bash

DOCKER_IMAGE="sheaffej/b2-base"
CONTAINER_NAME="b2_base_nodes"

[ -z "$ROS_MASTER_URI" ] && echo "Please set ROS_MASTER_URI env" && exit 1

while [ $# -gt 0 ]; do
    case $1 in
        "test")
            TEST="test_mode:=true"
            echo "Running in test mode"
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
${DOCKER_IMAGE} roslaunch --screen b2_base b2-base.launch ${TEST}