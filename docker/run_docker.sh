#!/bin/bash

IMAGE_NAME="b2-base:latest"
CONTAINER_NAME="b2-base"    # Hostname for the started container
ROBOT_HOSTNAME="b2"         # Hostname of the Ubuntu robot host
DEV_HOSTNAME="ros-dev"      # Hostname of the Ubuntu dev host/VM

CODE_MOUNT="/workspaces"

# DEVICES="/dev/roboclaw_front /dev/roboclaw_rear /dev/spidev0.0 /dev/spidev0.1 /dev/input/js0 /dev/dri"
VOLUMES="/tmp/.X11-unix $HOME/.Xauthority:/root/.Xauthority"

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
REPOS_DIR=$MYDIR/../..  # Directory containing the cloned git repos
# LOCAL_DIR="$REPOS_DIR"

# Daemon or Interactive
OPTIONS="-d"
if [[ ! -z $1 && $1 == "it" ]]; then
    OPTIONS="-it"
fi

# Configure volumes, devices, and networking based on host
if [[ `hostname` == $ROBOT_HOSTNAME || `hostname` == $DEV_HOSTNAME ]]; then

    # Attach volumes if present
    for VOL in $VOLUMES; do
        LOCALVOL=$(echo $VOL | tr : " " | awk '{print $1}')
        if [[ -e $LOCALVOL ]]; then
            DOCKER_VOLUMES="$DOCKER_VOLUMES -v $VOL"
        fi
    done

    # Always restart container on robot hardware
    if [[ `hostname` == $ROBOT_HOSTNAME ]]; then
        RESTART="--restart always"
    fi

    NETWORK="--network=host --env ROS_HOSTNAME=`hostname`"
    DOCKER_DEVICES="--privileged"
fi


# -----------------------
# Re-create the container
# -----------------------

# Clean up any previous container
ID=`docker ps -aqf "name=$CONTAINER_NAME"`
if [[ -n $ID ]]; then
    echo "Removing previous container $ID"
    docker rm -f $CONTAINER_NAME 1>/dev/null
fi

echo "Starting container..."
docker run $OPTIONS \
--name $CONTAINER_NAME \
--hostname $CONTAINER_NAME \
--mount type=bind,source=$REPOS_DIR,target=$CODE_MOUNT \
-e DISPLAY=$DISPLAY \
$DOCKER_VOLUMES \
$DOCKER_DEVICES \
$NETWORK \
$RESTART \
$IMAGE_NAME


# ---------------------------
# Attach an interactive shell
# ---------------------------
# Assume we would like to be in an attached shell afterwards
# Only run if container started in daemon mode
if [[ -z $1 ]]; then
sleep 2     # Ensure the entry script starts before attaching the shelle
docker exec -it $CONTAINER_NAME bash
fi
