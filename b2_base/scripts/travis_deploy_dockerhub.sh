#!/usr/bin/env bash

IMAGE_NAME="b2-base"
DOCKER_REPO="sheaffej/b2-base"

echo "$DOCKER_TOKEN" | docker login -u "$DOCKER_USERNAME" --password-stdin

case $TRAVIS_BRANCH in
    "master")
        docker tag ${IMAGE_NAME} ${DOCKER_REPO}:latest
        docker push ${DOCKER_REPO}:latest
        ;;
    # "dev")
    #     docker tag ${IMAGE_NAME} ${DOCKER_REPO}:dev
    #     docker push ${DOCKER_REPO}:dev
    #     ;;
    *)
        docker tag ${IMAGE_NAME} ${DOCKER_REPO}:${TRAVIS_BRANCH}
        docker push ${DOCKER_REPO}:${TRAVIS_BRANCH}
esac
