#!/usr/bin/env bash
set -e

IMAGE_NAME="sheaffej/b2-base"
# CONTAINER_NAME="b2-base-tests"
PKG_DIR="/ros/src/b2_base"

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
UNIT=""
VOLUME="$VOLUME"
COVERALLS=""
TRAVIS_ENV=""

echo
while [ $# -gt 0 ]; do
    case $1 in
        "unit")
            UNIT="unit"
            echo "Running unit tests only"
            ;;
        "nomount")
            VOLUME=""
            echo "Using image code, vs locally mounted code"
            ;;
        "coveralls")
            COVERALLS="; cd $PKG_DIR; coveralls"
            TRAVIS_ENV="-e TRAVIS_JOB_ID -e TRAVIS_BRANCH -e COVERALLS_REPO_TOKEN"
            echo "Running coveralls after unit tests"
            ;;
        *)
            echo "Unknown argument" $1
    esac
    shift
done


echo
echo "============================"
echo "     Running Unit Tests     "
echo "============================"
echo
docker run --rm \
$VOLUME \
$TRAVIS_ENV \
$IMAGE_NAME \
bash -c "pytest -v --cov=$PKG_DIR/src $PKG_DIR/tests/unit/ $COVERALLS"

# Exit if only running unit tests
if [[ $UNIT == 'unit' ]]; then
    exit
fi

echo
echo "============================"
echo "     Running Node Tests     "
echo "============================"
echo
docker run --rm $VOLUME $IMAGE_NAME rostest b2 base.test
docker run --rm $VOLUME $IMAGE_NAME rostest b2 sensors.test
docker run --rm $VOLUME $IMAGE_NAME rostest b2 pilot.test
