#!/bin/bash

# DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PKG_DIR=`rospack find b2_base`

echo
echo "============================"
echo "     Running Unit Tests     "
echo "============================"
echo
/usr/bin/env pytest -v --cache-clear --cov=$PKG_DIR/src $PKG_DIR/tests/unit/

echo
echo "============================"
echo "     Running Node Tests     "
echo "============================"
echo
rostest b2_base base.test
rostest b2_base sensors.test
rostest b2_base pilot.test
