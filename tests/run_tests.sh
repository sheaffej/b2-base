#!/bin/bash

# DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PKG_DIR=`rospack find b2`

echo
echo "============================"
echo "     Running Unit Tests     "
echo "============================"
echo
pytest -v --cache-clear --cov=b2_logic $PKG_DIR/tests/unit/

echo
echo "============================"
echo "     Running Node Tests     "
echo "============================"
echo
rostest b2 base.test
rostest b2 sensors.test
