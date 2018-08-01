#!/bin/bash

if [[ $USER == "root" ]]; then 
    # For industrial_ci's docker-based builds
    # since the repo file system is mounted read-only
    cd /root >/dev/null
fi

# DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PKG_DIR=`rospack find b2`


echo
echo "============================"
echo "     Running Unit Tests     "
echo "============================"
echo
pytest -v --cache-clear --cov=b2_logic $PKG_DIR/tests/unit/
