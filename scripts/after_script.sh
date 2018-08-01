echo "==== catkin_make ===="
pushd $CATKIN_WORKSPACE
rm -Rf build devel
catkin_make
source $CATKIN_WORKSPACE/devel/setup.bash
echo

echo "==== $CATKIN_WORKSPACE/src ===="
ls -l $CATKIN_WORKSPACE/src
echo

echo "==== whoami ===="
whoami
echo

PKGS="b2 roboclaw_driver"
for P in $PKGS; do
    echo "===== rospack find $P ===="
    rospack find $P
    echo
done

echo "==== install pytest ===="
pip install -q pytest pytest-cov coveralls
echo

echo "==== run_tests ===="
source `rospack find b2`/tests/run_tests.sh

# echo "==== coveralls ===="
# coveralls
