#!/bin/bash

set -o errexit
set -o verbose

function post_error()
{
	if [[ ${TRAVIS_PULL_REQUEST} != "false" ]];
	then
		curl -X POST -H 'Content-Type:application/json' -d "{\"body\":\"** Travis-CI status notifier\n\n$1\"}" \
			https://api.github.com/repos/${TRAVIS_REPO_SLUG}/pull/${TRAVIS_PULL_REQUEST}/comments?access_token=${TRAVIS_BOT_GITHUB_TOKEN}
	fi
}


source /opt/ros/${ROS_DISTRO}/setup.bash
source /catkin_ws/devel/setup.bash

cd /catkin_ws

mkdir -p /catkin_ws/build/mcl_3dl/test/
mv /catkin_ws/src/mcl_3dl/.cached-dataset/* /catkin_ws/build/mcl_3dl/test/
ls -lh /catkin_ws/build/mcl_3dl/test/

apt-get -qq update && \
rosdep install --from-paths src/mcl_3dl --ignore-src --rosdistro=${ROS_DISTRO} -y && \
apt-get clean && \
rm -rf /var/lib/apt/lists/*

catkin_make || post_error('```catkin_make``` failed') || false
catkin_make tests --cmake-args -DMCL_3DL_EXTRA_TESTS:=ON || post_error('```catkin_make tests``` failed') || false
catkin_make run_tests  --cmake-args -DMCL_3DL_EXTRA_TESTS:=ON || post_error('```catkin_make run_tests``` failed') || false

catkin_test_results || post_error('Test failed') || false

post_error('All tests passed')

cd ..
rm -rf /catkin_ws || true

