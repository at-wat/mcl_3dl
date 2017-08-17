#!/bin/bash

set -o errexit
set -o verbose

wget -q -P /tmp https://raw.githubusercontent.com/at-wat/gh-pr-comment/master/gh-pr-comment.sh
source /tmp/gh-pr-comment.sh

source /opt/ros/${ROS_DISTRO}/setup.bash
source /catkin_ws/devel/setup.bash

cd /catkin_ws

mkdir -p /catkin_ws/build/mcl_3dl/test/
mv /catkin_ws/src/mcl_3dl/.cached-dataset/* /catkin_ws/build/mcl_3dl/test/
ls -lh /catkin_ws/build/mcl_3dl/test/

apt-get -qq update && \
apt-get install libxml2-utils && \
rosdep install --from-paths src/mcl_3dl --ignore-src --rosdistro=${ROS_DISTRO} -y && \
apt-get clean && \
rm -rf /var/lib/apt/lists/*

catkin_make || (gh-pr-comment FAILED '```catkin_make``` failed'; false)
catkin_make tests --cmake-args -DMCL_3DL_EXTRA_TESTS:=ON || (gh-pr-comment FAILED '```catkin_make tests``` failed'; false)
catkin_make run_tests  --cmake-args -DMCL_3DL_EXTRA_TESTS:=ON || (gh-pr-comment FAILED '```catkin_make run_tests``` failed'; false)

if [ catkin_test_results ];
then
  result_text="
\`\`\`
`catkin_test_results --all || true`
\`\`\`
`find build/test_results/ -name *.xml | xargs -n 1 -- bash -c 'echo; echo \#\#\# $0; echo; echo \\\`\\\`\\\`; xmllint --format $0; echo \\\`\\\`\\\`;'`
"
else
  result_text="
\`\`\`
`catkin_test_results --all || true`
\`\`\`
"
fi
catkin_test_results || (gh-pr-comment FAILED "Test failed$result_text"; false)

gh-pr-comment PASSED "All tests passed$result_text"

cd ..
rm -rf /catkin_ws || true

