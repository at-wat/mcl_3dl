#!/bin/bash

set -o errexit
set -o verbose

source /opt/ros/${ROS_DISTRO}/setup.bash
source /catkin_ws/devel/setup.bash

cd /catkin_ws

rosdep install --from-paths src/mcl_3dl --ignore-src --rosdistro=${ROS_DISTRO} -y

apt-get clean && rm -rf /var/lib/apt/lists/*

catkin_make
catkin_make tests --cmake-args -DMCL_3DL_EXTRA_TESTS:=ON
catkin_make run_tests  --cmake-args -DMCL_3DL_EXTRA_TESTS:=ON

catkin_test_results

