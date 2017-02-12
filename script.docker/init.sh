#!/bin/bash

set -o errexit
set -o verbose

source /opt/ros/${ROS_DISTRO}/setup.bash

mkdir -p /catkin_ws/src
cd /catkin_ws/src
catkin_init_workspace

cd ..

catkin_make

