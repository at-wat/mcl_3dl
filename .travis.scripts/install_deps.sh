#!/bin/bash

set -o errexit
set -o verbose

source /opt/ros/${ROS_DISTRO}/setup.bash
source /catkin_ws/devel/setup.bash

cd /catkin_ws

apt-get -qq update && \
apt-get install libxml2-utils && \
rosdep install --from-paths src/mcl_3dl --ignore-src --rosdistro=${ROS_DISTRO} -y && \
apt-get clean && \
rm -rf /var/lib/apt/lists/*
