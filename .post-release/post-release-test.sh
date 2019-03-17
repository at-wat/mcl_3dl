#!/bin/bash

if [ $# -lt 1 ]
then
  echo "$0 ros-distro"
  exit 0
fi

export ROS_DISTRO=$1
echo "===================================="
echo "Testing shadow-fixed for $ROS_DISTRO"
echo "PLEASE MAKE SURE TO RUN \"xhost +\""
echo "===================================="

set -e

docker build -t mcl-3dl-shadow-fixed:${ROS_DISTRO} --build-arg ROS_DISTRO --build-arg CACHE_CLEAN=`date +%s` .
docker-compose up
