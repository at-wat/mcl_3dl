#!/bin/bash

set -eu

if [[ ! ${TRAVIS_BRANCH} =~ ^release-.*$ ]]; then
  echo "Skipping prerelease test"
  exit 0
fi

echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update -qq
sudo apt-get install -y python3-ros-buildfarm

mkdir -p /tmp/prerelease_job
cd /tmp/prerelease_job

generate_prerelease_script.py \
  https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml \
  ${ROS_DISTRO_TARGET} default ubuntu xenial amd64 \
  --custom-repo \
    mcl_3dl__custom-2:git:https://github.com/at-wat/mcl_3dl.git:${TRAVIS_BRANCH} \
  --level 1 \
  --output-dir ./ \
  && gh-pr-comment "[prerelease #${TRAVIS_BUILD_NUMBER}] PASSED on ${ROS_DISTRO_TARGET}" "" \
  || (gh-pr-comment "[prerelease #${TRAVIS_BUILD_NUMBER}] FAILED on ${ROS_DISTRO_TARGET}" ""; false)
