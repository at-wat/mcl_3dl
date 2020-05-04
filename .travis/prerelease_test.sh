#!/bin/bash

set -eu

if [[ ! "${TRAVIS_PULL_REQUEST_BRANCH}" =~ ^release-.*$ ]]; then
  echo "Skipping prerelease test"
  exit 0
fi

case ${ROS_DISTRO_TARGET} in
  kinetic )
    UBUNTU_DIST_TARGET=xenial
    ;;
  melodic )
    UBUNTU_DIST_TARGET=bionic
    ;;
  noetic )
    UBUNTU_DIST_TARGET=focal
    ;;
  * )
    echo "Unknown ROS_DISTRO_TARGET: ${ROS_DISTRO_TARGET}"
    exit 1
    ;;
esac

echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
for i in 1 2 3; do
  sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && break \
    || true
done
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends python3-ros-buildfarm python3-pip git
sudo pip3 install git+https://github.com/ros/catkin

mkdir -p /tmp/prerelease_job
cd /tmp/prerelease_job


git clone \
  --depth 1 \
  -b apt-get-us-east-1 \
  https://github.com/at-wat/ros_buildfarm.git ros_buildfarm

sudo pip3 install ./ros_buildfarm


generate_prerelease_script.py \
  https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml \
  ${ROS_DISTRO_TARGET} default ubuntu ${UBUNTU_DIST_TARGET} amd64 \
  --custom-repo \
    mcl_3dl__custom-2:git:https://github.com/at-wat/mcl_3dl.git:${TRAVIS_PULL_REQUEST_BRANCH} \
    mcl_3dl_msgs__custom-2:git:https://github.com/at-wat/mcl_3dl_msgs.git:master \
  --level 1 \
  --output-dir ./

yes | ./prerelease.sh \
  && gh-pr-comment "[#${TRAVIS_BUILD_NUMBER}-prerelease] PASSED on ${ROS_DISTRO_TARGET}" "" \
  || (gh-pr-comment "[#${TRAVIS_BUILD_NUMBER}-prerelease] FAILED on ${ROS_DISTRO_TARGET}" ""; false)
