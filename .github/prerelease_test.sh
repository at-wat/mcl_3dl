#!/bin/bash

# This file is automatically deployed from https://github.com/at-wat/.rospkg-assets.
# Please don't directly edit; update at-wat/.rospkg-assets instead.

set -eu

case ${ROS_DISTRO} in
  melodic )
    UBUNTU_DIST_TARGET=bionic
    ;;
  noetic )
    UBUNTU_DIST_TARGET=focal
    ;;
  * )
    echo "Unknown ROS_DISTRO: ${ROS_DISTRO}"
    exit 1
    ;;
esac

sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
  git \
  jq \
  python3-pip
python3 -m pip install --upgrade \
  pip \
  setuptools \
  wheel
python3 -m pip install \
  git+https://github.com/at-wat/ros_buildfarm.git@apt-get-us-east-1

mkdir -p /tmp/prerelease_job
cd /tmp/prerelease_job

git clone \
  --depth 1 \
  -b apt-get-us-east-1 \
  https://github.com/at-wat/ros_buildfarm.git ros_buildfarm


build_link="[${GITHUB_RUN_NUMBER}-prerelease]"

generate_prerelease_script.py \
  https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml \
  ${ROS_DISTRO} default ubuntu ${UBUNTU_DIST_TARGET} amd64 \
  --custom-repo $@ \
  --level 1 \
  --output-dir ./

yes | ./prerelease.sh \
  && gh-pr-comment "${build_link} PASSED on ${ROS_DISTRO}" "" \
  || (gh-pr-comment "${build_link} FAILED on ${ROS_DISTRO}" ""; false)
