#!/bin/bash

set -o errexit
set -o verbose

docker pull ${DOCKER_CACHE_TARGET}:${ROS_DISTRO_TARGET} || true
