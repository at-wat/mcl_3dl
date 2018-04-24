#!/bin/bash

set -o errexit
set -o verbose

if [[ ${TRAVIS_BRANCH} == "master" ]] && [[ ${TRAVIS_PULL_REQUEST} == "false" ]];
then
  docker tag ${DOCKER_CACHE_TARGET}:${ROS_DISTRO_TARGET} ${DOCKER_CACHE_REGISTRY}:${ROS_DISTRO_TARGET}
  docker login -u ${DOCKER_HUB_USER} -p ${DOCKER_HUB_TOKEN}
	docker push ${DOCKER_CACHE_REGISTRY}:${ROS_DISTRO_TARGET}
fi
