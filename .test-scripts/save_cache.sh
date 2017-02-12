#!/bin/bash

set -o errexit
set -o verbose

if [[ ${TRAVIS_BRANCH} == "master" ]] && [[ ${TRAVIS_PULL_REQUEST} == "false" ]];
then
	mkdir -p $(dirname ${DOCKER_CACHE_FILE})
	docker save $(docker history -q docker-mcl3dl:latest | grep -v '<missing>') | lz4 -zcf - > ${DOCKER_CACHE_FILE}

	ls -lh ${DOCKER_CACHE_FILE}
fi

