#!/bin/bash

set -o errexit
set -o verbose

if [[ ${TRAVIS_BRANCH} == "master" ]] && [[ ${TRAVIS_PULL_REQUEST} == "false" ]];
then
	mkdir -p $(dirname ${DOCKER_CACHE_FILE})
	docker save $(echo ${DOCKER_CACHE_TARGETS} | xargs -n1 docker history -q | grep -v '<missing>') | lz4 -zcf - > ${DOCKER_CACHE_FILE}

	echo "------------"
	ls -lh $(dirname ${DOCKER_CACHE_FILE})
	echo "------------"
fi

