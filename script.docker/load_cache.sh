#!/bin/bash

set -o errexit
set -o verbose

apt-get install lz4

if [ -f ${DOCKER_CACHE_FILE} ]; then
	lz4 -dc ${DOCKER_CACHE_FILE} | docker load
fi

