#!/bin/bash

set -o errexit
set -o verbose

if [ -f ${DOCKER_CACHE_FILE} ]; then
	lz4 -dc ${DOCKER_CACHE_FILE} | docker load
fi

