#!/bin/bash

set -o errexit
set -o verbose

sudo apt-get -qq update
sudo apt-get install -y liblz4-tool

if [ -f ${DOCKER_CACHE_FILE} ]; then
	lz4 -dc ${DOCKER_CACHE_FILE} | docker load
fi

