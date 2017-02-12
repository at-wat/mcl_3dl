#!/bin/bash

set -o errexit
set -o verbose

docker save $(docker history -q docker-mcl3dl:latest | grep -v '<missing>') | lz4 -zcf - > /tmp/docker-cache.lz4

