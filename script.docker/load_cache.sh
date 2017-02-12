#!/bin/bash

set -o errexit
set -o verbose

lz4 -dc /tmp/docker-cache.lz4 | docker load

